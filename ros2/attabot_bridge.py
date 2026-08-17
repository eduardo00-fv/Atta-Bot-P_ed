#!/usr/bin/env python3
"""
attabot_bridge — expone las poses de la Base de AttaBot como topics de ROS 2.

No reemplaza nada: escucha por UDP el espejo de poses que manda la Base
(`telemetry_port` en configSystem.json) y lo traduce. La Base, el firmware y
Webots siguen hablando su protocolo de siempre.

Como el espejo sale de la Base, da igual si las poses vienen de la camara real
o de los controllers de Webots: para ROS ambas fuentes son indistinguibles.

Publica:
    /attabot/<id>/pose   geometry_msgs/PoseStamped  — pose por robot
    /attabot/markers     visualization_msgs/MarkerArray — cuerpos + etiquetas + arena
    TF: map -> attabot_<id>

Unidades: la Base trabaja en mm y grados; ROS exige metros y radianes
(REP-103). La conversion se hace aca, que es la unica frontera donde debe
ocurrir.

Uso (dentro del contenedor):
    python3 /ws/Atta-Bot-P_ed/ros2/attabot_bridge.py
"""
import json
import math
import socket
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

# Ruta del config de la Base, montado en el contenedor por run.sh.
CONFIG = Path(__file__).resolve().parent.parent / 'Base' / 'configSystem.json'

# Un robot deja de dibujarse si la Base no reporta su pose en este tiempo.
# La camara pierde markers unos segundos de vez en cuando; sin esto quedarian
# robots fantasma clavados en su ultima posicion.
STALE_S = 2.0

# Lado del cuerpo dibujado (m). Aproximado: solo afecta lo que se ve.
ROBOT_SIZE_M = 0.15


def _quat_from_yaw(yaw_rad):
    """Cuaternion de una rotacion pura en Z. ROS no usa angulos de Euler."""
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


class AttaBotBridge(Node):

    def __init__(self):
        super().__init__('attabot_bridge')

        cfg = json.loads(CONFIG.read_text())
        self.port = cfg['udp_communication'].get('telemetry_port', 6070)
        self.arena_mm = cfg['scenario']['arena_mm']
        self.names = {int(k): v['name'] for k, v in cfg['robots'].items()}

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('127.0.0.1', self.port))
        self.sock.setblocking(False)

        self.tf = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, '/attabot/markers', 10)
        self.pose_pubs = {}          # id -> publisher, se crean al ver cada robot
        self.poses = {}              # id -> (x_mm, y_mm, ang_deg, t_visto)

        # Drenar el socket rapido (la Base manda hasta ~20 Hz por robot) y
        # redibujar a 20 Hz. Separados a proposito: recibir y publicar son
        # ritmos distintos.
        self.create_timer(0.01, self._drain_udp)
        self.create_timer(0.05, self._publish)

        self.get_logger().info(
            f'escuchando UDP en 127.0.0.1:{self.port} | '
            f'arena {self.arena_mm[0]}x{self.arena_mm[1]} mm')

    def _slug(self, rid):
        """Nombre usable como topic: ROS no admite segmentos que empiecen con
        numero, asi que '1' no sirve pero 'atta_1' si. Se reusa el nombre de
        configSystem.json para que el topic diga lo mismo que los logs."""
        return self.names.get(rid, f'robot_{rid}').lower()

    def _drain_udp(self):
        """Vacia el socket. Formato: '<id>.POSE|x|y|ang' (la gramatica del lab)."""
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
            except BlockingIOError:
                return
            try:
                dest, _, args = data.decode().partition('.')
                verb, _, payload = args.partition('|')
                if verb != 'POSE':
                    continue
                x, y, ang = (float(v) for v in payload.split('|')[:3])
                self.poses[int(dest)] = (x, y, ang, time.time())
            except (ValueError, UnicodeDecodeError):
                # Un datagrama malformado no debe tumbar el puente.
                continue

    def _publish(self):
        now = time.time()
        stamp = self.get_clock().now().to_msg()

        # Descartar robots que la camara dejo de ver.
        for rid in [r for r, p in self.poses.items() if now - p[3] > STALE_S]:
            del self.poses[rid]

        markers = MarkerArray()
        markers.markers.append(self._arena_marker(stamp))

        for rid, (x_mm, y_mm, ang_deg, _) in self.poses.items():
            # mm -> m, grados -> radianes. Unica frontera de unidades.
            x, y, yaw = x_mm / 1000.0, y_mm / 1000.0, math.radians(ang_deg)
            qx, qy, qz, qw = _quat_from_yaw(yaw)

            tfm = TransformStamped()
            tfm.header.stamp = stamp
            tfm.header.frame_id = 'map'
            tfm.child_frame_id = f'attabot_{rid}'
            tfm.transform.translation.x = x
            tfm.transform.translation.y = y
            tfm.transform.rotation.x = qx
            tfm.transform.rotation.y = qy
            tfm.transform.rotation.z = qz
            tfm.transform.rotation.w = qw
            self.tf.sendTransform(tfm)

            if rid not in self.pose_pubs:
                self.pose_pubs[rid] = self.create_publisher(
                    PoseStamped, f'/attabot/{self._slug(rid)}/pose', 10)
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = 'map'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            self.pose_pubs[rid].publish(ps)

            markers.markers.extend(self._robot_markers(stamp, rid, x, y, qz, qw))

        self.marker_pub.publish(markers)

    def _robot_markers(self, stamp, rid, x, y, qz, qw):
        """Cuerpo del robot + etiqueta con su nombre."""
        body = Marker()
        body.header.stamp = stamp
        body.header.frame_id = 'map'
        body.ns = 'cuerpos'
        body.id = rid
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.pose.position.x = x
        body.pose.position.y = y
        body.pose.position.z = ROBOT_SIZE_M / 2.0
        body.pose.orientation.z = qz
        body.pose.orientation.w = qw
        body.scale.x = body.scale.y = body.scale.z = ROBOT_SIZE_M
        # Color estable por id, para reconocerlos entre corridas.
        body.color.r = 0.2 + 0.8 * ((rid * 97) % 10) / 10.0
        body.color.g = 0.2 + 0.8 * ((rid * 53) % 10) / 10.0
        body.color.b = 0.9
        body.color.a = 0.9

        label = Marker()
        label.header.stamp = stamp
        label.header.frame_id = 'map'
        label.ns = 'etiquetas'
        label.id = rid
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = x
        label.pose.position.y = y
        label.pose.position.z = ROBOT_SIZE_M + 0.08
        label.pose.orientation.w = 1.0
        label.scale.z = 0.09
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = self.names.get(rid, f'id {rid}')
        return [body, label]

    def _arena_marker(self, stamp):
        """Contorno de la arena util, para tener referencia de escala."""
        from geometry_msgs.msg import Point
        w, h = self.arena_mm[0] / 1000.0, self.arena_mm[1] / 1000.0

        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = 'map'
        m.ns = 'arena'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.01
        m.color.r = m.color.g = m.color.b = 0.6
        m.color.a = 0.8
        # float() explícito: los campos de mensajes ROS rechazan int.
        m.points = [Point(x=float(px), y=float(py), z=0.0) for px, py in
                    ((0, 0), (w, 0), (w, h), (0, h), (0, 0))]
        return m


def main():
    rclpy.init()
    node = AttaBotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
