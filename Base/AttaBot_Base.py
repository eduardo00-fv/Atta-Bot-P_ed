import os
os.environ['QT_QPA_PLATFORM'] = 'xcb'  # forzar xcb — cv2 no tiene plugin wayland
import cv2, json, math, re, sys, time, socket, threading, csv, multiprocessing, threading, platform
import numpy as np
import readline
from datetime import datetime

# GUI opcional — se usa cuando se llama vía AttaBot_GUI.launch()
_gui_instance = None


def runOnThread(func):
    """
    Ejecuta una función en un hilo separado, permitiendo que el código principal
    continúe sin bloquearse.

    La función decorada se ejecutará en un hilo en segundo plano,
    usando el modo daemon para que el hilo termine automáticamente al finalizar
    el programa principal.

    Parámetros:
    func (callable): La función que se desea ejecutar en un hilo.

    Retorna:
    function: Un decorador que inicia un hilo para ejecutar la función `func`
    con los argumentos proporcionados y devuelve el hilo en ejecución.
    """
    def wrapper(*args, **kwargs):
        def executeFunc():
            func(*args, **kwargs)

        thread = threading.Thread(target=executeFunc, daemon=True)
        thread.start()

        return thread

    return wrapper


# Colores BGR por ID de robot para el mapa de cobertura
_ROBOT_COLORS_BGR = [
    (220,  70,  30),  # 0: azul
    ( 30, 180,  30),  # 1: verde
    ( 30,  30, 210),  # 2: rojo
    (190,  40, 190),  # 3: magenta
    ( 20, 190, 190),  # 4: amarillo
    ( 20, 130, 220),  # 5: naranja
]


def videoWriter(frameResolution, numRobots, pathVideo, processInterval, queue, debugResolution):
    """
    Graba un video en el disco utilizando los cuadros de video que llegan a través de una cola,
    mostrando además el video en una ventana.

    Este proceso lee pares de fotogramas de `queue`, une los fotogramas en uno solo y
    los guarda en un archivo AVI con un nombre específico que incluye la cantidad de robots y la fecha y hora actuales.
    También muestra el video en pantalla.

    Parámetros:
    frameResolution (tuple): Resolución original de los fotogramas de entrada (ancho, alto).
    numRobots (int): Número de robots, usado para el nombre del archivo de video.
    pathVideo (str): Directorio donde se guardará el video generado.
    processInterval (float): Intervalo de procesamiento en segundos para calcular los FPS.
    queue (Queue): Cola que contiene los fotogramas a grabar, en formato `(frame, resultsFrame)`.
    debugResolution (tuple): Resolución para mostrar el video de depuración (ancho, alto).

    Retorna:
    None
    """
    currentTime = datetime.now().strftime(r'%d-%m_%H-%M')
    videoName = f'Video_{currentTime}_Robots_{numRobots}.avi'
    pathVideo = os.path.join(pathVideo, videoName)
    resolution = (frameResolution[1], frameResolution[0] * 2)
    fps = 1 / processInterval - 1
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    video = cv2.VideoWriter(pathVideo, fourcc, fps, resolution)

    while True:
        frames = queue.get()

        if frames is None:
            break

        frame, resultsFrame = frames

        # Solo grabación — el display lo maneja la GUI o el proceso principal
        results = cv2.vconcat([frame, resultsFrame])
        video.write(results)

    video.release()


# =============================================================================
# CLASE COLOR ELIMINADA
# El sistema de detección por círculos de color HSV fue reemplazado por
# detección de ArUco markers. La identificación del robot se realiza
# directamente por el ID del marker, eliminando la necesidad de segmentación
# por color y el cálculo de centroides.
# =============================================================================


class Robot(object):
    """
    Clase que representa un robot en un sistema de enjambre.

    La detección de pose se realiza mediante ArUco markers.
    El ID del marker ArUco corresponde directamente al ID del robot.

    Atributos:
        id (str): Identificador único del robot (igual al ID del marker ArUco).
        name (str): Nombre del robot.
        IP (str): Dirección IP del robot para la comunicación.
        previousPose (tuple): Última posición conocida del robot en formato (x, y, angle).
    """

    def __init__(self, id, configRobot):
        self.id = id
        self.name = ''
        self.IP = ''
        self.previousPose = (-1, -1, -1)
        # Última pose del EKF del firmware (EKF_POSE, 2Hz) y su timestamp. Es
        # telemetría pasiva: el EKF no controla nada mientras EKF_NAV esté
        # apagado. Se registra junto a la pose de ArUco para poder medir su
        # deriva antes de decidir si se le confía la navegación.
        self.ekfPose = None               # (x, y, angle) o None si nunca llegó
        self.ekfStamp = 0.0               # time.time() de la última recepción
        self.initRobot(configRobot)


    def getPose(self):
        """
        Obtiene la pose actual del robot desde el diccionario de detecciones ArUco.

        El ID del marker ArUco coincide con el ID del robot, por lo que la
        identificación es directa sin necesidad de comparar colores ni distancias.

        Returns:
            tuple: (x_mm, y_mm, angle_deg) si el robot es visible, (-1, -1, -1) si no.
        """
        if self.id in base.currentArucoDetections:
            return base.currentArucoDetections[self.id]
        return (-1, -1, -1)


    def initRobot(self, configRobot):
        """
        Inicializa el robot configurando su nombre y establece su pose inicial.

        Args:
            configRobot (dict): Diccionario de configuración de los robots.
        """
        self.name = configRobot[self.id]['name']
        self.angleOffset = float(configRobot[self.id].get('angle_offset', 0.0))
        wd = configRobot[self.id].get('wheel_distance')
        self.wheelDistance = float(wd) if wd is not None else None
        self.previousPose = self.getPose()


    def getDisplacement(self):
        """
        Calcula el desplazamiento lineal y angular del robot en función de su
        pose actual y anterior.

        Este método obtiene la pose actual del robot y si es válida, calcula
        la diferencia entre la pose actual y la anterior. Devuelve el desplazamiento
        lineal y angular si estas diferencias superan un umbral definido.

        Returns:
            tuple or None: Un tuple que contiene el desplazamiento lineal y
                            angular en milímetros y grados respectivamente,
                            o None si la pose actual no es válida o si los
                            desplazamientos son menores al umbral.

        Note:
            Se considera que un desplazamiento es significativo si la diferencia
            angular es mayor o igual a 4 grados o si la diferencia lineal es
            mayor o igual a 4 milímetros.
        """
        currentPose = self.getPose()
        if currentPose == (-1, -1, -1):
            return None

        x1, y1, a1 = self.previousPose
        x2, y2, a2 = currentPose

        linearDisp = self.linearDisplacement(x1, y1, a1, x2, y2)
        angularDisp = self.angularDisplacement(a1, a2)

        if abs(angularDisp) >= 4 or abs(linearDisp) >= 4:
            self.previousPose = currentPose
            return linearDisp, angularDisp

        return None


    def angularDisplacement(self, a1, a2):
        """
        Calcula el desplazamiento angular entre dos ángulos.

        Asegura que el resultado se mantenga en el rango de -180 a 180 grados.

        Args:
            a1 (float): El ángulo inicial en grados.
            a2 (float): El ángulo final en grados.

        Returns:
            float: El desplazamiento angular en grados, re1dondeado a un decimal.
        """
        angularDisplacement = a2 - a1
        if angularDisplacement > 180:
            angularDisplacement -= 360
        elif angularDisplacement < -180:
            angularDisplacement += 360

        return round(angularDisplacement, 1)


    def linearDisplacement(self, x1, y1, a1, x2, y2):
        """
        Calcula el desplazamiento lineal entre dos posiciones dadas,
        teniendo en cuenta la dirección del ángulo de la primera posición.

        Args:
            x1 (float): Coordenada x de la primera posición.
            y1 (float): Coordenada y de la primera posición.
            a1 (float): Ángulo en grados de la primera posición.
            x2 (float): Coordenada x de la segunda posición.
            y2 (float): Coordenada y de la segunda posición.

        Returns:
            float: El desplazamiento lineal entre las dos posiciones, redondeado a
                un decimal. Positivo = misma dirección que el ángulo.
        """
        a1Rad = math.radians(a1)
        dx = x2 - x1
        dy = y2 - y1

        dotProduct = dx * math.cos(a1Rad) + dy * math.sin(a1Rad)
        displacement = round(math.dist((x1, y1), (x2, y2)), 1)

        return displacement if dotProduct >= 0 else -displacement


    def setupIP(self, ip):
        """
        Configura la dirección IP del robot y envía una instrucción de configuración.

        Args:
            ip (str): La dirección IP que se asignará al robot.
        """
        self.IP = ip
        instructions = [f'CONFIG|{self.id}']
        if self.wheelDistance is not None:
            instructions.append(f'NAV_CONFIG|WHEEL_DIST|{self.wheelDistance}')
        # Arena del escenario en curso. Sin esto el firmware usa su default
        # hardcodeado (2400x1750) para decidir el slot seguro del anillo, el lado
        # del escape de deadlock y si un destino de GT es válido, así que en otro
        # montaje esos límites son mentira.
        # Va la arena FÍSICA, no el recorte de la cámara: antes se mandaba el FOV
        # y el firmware rechazaba destinos alcanzables que la cámara no alcanza a
        # ver (2200|850 con FOV de 2170mm). Ver Base.arenaMm().
        arenaW, arenaH = base.arenaMm()
        instructions.append(f'NAV_CONFIG|ARENA|{arenaW:.0f}|{arenaH:.0f}')
        base.sendInstruction(ip, instructions, False)



class SimVision(object):
    """
    Fuente de visión para el modo simulación (--sim).

    Reemplaza a la cámara física: recibe por UDP los paquetes 'CAM|id,x,y,ang;...'
    que emite base_camera.py (el supervisor de Webots en modo solo-cámara) y
    sintetiza un frame BGR equivalente para el resto del pipeline (video, mapa
    de cobertura, debug). Las detecciones ya vienen en el marco de cámara del
    lab (mm, y hacia abajo, ángulo CW) y CON el jitter ArUco aplicado por el
    supervisor según robot_profiles.json — aquí no se agrega ruido.

    La oclusión de cámara se simula con paquetes 'CAM|' vacíos (equivale a
    tapar el lente: llegan frames pero sin markers detectados).
    """

    def __init__(self, base, visionPort, controlAddr):
        self.base = base
        self.controlAddr = controlAddr
        self.detections = {}
        self.lastPacketTime = 0.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(('127.0.0.1', visionPort))
        except OSError as e:
            raise Exception(f'Puerto de visión sim {visionPort} ocupado '
                            f'(¿otra base --sim corriendo?): {e}')
        print(f'✓ Visión sim escuchando en 127.0.0.1:{visionPort} '
              f'(feed CAM de base_camera.py)')
        self._recvLoop()

    @runOnThread
    def _recvLoop(self):
        """Actualiza las detecciones con cada paquete CAM del supervisor."""
        while True:
            data, _ = self.sock.recvfrom(2048)
            msg = data.decode().strip()
            if not msg.startswith('CAM|'):
                continue
            detections = {}
            body = msg[4:]
            if body:
                for item in body.split(';'):
                    rid, x, y, ang = item.split(',')
                    detections[rid] = (round(float(x), 1), round(float(y), 1),
                                       round(float(ang), 1))
            self.detections = detections
            self.lastPacketTime = time.time()

    def snapshot(self):
        """Detecciones vigentes. Feed muerto >1s (Webots pausado/cerrado) = vacío."""
        if time.time() - self.lastPacketTime > 1.0:
            return {}
        return dict(self.detections)

    def read(self):
        """Equivalente de camera.read(): sintetiza el frame de la escena."""
        time.sleep(0.02)   # pace mínimo; el gate de processInterval hace el resto
        h, w = self.base.cameraResolution
        frame = np.full((h, w, 3), 235, dtype=np.uint8)
        mm = self.base.mmPixel
        for rid, (x, y, ang) in self.snapshot().items():
            px, py = int(x / mm), int(y / mm)
            if not (0 <= px < w and 0 <= py < h):
                continue
            color = _ROBOT_COLORS_BGR[int(rid) % len(_ROBOT_COLORS_BGR)]
            r = max(4, int(75 / mm))   # radio del cuerpo del AttaBot
            cv2.circle(frame, (px, py), r, color, 2)
            hx = px + int(r * 1.6 * math.cos(math.radians(ang)))
            hy = py + int(r * 1.6 * math.sin(math.radians(ang)))
            cv2.line(frame, (px, py), (hx, hy), color, 2)
            cv2.putText(frame, rid, (px - 5, py + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        return True, frame

    def sendControl(self, message):
        """Comando al supervisor de Webots (OCCLUDE.n, COLOR_QUERY.rid)."""
        self.sock.sendto(message.encode(), self.controlAddr)


class Base(object):
    """
    Clase Base para la gestión de un sistema de robots de enjambre con detección ArUco.

    El sistema de visión utiliza markers ArUco (DICT_4X4_50) para identificar y
    localizar los robots. Cada robot lleva un marker cuyo ID corresponde al ID del robot.
    La pose (x, y, ángulo) se calcula con solvePnP a partir de las esquinas detectadas.

    Atributos principales:
        arucoDetector: Detector de markers ArUco inicializado con DICT_4X4_50.
        markerSizeMm (float): Tamaño físico del marker en mm (configurado en JSON).
        currentArucoDetections (dict): {robot_id (str): (x_mm, y_mm, angle_deg)}
            Actualizado en cada frame procesado. Accedido por Robot.getPose().
        robots (dict): Diccionario {id: Robot} de robots activos.
        [resto de atributos igual que antes]
    """

    def __init__(self):
        self.robots = {}
        self.robotsConfig = {}
        self.numRobots = int
        self.debug = False
        self.camera = None
        self.cameraIndex = None
        self.cameraBackend = None
        self.debugResolution = tuple
        self.mmPixel = float
        self.processInterval = float
        self.startTime = float
        self.cameraMatriz = []
        self.distance = []
        self.sock = None
        self.baseIP = ''
        self.broadcastIP = ''
        self.port = int
        self.threadInputAlive = True
        self.pathVideo = ''
        self.pathPositionLogs = ''
        self.pathConsolelog = ''
        self.cameraResolution = []
        self.newCameraMatriz = None
        self.roi = None
        self.frameQueue = None
        self.videoProcess = multiprocessing.Process()
        self.congregationActive = False
        self.leaderID = None
        self.robotPositions = {}
        # --- ArUco ---
        self.arucoDetector = None
        self.markerSizeMm = 80.0          # valor por defecto, sobreescrito desde JSON
        self.currentArucoDetections = {}    # raw: {robot_id: (x_mm, y_mm, angle_deg)}
        self._smoothedArucoDetections = {} # EMA-suavizado, solo para enviar posiciones al robot
        self._arucoEma = {}               # estado interno del EMA
        self.arucoEmaAlpha = 0.4          # peso del frame nuevo (0=sin cambio, 1=sin suavizado)
        self.bigCircleRadius = 10         # radio visual en el resultsFrame (px)
        self.cellSizeMm = 50.0            # tamaño de celda del mapa de cobertura en mm
        self.coverageGrid = None          # grilla de cobertura: -1=libre, else robot_id
        self.cellPx = 1                   # tamaño de celda en píxeles
        self.gui = None                   # referencia a AttaBotGUI (None = modo terminal)
        # --- Calibración por robot (CALIBRATE.robotId) ---
        self._calib = None                # estado de la rutina activa, None = inactiva
        # --- Modo simulación (--sim): visión desde Webots, robots en localhost ---
        self.simMode = False
        self.simVision = None             # instancia de SimVision
        self.simConfig = {}               # sección 'simulation' del JSON
        self.scenarioConfig = {}          # sección 'scenario' del JSON (arena física)
        # Cuánto esperar una detección ArUco buena antes de dejar sin responder un
        # REQUEST_POSITION. El ArUco titila por posición y el timeout del firmware
        # es de 5s, así que rendirse en el primer frame malo sale carísimo.
        self.poseWaitTimeout = 0.5        # s
        self.logTag = ''                  # 'SIM_' en los nombres de log de sim
        # --- Enjambre: broadcast periódico de posiciones (dispersión/flocking) ---
        self._lastNeighborCast = 0.0


    def arenaMm(self):
        """
        Arena FÍSICA del escenario en curso, en mm: (ancho, alto).

        Es dónde el robot PUEDE ESTAR, y no debe confundirse con el recorte que
        ve la cámara (cameraFovMm), que es dónde la base puede MEDIRLO. El FOV
        suele ser más chico: con la C920 a 1280px y 39/23 mm/px son 2170x1221mm
        contra una arena de 2400x1750. Mientras la arena la definía el FOV, un
        destino perfectamente alcanzable como 2200|850 lo rechazaba el firmware
        con 'GT objetivo fuera de la arena' (2026-07-29).

        Sale de la sección 'scenario': 'presets' por cantidad de robots si hay
        uno para este N, si no 'arena_mm'.
        """
        sc = self.scenarioConfig
        presets = sc.get('presets', {})
        preset = presets.get(str(self.numRobots))
        arena = preset if preset else sc.get('arena_mm', [2400, 1750])
        return float(arena[0]), float(arena[1])


    def cameraFovMm(self):
        """Recorte observable por la cámara, en mm: (ancho, alto). Ver arenaMm()."""
        return (self.cameraResolution[1] * self.mmPixel,
                self.cameraResolution[0] * self.mmPixel)


    def warnIfOutsideFov(self, instruction):
        """
        Avisa si una instrucción con destino apunta fuera de lo que ve la cámara.

        El destino es LEGAL mientras caiga en la arena física (ver arenaMm), pero
        si además cae fuera del FOV el robot llega a ciegas: la cámara deja de
        publicar su pose y se queda quieto esperando coordenadas. Con EKF_NAV|1
        sigue por odometría, sin eso se congela. Esto no bloquea nada — solo
        evita el diagnóstico equivocado de 'el robot se colgó'.
        """
        parts = instruction.split('|')
        if parts[0] not in ('GT', 'GOTO', 'POSITIONGT', 'MEET') or len(parts) < 3:
            return
        try:
            x, y = float(parts[1]), float(parts[2])
        except ValueError:
            return

        fovW, fovH = self.cameraFovMm()
        arenaW, arenaH = self.arenaMm()
        if not (0 <= x <= fovW and 0 <= y <= fovH):
            dentro = (0 <= x <= arenaW and 0 <= y <= arenaH)
            self.log(f'⚠ Destino ({x:.0f},{y:.0f}) fuera del FOV de la cámara '
                     f'({fovW:.0f}x{fovH:.0f}mm)'
                     + (f' pero dentro de la arena ({arenaW:.0f}x{arenaH:.0f}mm): '
                        'el robot va a perder la pose al llegar. Activá EKF_NAV|1.'
                        if dentro else
                        f'. Además está fuera de la arena ({arenaW:.0f}x{arenaH:.0f}mm): '
                        'el firmware lo va a rechazar.'))


    def log(self, msg: str):
        """Muestra un mensaje en el log de la GUI o en la terminal si no hay GUI."""
        if self.gui is not None:
            self.gui.logSignal.emit(msg)
        else:
            print(msg)


    # =========================================================================
    # DETECCIÓN ARUCO
    # =========================================================================

    def _medianGate(self, mid, x, y, ang):
        """Mediana de las últimas 3 lecturas del marker (ventana 0.5s).

        Un misread de UN frame (identidad confundida, esquina mal refinada)
        queda en minoría y no sale de acá; un cambio real sostenido gana la
        mediana al segundo frame. El ángulo se decide por distancia circular
        para no romperse en el wrap 359°↔1°. Costo: ~1 frame de retardo, y la
        navegación muestrea con el robot quieto, así que no le pesa.
        """
        import time as _t
        now = _t.time()
        hist = [h for h in self._poseHist.get(mid, []) if now - h[0] <= 0.5]
        hist.append((now, x, y, ang))
        self._poseHist[mid] = hist[-3:]
        if len(self._poseHist[mid]) < 3:
            return (x, y, ang)
        xs, ys, angs = zip(*[(h[1], h[2], h[3]) for h in self._poseHist[mid]])
        angMed = min(angs, key=lambda a: sum(
            abs((a - b + 180.0) % 360.0 - 180.0) for b in angs))
        return (sorted(xs)[1], sorted(ys)[1], angMed)


    def detectArucoMarkers(self, frame):
        """
        Detecta ArUco markers en el frame BGR y retorna poses en mm y grados.

        Reemplaza completamente el sistema de detección por círculos de color.
        Usa solvePnP con SOLVEPNP_IPPE_SQUARE para obtener posición 3D y orientación
        de cada marker visible. Las coordenadas se expresan en el plano de la cámara:
            - X positivo: derecha
            - Y positivo: abajo
            - El ángulo es el heading del robot en el plano XY (0° = derecha, CW positivo)

        Parámetros:
        - frame (ndarray): Frame BGR de la cámara ya corregido por distorsión.

        Retorna:
        - dict: {marker_id (str): (x_mm, y_mm, angle_deg)}
                Los IDs son strings para compatibilidad con el resto del sistema.
                Retorna {} si no se detecta ningún marker.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Detección a resolución completa: a 2.5m de altura el marker de 80mm ocupa
        # solo ~47px — a media resolución baja a ~24px (3.9px/celda), límite de fallo.
        corners, ids, _ = self.arucoDetector.detectMarkers(gray)

        detectedPoses = {}

        if ids is None:
            return detectedPoses

        # Puntos 3D del marker en coordenadas locales (metros)
        # Orden: top-left, top-right, bottom-right, bottom-left
        halfSize = (self.markerSizeMm / 1000.0) / 2.0
        objectPoints = np.array([
            [-halfSize,  halfSize, 0.0],
            [ halfSize,  halfSize, 0.0],
            [ halfSize, -halfSize, 0.0],
            [-halfSize, -halfSize, 0.0]
        ], dtype=np.float32)

        # Paso 1: resolver pose de todos los markers y buscar el de referencia
        rawPositions = {}  # {str(id): (raw_x_mm, raw_y_mm, angle_deg)}
        for i, marker_id in enumerate(ids.flatten()):
            imagePoints = corners[i][0].astype(np.float32)
            side = sum(float(np.linalg.norm(imagePoints[j] - imagePoints[(j + 1) % 4]))
                       for j in range(4)) / 4.0
            if side < getattr(self, 'minMarkerSidePx', 0.0):
                continue      # blob demasiado chico para ser un marker real
            success, rvec, tvec = cv2.solvePnP(
                objectPoints, imagePoints,
                self.cameraMatriz, self.distance,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not success:
                continue
            raw_x = float(tvec[0][0]) * 1000.0
            raw_y = float(tvec[1][0]) * 1000.0
            if getattr(self, 'angleFromCorners', False):
                # Dirección +x del marker medida sobre sus dos aristas
                # horizontales (TL→TR y BL→BR), en coordenadas sin distorsión.
                # No usa el rvec → inmune al flip de IPPE.
                und = cv2.undistortPoints(imagePoints.reshape(-1, 1, 2),
                                          self.cameraMatriz,
                                          self.distance).reshape(4, 2)
                ex, ey = (und[1] - und[0] + und[2] - und[3]) / 2.0
                angle_deg = round(float(np.degrees(np.arctan2(ey, ex))) % 360, 1)
            else:
                rotMatrix, _ = cv2.Rodrigues(rvec)
                angle_rad = np.arctan2(rotMatrix[1][0], rotMatrix[0][0])
                angle_deg = round(float(np.degrees(angle_rad) % 360), 1)
            rawPositions[str(marker_id)] = self._medianGate(
                str(marker_id), raw_x, raw_y, angle_deg)

        # Paso 2: si hay marker de referencia visible, anclar origen a él
        if self.referenceMarkerId and self.referenceMarkerId in rawPositions:
            ref_x, ref_y, _ = rawPositions[self.referenceMarkerId]
            self.originXmm = round(ref_x, 1)
            self.originYmm = round(ref_y, 1)

        # Paso 3: calcular poses relativas al origen
        for marker_id_str, (raw_x, raw_y, angle_deg) in rawPositions.items():
            if marker_id_str == self.referenceMarkerId:
                detectedPoses[marker_id_str] = (0.0, 0.0, angle_deg)
                continue
            x_mm = round(raw_x - self.originXmm, 1)
            y_mm = round(raw_y - self.originYmm, 1)
            # Aplicar offset de ángulo por robot (compensa marker montado rotado)
            offset = 0.0
            if marker_id_str in self.robots:
                offset = self.robots[marker_id_str].angleOffset
            corrected_angle = round((angle_deg + offset) % 360, 1)
            detectedPoses[marker_id_str] = (x_mm, y_mm, corrected_angle)

        return detectedPoses


    def drawArucoDebug(self, frame):
        """
        Dibuja los markers detectados sobre el frame para depuración visual.

        Muestra los ejes de coordenadas de cada marker y su ID.
        Solo se llama cuando self.debug está activado.

        Parámetros:
        - frame (ndarray): Frame BGR donde dibujar las anotaciones.

        Retorna:
        - ndarray: Frame anotado.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.arucoDetector.detectMarkers(gray)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            halfSize = (self.markerSizeMm / 1000.0) / 2.0
            objectPoints = np.array([
                [-halfSize,  halfSize, 0.0],
                [ halfSize,  halfSize, 0.0],
                [ halfSize, -halfSize, 0.0],
                [-halfSize, -halfSize, 0.0]
            ], dtype=np.float32)

            for i, marker_id in enumerate(ids.flatten()):
                imagePoints = corners[i][0].astype(np.float32)
                success, rvec, tvec = cv2.solvePnP(
                    objectPoints, imagePoints,
                    self.cameraMatriz, self.distance,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                if success:
                    cv2.drawFrameAxes(frame, self.cameraMatriz, self.distance,
                                      rvec, tvec, self.markerSizeMm / 1000.0 * 0.5)

                    # Texto con pose encima del marker
                    cx = int(np.mean(corners[i][0][:, 0]))
                    cy = int(np.mean(corners[i][0][:, 1]))
                    if str(marker_id) in self.currentArucoDetections:
                        x_mm, y_mm, ang = self.currentArucoDetections[str(marker_id)]
                        label = f'ID:{marker_id} ({x_mm},{y_mm}) {ang}deg'
                        cv2.putText(frame, label, (cx - 60, cy - 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 100), 1, cv2.LINE_AA)

        return frame


    # =========================================================================
    # BÚSQUEDA Y SETUP DE ROBOTS
    # =========================================================================

    def searchRobotsAruco(self, robots):
        """
        Detecta qué robots están visibles en el frame actual por su marker ArUco.

        Reemplaza searchRobotsColor. Con ArUco el ID del marker es directamente el
        ID del robot, por lo que no se necesita correlación de colores ni distancias.

        Parámetros:
        - robots (dict): Configuración de robots desde el JSON.

        Retorna:
        - foundRobots (set): IDs (str) de robots detectados visualmente.
        - foundColors (set): Set vacío — mantenido por compatibilidad con processFoundRobots.
        """
        foundRobots = set()

        for robotId in robots.keys():
            # El marker de referencia (origen del escenario) no es un robot
            if robotId == self.referenceMarkerId:
                continue
            if robotId in self.currentArucoDetections:
                foundRobots.add(robotId)

        return foundRobots, set()


    def processFoundRobots(self, foundRobots):
        """
        Crea instancias de Robot para cada robot encontrado y los ordena por ID.

        Versión simplificada para ArUco: ya no necesita eliminar colores no usados
        porque no existe el diccionario frameColors.

        Parámetros:
        - foundRobots (set): IDs de robots detectados visualmente.
        """
        self.robots = {robot: Robot(robot, self.robotsConfig) for robot in sorted(foundRobots, key=int)}
        if self.gui is not None:
            self.gui.refreshRobots()


    def setupRobots(self, robotIP, robotsIPs, configuredRobots):
        """
        Asocia una dirección IP a un robot en función de la detección del sistema de visión.

        Si no se proporciona una dirección IP se selecciona una de robotsIPs, después se
        gira el robot para identificar cuál se mueve y se le asigna la dirección IP
        seleccionada.

        Parameters:
        - robotIP (str o None): Dirección IP actual del robot.
        - robotsIPs (list): Lista de direcciones IP disponibles para asignar a los robots.
        - configuredRobots (set): Conjunto de IDs de robots ya configurados.

        Returns:
        - tuple: (robotIP, isValidFrame)
        """
        if robotIP is None:
            # Snapshot del ángulo de cada robot ANTES del giro de identificación.
            # Comparar contra el snapshot (y no frame-a-frame) evita que el giro
            # se vea "en cachitos" <45° cuando hay frames atrasados en el buffer.
            self._setupAngleSnapshot = {}
            for robot in self.robots.values():
                pose = robot.getPose()
                if pose != (-1, -1, -1):
                    self._setupAngleSnapshot[robot.id] = pose[2]
            robotIP = self.setupMoveRobot(robotsIPs)
            isValidFrame = False
        else:
            prevLen = len(robotsIPs)
            self.setupRobotIP(robotsIPs, configuredRobots)
            robotIP = None if len(robotsIPs) < prevLen else robotIP
            isValidFrame = True

        if len(robotsIPs) == 0:
            self.printRobots()
            time.sleep(0.5)

        return robotIP, isValidFrame


    def printRobots(self):
        """
        Imprime la lista de robots encontrados y envía una instrucción para que
        cada robot gire 90 grados en sentido antihorario.
        """
        print('Robots encontrados: ')
        for robot in self.robots.values():
            self.sendInstruction(robot.IP, ['TURN|-90'], False)
            print(f"\t{robot.id}. {robot.name}, con IP: {robot.IP}")


    def setupRobotIP(self, robotsIPs, configuredRobots):
        """
        Asocia una dirección IP a un robot en función de su desplazamiento angular.

        Con ArUco, el desplazamiento se detecta directamente desde el cambio de ángulo
        del marker, sin necesidad de comparar círculos de color.

        Parameters:
        - robotsIPs (list): Lista de direcciones IP disponibles para los robots.
        - configuredRobots (set): Conjunto de IDs de robots ya configurados.

        Returns:
        - None
        """
        for robot in self.robots.values():
            if robot.id in configuredRobots:
                continue

            pose = robot.getPose()
            snapshot = getattr(self, '_setupAngleSnapshot', {})
            refAngle = snapshot.get(robot.id)

            # Inicialización tardía: si el robot no era visible cuando se tomó el
            # snapshot, se quedaba con ref=None PARA SIEMPRE en este intento y el
            # desplazamiento nunca podía calcularse (visto 2026-07-27: el robot
            # giró 87° y el giro se descartó por esto). Al primer frame en que
            # aparezca, se ancla la referencia.
            if refAngle is None and pose != (-1, -1, -1):
                snapshot[robot.id] = pose[2]
                self._setupAngleSnapshot = snapshot
                continue

            now = time.time()
            if now - getattr(self, '_setupDbgTime', 0) >= 1.0:
                self._setupDbgTime = now
                visible = pose != (-1, -1, -1)
                print(f"[Setup-dbg] Robot {robot.id}: visible={visible} "
                      f"ref={refAngle} actual={pose[2] if visible else '—'} "
                      f"disp={robot.angularDisplacement(refAngle, pose[2]) if (visible and refAngle is not None) else '—'}")

            if pose == (-1, -1, -1) or refAngle is None:
                continue

            angularDisp = robot.angularDisplacement(refAngle, pose[2])
            if abs(angularDisp) >= 45:
                robotIP = robotsIPs.pop()
                robot.setupIP(robotIP)
                configuredRobots.add(robot.id)
                break

        return None


    def assignConfiguredAddresses(self, foundRobots, configuredRobots):
        """
        Asocia marker → IP desde el JSON, saltándose el giro de identificación.

        El giro de identificación existe para descubrir qué IP corresponde a qué
        marker, pero en el banco eso lo sabe el operador: es él quien pega el
        marker en el robot. Declarándolo en configSystem.json (robots.<id>.ip) la
        asociación es determinista, instantánea y —sobre todo— no depende de que
        el robot gire bien: un robot con el giro comprometido no se asociaba
        nunca, o peor, le robaba la IP al marker vecino.

        Solo se aplica si TODOS los markers detectados tienen 'ip' declarada; si
        falta alguna se cae al giro de identificación de siempre.

        Parámetros:
        - foundRobots (set/list): IDs de marker detectados por la cámara.
        - configuredRobots (set): Conjunto de IDs ya configurados (se llena aquí).

        Returns:
        - bool: True si asignó todas las direcciones (se puede omitir el giro).
        """
        ips = {}
        for rid in foundRobots:
            ip = self.robotsConfig.get(str(rid), {}).get('ip')
            if not ip:
                return False
            ips[str(rid)] = ip

        if len(set(ips.values())) != len(ips):
            print(f'[Setup] ⚠ IPs repetidas en configSystem.json: {ips} — '
                  f'se usa el giro de identificación')
            return False

        for robot in self.robots.values():
            if robot.id in ips:
                robot.setupIP(ips[robot.id])
                configuredRobots.add(robot.id)
        # No se usa printRobots(): ese manda TURN|-90 a cada robot, justamente
        # el giro que este camino busca evitar.
        print('[Setup] Identidad tomada de configSystem.json (sin giro):')
        for robot in self.robots.values():
            print(f'\t{robot.id}. {robot.name}, con IP: {robot.IP}')
        return True


    def assignSimAddresses(self, configuredRobots):
        """
        Asigna direcciones a los robots en modo simulación.

        En Webots cada controller escucha en 127.0.0.1:(puerto_base + id), así
        que la asociación id → dirección es directa, sin la rutina de giro
        del lab.

        Parámetros:
        - configuredRobots (set): Conjunto de IDs ya configurados (se llena aquí).
        """
        host = self.simConfig.get('robot_host', '127.0.0.1')
        portBase = int(self.simConfig.get('robot_port_base', self.port))
        for robot in self.robots.values():
            robot.setupIP(f'{host}:{portBase + int(robot.id)}')
            configuredRobots.add(robot.id)
        self.printRobots()


    def setupMoveRobot(self, robotsIPs):
        """
        Envía instrucciones para girar al último robot de la lista robotsIPs.

        Parameters:
        - robotsIPs (list): Lista de direcciones IP disponibles para los robots.

        Returns:
        - str: La dirección IP asignada al robot.
        """
        robotIP = robotsIPs[-1]
        instructions = ['TURN|90', 'MESSAGE_BASE|1']
        self.sendInstruction(robotIP, instructions, False)

        self.sock.settimeout(0.5)
        deadline = time.time() + 8.0
        while time.time() < deadline:
            try:
                data, addr = self.sock.recvfrom(1024)
                if addr[0] != self.baseIP and robotIP == addr[0]:
                    message = data.decode()
                    print(f"Respuesta de {addr[0]}: {message}")
                    if message == 'READY':
                        print(f"[Setup] Robot {robotIP} listo")
                        break
            except socket.timeout:
                pass

        return robotIP


    # =========================================================================
    # CONFIGURACIÓN
    # =========================================================================

    def readConfigFile(self, filePath):
        """
        Lee un archivo de configuración JSON y aplica las configuraciones del sistema.

        Cambios respecto al sistema de círculos:
        - Se elimina la sección 'colors' del JSON.
        - Se elimina 'circles_radius' y 'distance_between_centers'.
        - Se agrega 'marker_size_mm' en vision_system.
        - Se elimina el campo 'colors' de cada robot en la sección 'robots'.

        Parámetros:
        - filePath (str): La ruta del archivo de configuración JSON.
        """
        with open(filePath, 'r') as file:
            configuration = json.load(file)

        self.simConfig = configuration.get('simulation', {})
        self.scenarioConfig = configuration.get('scenario', {})
        if self.simMode:
            self.logTag = 'SIM_'

        self.configVisionSystem(configuration['vision_system'])
        self.configUdp(configuration['udp_communication'])
        self.generalConfig(configuration['general'])

        self.robotsConfig = configuration['robots']


    def generalConfig(self, configuration):
        """
        Configura las opciones generales del sistema a partir de un diccionario de configuración.

        Parámetros:
        - configuration (dict): Configuración general del sistema.
        """
        self.debug = configuration['debug_enable']

        self.pathVideo = configuration['path_save_videos']
        self.pathPositionLogs = configuration['path_save_position_logs']
        self.pathConsolelog = configuration['path_save_console_logs']

        for path in [self.pathVideo, self.pathPositionLogs, self.pathConsolelog]:
            if not os.path.exists(path):
                print(f"La ruta {path} no existe.")
                try:
                    os.makedirs(path)
                    print(f"Se ha creado la ruta {path}.")
                except Exception as e:
                    raise Exception(f"Error al crear la ruta: {e}")


    def configUdp(self, configuration):
        """
        Configura la comunicación UDP para el sistema.

        Parámetros:
        - configuration (dict): Configuración UDP.
        """
        self.port = configuration['port']

        if self.simMode:
            # La base toma 127.0.0.1:6060 — los controllers de Webots mandan
            # todo ahí. Sin SO_REUSEPORT a propósito: si base_camera.py llega
            # después, su bind falla y entra en modo solo-cámara (feed CAM).
            self.baseIP = '127.0.0.1'
            self.broadcastIP = '127.0.0.1'
            self.networkInterface = None
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2**20)
            try:
                self.sock.bind(('127.0.0.1', self.port))
                print(f'✓ Socket UDP bind exitoso en 127.0.0.1:{self.port} (SIM)')
            except OSError as e:
                print(f'✗ Puerto {self.port} ocupado: {e}')
                print('  En modo sim la base debe iniciarse ANTES que Webots.')
                print('  Cerrá Webots (flatpak kill com.cyberbotics.webots) y reintentá.')
                raise
            return

        self.baseIP = configuration['base_ip']
        self.broadcastIP = configuration['broadcast_ip']
        self.networkInterface = configuration.get('network_interface', None)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        if platform.system() == 'Linux':
            try:
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                print('✓ SO_REUSEPORT habilitado (Linux)')
            except AttributeError:
                print('⚠ SO_REUSEPORT no disponible en esta versión de Python')

            if self.networkInterface:
                try:
                    self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE,
                                         self.networkInterface.encode())
                    print(f'✓ Socket vinculado a interfaz {self.networkInterface}')
                except (AttributeError, OSError) as e:
                    print(f'⚠ No se pudo vincular a interfaz {self.networkInterface}: {e}')

        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2**20)

        # Bind en 0.0.0.0 para evitar [Errno 99] cuando base_ip no está asignada
        # a la interfaz en el momento del bind. SO_BINDTODEVICE (Linux) ya restringe
        # el tráfico a la interfaz correcta (enp3s0), por lo que el bind amplio es seguro.
        bindAddress = '0.0.0.0' if platform.system() == 'Linux' else self.baseIP
        try:
            self.sock.bind((bindAddress, self.port))
            print(f'✓ Socket UDP bind exitoso en {bindAddress}:{self.port}')
            if bindAddress == '0.0.0.0':
                print(f'  (Tráfico restringido a interfaz {self.networkInterface or "todas"})')
        except OSError as e:
            print(f'✗ Error al hacer bind en {bindAddress}:{self.port}')
            print(f'  Motivo: {e}')
            raise


    def configVisionSystem(self, configuration):
        """
        Configura el sistema de visión con soporte ArUco.

        Cambios respecto al sistema de círculos:
        - Inicializa el detector ArUco con DICT_4X4_50.
        - Lee marker_size_mm desde la configuración.
        - Elimina la configuración de circles_radius y distance_between_centers.

        Parámetros:
        - configuration (dict): Configuración del sistema de visión.
            Claves requeridas:
                'path_cameraMatrix', 'path_distance', 'mmPixel',
                'frame_processing_interval', 'marker_size_mm'
        """
        if self.simMode:
            self.configSimVision(configuration)
            return

        self.setCamera(configuration)

        self.cameraMatriz = np.loadtxt(configuration['path_cameraMatrix'], dtype=float)
        self.distance = np.loadtxt(configuration['path_distance'], dtype=float)

        # Ángulo desde las ARISTAS del marker en vez del rvec de solvePnP.
        # IPPE_SQUARE tiene dos soluciones casi empatadas con cámara cenital y
        # marker plano (ambigüedad de flip); el desempate parpadea y eso explica
        # la σ=3.8° medida el 2026-07-27 con el robot QUIETO (el jitter de
        # esquinas solo daría ~0.5°). La dirección de la arista superior es el
        # mismo ángulo, sin pasar por PnP. Apagable por config para A/B en lab.
        self.angleFromCorners = bool(configuration.get('angle_from_corners', True))
        # Mediana-de-3 por marker: mata misreads de UN frame (saltos de ~500mm
        # vistos hoy) antes de que lleguen a robots y logs. Con cambio real
        # sostenido converge sola en 2 frames — sin contadores ni resync.
        self._poseHist = {}   # id → [(t, x, y, ang), ...] máx 3, ventana 0.5s
        h, w = self.cameraResolution

        # La calibración se hizo a 1920x1080. Escalar la matriz si la resolución cambió.
        calib_w, calib_h = 1920, 1080
        sx, sy = w / calib_w, h / calib_h
        if sx != 1.0 or sy != 1.0:
            self.cameraMatriz = self.cameraMatriz.copy()
            self.cameraMatriz[0, 0] *= sx  # fx
            self.cameraMatriz[1, 1] *= sy  # fy
            self.cameraMatriz[0, 2] *= sx  # cx
            self.cameraMatriz[1, 2] *= sy  # cy
            print(f'  Matriz de cámara escalada {calib_w}x{calib_h} → {w}x{h}')

        self.newCameraMatriz, self.roi = cv2.getOptimalNewCameraMatrix(
            self.cameraMatriz, self.distance, (w, h), 1, (w, h)
        )
        # Pre-computar mapas de corrección — remap es 3-5x más rápido que undistort
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.cameraMatriz, self.distance, None, self.newCameraMatriz, (w, h), cv2.CV_16SC2
        )

        numerator, denominator = map(int, configuration['mmPixel'].split('/'))
        self.mmPixel = numerator / denominator
        self.processInterval = configuration['frame_processing_interval']

        # Tamaño físico del marker en mm (recomendado: 80mm+ a 2.5m de distancia)
        self.markerSizeMm = float(configuration['marker_size_mm'])
        self.originXmm = float(configuration.get('origin_x_mm', 0))
        self.originYmm = float(configuration.get('origin_y_mm', 0))
        # ID del marker fijo de referencia que ancla el origen del escenario.
        # Si está presente en el frame, el sistema lo usa como (0,0) automáticamente.
        ref = configuration.get('reference_marker_id', '')
        self.referenceMarkerId = str(ref) if ref != '' else None

        # Radio visual para el resultsFrame (proporcional al tamaño del marker en px)
        self.bigCircleRadius = max(6, int((self.markerSizeMm / self.mmPixel) * 0.5))

        # Inicializar detector ArUco
        # DICT_4X4_50: markers 4x4 bits, 50 IDs disponibles — robusto y compacto
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters()

        # A 2.5m de altura los markers de 80mm miden ~47px a resolución completa.
        # WinSize: ventana adaptativa relativa al tamaño del marker — mín 3, máx ~marker/2
        arucoParams.adaptiveThreshWinSizeMin = 3
        arucoParams.adaptiveThreshWinSizeMax = 23    # ~marker/2; antes era 53 (para media-res)
        arucoParams.adaptiveThreshWinSizeStep = 4
        arucoParams.adaptiveThreshConstant = 7       # default; con full-res hay más señal
        arucoParams.minMarkerPerimeterRate = 0.02    # 47px perím. / 1280px ancho ≈ 0.037 — margen
        arucoParams.maxMarkerPerimeterRate = 4.0
        arucoParams.polygonalApproxAccuracyRate = 0.05
        arucoParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        # 1.0 = usar TODA la capacidad de corrección del diccionario. Con los
        # markers de 90mm los robots miden ~40px (10% menos que con los de
        # 100mm) y a ese tamaño 0.9 los descartaba: medido 2026-07-28, con 0.9
        # solo aparecía el marker fijo del piso; con 1.0 aparecen los tres.
        # Ablación: es el ÚNICO parámetro que los recupera (winSizeMax=53 trae
        # uno solo; pixelPerCell=4 rompe la detección entera).
        arucoParams.errorCorrectionRate = 0.9
        arucoParams.perspectiveRemovePixelPerCell = 8
        # Guarda contra falsos positivos: al relajar la detección aparecieron
        # blobs de ~3px con ID válido. Un fantasma con el ID de un robot sería
        # catastrófico (poses inventadas), así que se descarta por tamaño
        # aparente — los markers reales miden 35-40px a esta altura.
        self.minMarkerSidePx = 15.0

        self.arucoDetector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
        print(f'✓ Detector ArUco inicializado — DICT_4X4_50, marker: {self.markerSizeMm}mm')


    def configSimVision(self, configuration):
        """
        Configuración de visión en modo simulación: sin cámara física ni
        calibración — las poses llegan por UDP desde el supervisor de Webots
        (base_camera.py en modo solo-cámara). El frame se sintetiza en
        SimVision.read() para que video/cobertura/debug sigan funcionando.

        Parámetros:
        - configuration (dict): sección 'vision_system' del JSON (se reusan
          frame_processing_interval y debug_resolution; el resto se ignora).
        """
        sc = self.simConfig
        self.processInterval = configuration['frame_processing_interval']
        self.markerSizeMm = float(configuration['marker_size_mm'])
        self.debugResolution = tuple(map(int, configuration['debug_resolution'].split('x')))
        ref = configuration.get('reference_marker_id', '')
        self.referenceMarkerId = str(ref) if ref != '' else None

        # Escala del frame sintético: mm por píxel sobre el área de la arena.
        # La arena sale de 'scenario' (con presets por cantidad de robots), no de
        # 'simulation': acá se leía sc.get('arena_mm'), que no existe en esa
        # sección, así que caía siempre al default y los presets se ignoraban.
        # En sim el FOV sí cubre la arena entera — el frame se sintetiza de ella.
        self.mmPixel = float(sc.get('mm_per_px', 2.0))
        arenaW, arenaH = self.arenaMm()
        self.cameraResolution = (int(arenaH / self.mmPixel), int(arenaW / self.mmPixel))
        self.bigCircleRadius = max(6, int((self.markerSizeMm / self.mmPixel) * 0.5))

        visionPort = int(sc.get('vision_port', 6055))
        controlAddr = ('127.0.0.1', int(sc.get('control_port', 6059)))
        self.simVision = SimVision(self, visionPort, controlAddr)
        h, w = self.cameraResolution
        print(f'✓ Visión SIM inicializada — frame {w}x{h}px @ {self.mmPixel}mm/px')


    def setCamera(self, configuration):
        """
        Inicializa la captura de video desde la cámara.

        Parámetros:
        - configuration (dict): Configuración de la cámara.
        """
        system = platform.system()
        if system == 'Linux':
            backend = cv2.CAP_V4L2
        elif system == 'Windows':
            backend = cv2.CAP_DSHOW
        elif system == 'Darwin':
            backend = cv2.CAP_AVFOUNDATION
        else:
            backend = cv2.CAP_ANY

        camera_index = configuration.get('camera_index', None)

        if camera_index is None:
            print(f'Detectando cámara en {system}...')
            for i in range(5):
                test_cam = cv2.VideoCapture(i, backend)
                if test_cam.isOpened():
                    camera_index = i
                    test_cam.release()
                    print(f'✓ Cámara encontrada en índice {i}')
                    break

            if camera_index is None:
                print('Error: No se detectó ninguna cámara.')
                exit()

        self.cameraIndex = camera_index
        self.cameraBackend = backend
        self.camera = cv2.VideoCapture(camera_index, backend)
        print(f'Backend de cámara: {backend} (índice {camera_index})')

        self.debugResolution = tuple(map(int, configuration['debug_resolution'].split('x')))
        width, height = map(int, configuration['camera_resolution'].split('x'))
        self.cameraResolution = (height, width)

        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # evita acumulación de frames viejos
        # MJPEG permite 1080p @ 30 FPS por USB; sin esto V4L2 usa YUYV (~5 FPS)
        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
        # Óptica FIJA. Los tres automáticos de la C920 sabotean el ArUco y
        # ninguno sobrevive a desconectar el USB, así que se fijan en cada
        # arranque (medido 2026-07-28 con markers de bajo contraste):
        #  - autofocus: cazaba y desenfocaba → nitidez 88 (borroso). Con foco
        #    fijo al infinito da 100; a partir de focus=30 se derrumba a 48 y
        #    en 40 ya no detecta nada. Es el ajuste que más pesa.
        #  - exposición: en 77 el blanco marcaba 151/255 y el umbral adaptativo
        #    quedaba sin margen; en 250 el blanco llega a 205 sin saturar.
        #    Subirla más es un espejismo: en 600 el 89% de la imagen revienta.
        #  - balance de blancos: si deriva, cambia el punto de corte del umbral.
        cam = configuration.get('camera_controls', {})
        self.camera.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.camera.set(cv2.CAP_PROP_FOCUS, float(cam.get('focus', 0)))
        self.camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # 1 = manual en V4L2
        self.camera.set(cv2.CAP_PROP_EXPOSURE, float(cam.get('exposure', 250)))
        self.camera.set(cv2.CAP_PROP_AUTO_WB, 0)

        # Verificar que pegaron: V4L2 acepta el set() y lo ignora en silencio si
        # el driver no soporta el control, y un foco que no pegó se paga en
        # detecciones perdidas, no en un error.
        af = self.camera.get(cv2.CAP_PROP_AUTOFOCUS)
        if af not in (0, 0.0, -1):
            print(f'⚠ el autofocus NO quedó apagado (={af}) — si ves markers '
                  f'intermitentes, apagalo a mano:\n'
                  f'  v4l2-ctl -d /dev/video{camera_index} '
                  f'-c focus_automatic_continuous=0 -c focus_absolute=0')

        actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
        actual_w   = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h   = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f'✓ Cámara: {actual_w}x{actual_h} @ {actual_fps:.0f} FPS')

        # Drena frames iniciales corruptos (MJPEG tarda ~30 frames en estabilizarse)
        print('  Calentando cámara...', end='', flush=True)
        for _ in range(30):
            self.camera.read()
        print(' listo')

        # RE-APLICAR la exposición: al arrancar el streaming el driver la pisa
        # (medido 2026-07-28: se fijaba en 250 y tras 30 frames quedaba en 38,
        # con el blanco del papel en 106/255). Todas las sesiones anteriores
        # corrieron subexpuestas por esto, y como el contraste bajo también baja
        # la varianza del Laplaciano, parecía además un problema de foco.
        # Re-aplicarla acá deja el blanco en ~200 y la nitidez en ~194.
        self.camera.set(cv2.CAP_PROP_EXPOSURE, float(cam.get('exposure', 250)))
        for _ in range(10):
            self.camera.read()
        ok, chk = self.camera.read()
        if ok:
            gray = cv2.cvtColor(chk, cv2.COLOR_BGR2GRAY)
            white = float(np.percentile(gray, 95))
            print(f'  Exposición: blanco={white:.0f}/255 '
                  f'nitidez={cv2.Laplacian(gray, cv2.CV_64F).var():.0f}')
            if white < 150:
                print('  ⚠ imagen SUBEXPUESTA — el umbral adaptativo de ArUco '
                      'pierde margen; subí camera_controls.exposure en el JSON')

        if not self.camera.isOpened():
            print(f'Error: No se pudo abrir la cámara {camera_index}.')
            for i in range(5):
                test = cv2.VideoCapture(i, backend)
                if test.isOpened():
                    print(f'  - /dev/video{i} (índice {i})')
                    test.release()
            exit()


    # =========================================================================
    # PROCESAMIENTO DE FRAMES
    # =========================================================================

    def _applyArucoEma(self, raw):
        alpha = self.arucoEmaAlpha
        smoothed = {}
        for rid, (x, y, angle) in raw.items():
            if rid not in self._arucoEma:
                self._arucoEma[rid] = (x, y, angle)
            ex, ey, ea = self._arucoEma[rid]
            nx = alpha * x + (1 - alpha) * ex
            ny = alpha * y + (1 - alpha) * ey
            # ángulo: EMA sobre diferencia normalizada para evitar salto 0/360
            diff = ((angle - ea) + 180) % 360 - 180
            na = (ea + alpha * diff) % 360
            self._arucoEma[rid] = (nx, ny, na)
            smoothed[rid] = (round(nx, 1), round(ny, 1), round(na, 1))
        # limpiar EMA de markers que dejaron de verse
        for rid in list(self._arucoEma):
            if rid not in raw:
                del self._arucoEma[rid]
        return smoothed

    def cameraCorrection(self, frame):
        """
        Desdistorsiona y recorta la imagen de la cámara.

        Parámetros:
        - frame (ndarray): La imagen de entrada en formato BGR.

        Returns:
        - frame (ndarray): Imagen corregida en formato BGR.
        - frameGray (ndarray): Imagen en escala de grises (para uso interno).
        """
        x, y, w, h = self.roi
        frame = cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)[y:y+h, x:x+w]
        frameGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        return frame, frameGray


    def processFrame(self, frame):
        """
        Procesa un frame de la cámara: corrige distorsión, detecta markers ArUco
        y actualiza currentArucoDetections.

        Reemplaza processFrameColors. Ya no se procesan colores HSV ni se lanzan
        hilos por color — la detección ArUco opera sobre el frame BGR completo.

        Parámetros:
        - frame (ndarray): Frame BGR crudo de la cámara.

        Retorna:
        - frame (ndarray): Frame corregido por distorsión en BGR.
        - frameGray (ndarray): Frame en escala de grises.
        """
        if self.simMode:
            # Las detecciones vienen del feed CAM (ya en mm/grados del lab)
            frameGray = None
            raw = self.simVision.snapshot()
        else:
            frame, frameGray = self.cameraCorrection(frame)
            raw = self.detectArucoMarkers(frame)

        # Detección ArUco — raw para desplazamiento/setup, suavizado para navegación
        self.currentArucoDetections = raw
        self._smoothedArucoDetections = self._applyArucoEma(raw)

        if self.debug:
            self.cameraDebug(frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.debug = 'off'

        return frame, frameGray


    def cameraDebug(self, frame):
        """
        Muestra una ventana de depuración con los markers ArUco anotados.

        Parámetros:
        - frame (ndarray): Frame BGR ya corregido.
        """
        if self.gui is not None:
            return  # GUI recibe frames via frameSignal; no se necesita ventana separada
        # En sim el frame ya viene anotado por SimVision (no hay markers que detectar)
        debugFrame = frame.copy() if self.simMode else self.drawArucoDebug(frame.copy())
        resized = cv2.resize(debugFrame, self.debugResolution, interpolation=cv2.INTER_AREA)
        try:
            cv2.imshow('Debug ArUco', resized)
        except cv2.error:
            # build de cv2 sin highgui (headless): seguir sin ventana de debug
            print('⚠ cv2 sin soporte de ventanas — debug visual desactivado '
                  '(la vista queda en Webots)')
            self.debug = False


    # =========================================================================
    # LOOP PRINCIPAL
    # =========================================================================

    def cameraProcessing(self):
        """
        Loop principal de procesamiento de cámara.

        Fases:
        1. Detección visual: busca markers ArUco hasta encontrar numRobots robots.
        2. Setup de red: asocia IPs a robots por desplazamiento angular.
        3. Operación: envía poses a los robots en cada frame.

        El flujo es idéntico al sistema de círculos, solo cambia la fuente de
        detección (ArUco en lugar de HSV + contornos).
        """
        robotIP = None
        isValidFrame = True
        robotsIPs = []
        foundRobots = set()
        configuredRobots = set()
        lastProcessedTime = time.time()
        lastStatusTime = time.time()
        executed = False
        setupRetries = 0
        # Frames de gracia para ver el giro de identificación. A ~15-20 FPS los
        # 15 de antes daban ~1s, y un TURN|90 tarda ~2s en ejecutarse y asentarse
        # → la base declaraba timeout y reenviaba el giro antes de que el robot
        # terminara el anterior, desincronizándose.
        MAX_SETUP_RETRIES = 60

        failCount = 0

        while True:
            if self.simMode:
                ret, frame = self.simVision.read()
            else:
                ret, frame = self.camera.read()
            # Sello de tiempo del FRAME, no del final del procesamiento: la
            # detección ArUco tarda distinto en cada frame (según cuántos
            # markers vea), así que timestampear después metía esa varianza
            # dentro del Δt entre filas del log.
            frameTime = time.time()

            if not ret or frame is None:
                failCount += 1
                if failCount % 30 == 1:
                    print(f"[CAM] Fallo de lectura (#{failCount}), reintentando...")
                if failCount >= 30:
                    print("[CAM] Reabriendo cámara...")
                    self.camera.release()
                    self.camera = cv2.VideoCapture(self.cameraIndex, self.cameraBackend)
                    failCount = 0
                continue

            failCount = 0

            if time.time() - lastProcessedTime < self.processInterval or not isValidFrame:
                isValidFrame = True
                if (self.debug == 'off' or not self.threadInputAlive or
                        not self.videoProcess.is_alive()) and executed:
                    self.cleanup()
                    break
                continue

            lastProcessedTime = time.time()
            frame, frameGray = self.processFrame(frame)

            if len(foundRobots) < self.numRobots:
                foundRobots, _ = self.searchRobotsAruco(self.robotsConfig)

                if time.time() - lastStatusTime >= 2.0:
                    lastStatusTime = time.time()
                    visible = list(self.currentArucoDetections.keys())
                    print(f"[Búsqueda] Robots detectados: {sorted(foundRobots)} / "
                          f"necesarios: {self.numRobots} | "
                          f"ArUco visibles: {visible}")

                if len(foundRobots) == self.numRobots:
                    print(f"[Búsqueda] Todos los robots encontrados: {sorted(foundRobots)}")
                    if self.simMode:
                        # Identidad determinista en sim: id → 127.0.0.1:(6060+id).
                        # No hace falta el giro de identificación ni el broadcast.
                        self.processFoundRobots(foundRobots)
                        self.assignSimAddresses(configuredRobots)
                        robotsIPs = []
                    else:
                        robotsIPs = self.searchRobotsUdp()
                        self.processFoundRobots(foundRobots)
                        # Si el JSON declara la IP de cada marker, la identidad ya
                        # está dada y el giro de identificación sobra. Además de
                        # ahorrar tiempo, evita que un robot que gira mal quede
                        # asociado al marker equivocado (o no se asocie nunca).
                        if self.assignConfiguredAddresses(foundRobots, configuredRobots):
                            robotsIPs = []

            elif len(robotsIPs) != 0:
                robotIP, isValidFrame = self.setupRobots(robotIP, robotsIPs, configuredRobots)
                if robotIP is not None:
                    setupRetries += 1
                    if setupRetries >= MAX_SETUP_RETRIES:
                        print("[Setup] Timeout detectando desplazamiento, reintentando giro...")
                        robotIP = None
                        setupRetries = 0
                else:
                    setupRetries = 0

            else:
                if not executed:
                    executed, resultsFrame = self.initializeVideoAndLogging(frame.shape[:2])
                    if not executed:
                        # initializeVideoAndLogging falló — reintentar en el próximo frame
                        continue

                # 3 decimales (ms). Con 1 decimal el redondeo era más grueso que
                # el período de frame (~33-66ms): varias filas caían en el mismo
                # instante y otras saltaban 0.1s, así que cualquier Δt derivado
                # del log (velocidad, tiempo entre eventos) salía escalonado o
                # dividía por cero. Los consumidores (analyze_logs, scan_logs,
                # turn_check) leen con float(), así que aceptan ambos formatos.
                timeLog = round(frameTime - self.startTime, 3)
                processingStart = time.time()
                if time.time() - lastStatusTime >= 2.0:
                    lastStatusTime = time.time()
                self.sendPositions(resultsFrame, timeLog)
                self.addFrame(frame, resultsFrame, timeLog)
                self.addTimeLog(timeLog, round(time.time() - processingStart, 4))


    # =========================================================================
    # ENVÍO DE POSICIONES Y LOGGING
    # =========================================================================

    def sendPositions(self, resultsFrame, timeLog):
        """
        Envía las posiciones de los robots y actualiza el registro de posiciones.

        Parámetros:
        - resultsFrame (ndarray): Frame de resultados donde se dibuja la posición.
        - timeLog (float): Tiempo actual desde el inicio.
        """
        for robot in self.robots.values():
            displacement = robot.getDisplacement()
            if displacement is not None:
                x, y, angle = robot.previousPose
                self._paintCoverage(resultsFrame, robot.id, x, y)

                instruction = f'POSE|{x}|{y}|{angle}'
                self.sendInstruction(robot.IP, [instruction], False)

                self.addPositionLog(timeLog, robot.id, robot.name,
                                    robot.previousPose, displacement, robot)
        self._drawLegend(resultsFrame)

        # NEIGHBOR_POSITIONS a 1 Hz: cada robot conoce dónde están los demás
        # (insumo de dispersión y flocking; los firmware sin soporte lo ignoran)
        now = time.time()
        if now - self._lastNeighborCast >= 1.0:
            self._lastNeighborCast = now
            items = []
            for robot in self.robots.values():
                x, y, _ = robot.previousPose
                if x != -1 and robot.IP:
                    items.append(f'{robot.id},{x},{y}')
            if len(items) >= 2:
                message = 'NEIGHBOR_POSITIONS|' + ';'.join(items)
                for robot in self.robots.values():
                    if robot.IP:
                        self.sock.sendto(message.encode(), self._robotAddr(robot.IP))


    def initializeVideoAndLogging(self, resolution):
        """
        Inicializa el proceso de grabación de video y el registro de posiciones.

        Parámetros:
        - resolution (tuple): (alto, ancho) del frame.

        Returns:
        - tuple: (True, resultsFrame)
        """
        h, w = resolution
        resultsFrame = np.full((h, w, 3), (255, 255, 255), dtype=np.uint8)

        self.cellPx = max(4, int(self.cellSizeMm / self.mmPixel))
        grid_h = (h + self.cellPx - 1) // self.cellPx
        grid_w = (w + self.cellPx - 1) // self.cellPx
        self.coverageGrid = np.full((grid_h, grid_w), -1, dtype=np.int8)

        # Dibujar líneas de grilla tenues para visualizar la cuadrícula vacía
        for gx in range(0, w, self.cellPx):
            cv2.line(resultsFrame, (gx, 0), (gx, h - 1), (220, 220, 220), 1)
        for gy in range(0, h, self.cellPx):
            cv2.line(resultsFrame, (0, gy), (w - 1, gy), (220, 220, 220), 1)

        self.frameQueue = multiprocessing.Queue()
        args = (
            resolution,
            self.numRobots,
            self.pathVideo,
            self.processInterval,
            self.frameQueue,
            self.debugResolution
        )
        self.videoProcess = multiprocessing.Process(target=videoWriter, args=args)
        self.videoProcess.start()
        self.startTime = time.time()
        self.createPositionLog()
        self.createTimeLog()

        for robot in self.robots.values():
            x, y, _ = robot.previousPose
            self._paintCoverage(resultsFrame, robot.id, x, y)
            self.addPositionLog(0, robot.id, robot.name, robot.previousPose, (0, 0))
        self._drawLegend(resultsFrame)

        self.inputInstruction()
        self.readUdpConnection()

        return True, resultsFrame


    def _robotColor(self, robot_id):
        """Retorna el color BGR asignado al robot según su ID."""
        return _ROBOT_COLORS_BGR[int(robot_id) % len(_ROBOT_COLORS_BGR)]

    def _paintCoverage(self, resultsFrame, robot_id, x_mm, y_mm):
        """
        Marca la celda de la grilla de cobertura que corresponde a (x_mm, y_mm)
        y la pinta con el color del robot. Si la celda ya pertenece a este robot,
        no hace nada (evita redibujados innecesarios).
        Coordenadas fuera de la grilla se ignoran silenciosamente.
        """
        if self.coverageGrid is None:
            return
        cell_x = int(x_mm / self.cellSizeMm)
        cell_y = int(y_mm / self.cellSizeMm)
        gh, gw = self.coverageGrid.shape
        if not (0 <= cell_x < gw and 0 <= cell_y < gh):
            return
        robot_idx = int(robot_id)
        if self.coverageGrid[cell_y, cell_x] == robot_idx:
            return
        self.coverageGrid[cell_y, cell_x] = robot_idx
        color = self._robotColor(robot_id)
        px0, py0 = cell_x * self.cellPx, cell_y * self.cellPx
        px1 = min(px0 + self.cellPx, resultsFrame.shape[1])
        py1 = min(py0 + self.cellPx, resultsFrame.shape[0])
        cv2.rectangle(resultsFrame, (px0, py0), (px1 - 1, py1 - 1), color, -1)
        cv2.rectangle(resultsFrame, (px0, py0), (px1 - 1, py1 - 1), (180, 180, 180), 1)

    def _drawLegend(self, resultsFrame):
        """Dibuja la leyenda de colores por robot en la esquina superior derecha del mapa."""
        patch, gap, margin = 18, 4, 8
        x0 = resultsFrame.shape[1] - 130
        y0 = margin
        for robot in self.robots.values():
            color = self._robotColor(robot.id)
            cv2.rectangle(resultsFrame, (x0, y0), (x0 + patch, y0 + patch), color, -1)
            cv2.rectangle(resultsFrame, (x0, y0), (x0 + patch, y0 + patch), (60, 60, 60), 1)
            label = f'R{robot.id}: {robot.name[:7]}'
            cv2.putText(resultsFrame, label, (x0 + patch + 4, y0 + patch - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (30, 30, 30), 1)
            y0 += patch + gap


    def cleanup(self):
        """
        Libera los recursos utilizados por la cámara y cierra las ventanas de OpenCV.
        """
        if self.camera is not None:
            self.camera.release()

        if self.frameQueue is not None:
            self.frameQueue.put(None)
            time.sleep(0.2)
            while not self.frameQueue.empty():
                self.frameQueue.get()
            self.frameQueue.close()

        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass   # build de cv2 sin highgui (headless) — no hay ventanas que cerrar


    def addFrame(self, frame, resultsFrame, timeLog):
        """
        Agrega un fotograma y el frame de resultados a la cola de procesamiento.

        Parámetros:
        - frame (ndarray): Frame actual de la cámara.
        - resultsFrame (ndarray): Frame de resultados.
        - timeLog (float): Tiempo transcurrido en segundos.
        """
        # 1 decimal en el overlay a propósito: el CSV lleva ms, pero en el video
        # un número que cambia en la 3a cifra cada frame no se puede leer.
        cv2.putText(frame, f'Time: {timeLog:.1f} s', (2, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 200, 200), 1, cv2.LINE_AA)
        # Enviar a la GUI si está disponible; siempre encolar para grabación en disco
        if self.gui is not None:
            self.gui.frameSignal.emit(frame.copy(), resultsFrame.copy())
        if self.frameQueue is not None:
            self.frameQueue.put([frame, resultsFrame])


    # =========================================================================
    # LOGGING
    # =========================================================================

    def createTimeLog(self):
        """Crea un archivo CSV de registro de tiempos de procesamiento."""
        currentTime = datetime.now().strftime(r'%d-%m_%H-%M')
        logName = f'Time_Log_{currentTime}_Robots_{self.numRobots}.csv'
        os.makedirs('Logs', exist_ok=True)
        self.pathTimeLogs = os.path.join('Logs', logName)
        header = ['time', 'processingTime']
        with open(self.pathTimeLogs, 'w', newline='') as f:
            csv.writer(f).writerow(header)


    def addTimeLog(self, timeLog, processingTime):
        """Agrega una entrada al registro de tiempo de procesamiento."""
        row = [timeLog, processingTime]
        with open(self.pathTimeLogs, 'a', newline='') as f:
            csv.writer(f).writerow(row)


    def createPositionLog(self):
        """Crea un archivo CSV de registro de posiciones de robots."""
        currentTime = datetime.now().strftime(r'%d-%m_%H-%M')
        logName = f'Position_Log_{self.logTag}{currentTime}_Robots_{self.numRobots}.csv'
        self.pathPositionLogs = os.path.join(self.pathPositionLogs, logName)
        # Las columnas ekf_* van al final para no mover las que ya existen:
        # analyze_logs.py lee con DictReader, así que agregar al final es
        # compatible con los logs viejos (que simplemente no las traen).
        header = ['time', 'idrobot', 'robot', 'x', 'y', 'angle',
                  'linearDisplacement', 'angularDisplacement',
                  'ekf_x', 'ekf_y', 'ekf_angle', 'ekf_age_ms']
        with open(self.pathPositionLogs, 'w', newline='') as f:
            csv.writer(f).writerow(header)


    def addPositionLog(self, timeLog, id, name, position, displacement, robot=None):
        """
        Agrega una entrada al registro de posiciones de los robots.

        Si se pasa `robot`, se anexa su última pose de EKF (telemetría pasiva) y
        la antigüedad de esa muestra en ms. Con la pose de cámara y la del EKF en
        la misma fila, el error del EKF es una resta de columnas. `ekf_age_ms`
        importa para no comparar contra una muestra vieja: el EKF llega a 2Hz y
        el log se escribe por frame, así que valores de ~0-500ms son normales.
        """
        row = [timeLog, id, name, *position, *displacement]
        if robot is not None and robot.ekfPose is not None:
            ageMs = (time.time() - robot.ekfStamp) * 1000.0
            row += [*(f'{v:.1f}' for v in robot.ekfPose), f'{ageMs:.0f}']
        else:
            row += ['', '', '', '']
        with open(self.pathPositionLogs, 'a', newline='') as f:
            csv.writer(f).writerow(row)


    def createConcoleLog(self):
        """Crea un archivo CSV de registro de mensajes UDP recibidos."""
        currentTime = datetime.now().strftime(r'%d-%m_%H-%M')
        logName = f'Console_Log_{self.logTag}{currentTime}_Robots_{self.numRobots}.csv'
        self.pathConsolelog = os.path.join(self.pathConsolelog, logName)
        header = ['time', 'idrobot', 'robot', 'message']
        with open(self.pathConsolelog, 'w', newline='') as f:
            csv.writer(f).writerow(header)


    def addConcoleLog(self, timeLog, id, name, message):
        """Agrega una entrada al registro de la consola UDP."""
        row = [timeLog, id, name, message]
        with open(self.pathConsolelog, 'a', newline='') as f:
            csv.writer(f).writerow(row)


    # =========================================================================
    # COMUNICACIÓN UDP
    # =========================================================================

    def searchRobotsUdp(self):
        """
        Busca robots en la red mediante transmisión UDP.

        Retorna:
        - list: Lista de IPs de robots encontrados.
        """
        instruction = [f'CONFIG|START|{self.broadcastIP}']
        self.sendInstructionBroadcast(instruction)
        robotsIPs = set()

        self.sock.settimeout(2.0)
        while len(robotsIPs) < self.numRobots:
            try:
                data, addr = self.sock.recvfrom(1024)
                if addr[0] != self.baseIP:
                    print(f"Respuesta de {addr[0]}: {data.decode()}")
                    robotsIPs.add(addr[0])
            except socket.timeout:
                try:
                    self.sendInstructionBroadcast(instruction)
                except OSError as e:
                    print(f"⚠ Error al reenviar broadcast: {e}")
                    break

        return list(robotsIPs)


    def _robotAddr(self, ip):
        """
        Traduce la dirección de un robot a tupla (host, puerto).
        Acepta 'ip' (lab: puerto común) o 'ip:puerto' (sim: puerto por robot).
        """
        if ':' in ip:
            host, port = ip.rsplit(':', 1)
            return (host, int(port))
        return (ip, self.port)


    def sendInstructionBroadcast(self, instructions):
        """Envía instrucciones a todos los robots por broadcast."""
        if self.simMode:
            # En localhost no hay broadcast: se emula enviando a cada robot
            for instruction in instructions:
                for robot in self.robots.values():
                    if robot.IP:
                        self.sock.sendto(instruction.encode(), self._robotAddr(robot.IP))
                print(f"(Broadcast sim) Mensaje enviado: {instruction}")
            return
        for instruction in instructions:
            self.sock.sendto(instruction.encode(), (self.broadcastIP, self.port))
            print(f"(Broadcast) Mensaje enviado: {instruction}")


    @runOnThread
    def sendInstruction(self, ip, instructions, printing):
        """
        Envía instrucciones a un robot específico por IP (o 'ip:puerto' en sim).

        Parámetros:
        - ip (str): Dirección IP del robot.
        - instructions (list): Lista de instrucciones a enviar.
        - printing (bool): Si True, imprime confirmación en consola.
        """
        for instruction in instructions:
            self.sock.sendto(instruction.encode(), self._robotAddr(ip))
            name = next((robot.name for robot in self.robots.values() if robot.IP == ip), ip)
            if printing:
                self.log(f'Mensaje enviado a {name}: {instruction}')


    @runOnThread
    def readUdpConnection(self):
        """
        Escucha y procesa mensajes recibidos a través de la conexión UDP.
        """
        self.createConcoleLog()
        self.sock.settimeout(None)

        while True:
            data, addr = self.sock.recvfrom(1024)
            ip = addr[0]
            # En sim todos los robots comparten 127.0.0.1 — la identidad la da
            # el puerto de origen (cada controller tiene el suyo)
            peer = f'{addr[0]}:{addr[1]}' if self.simMode else ip

            if self.simMode or ip != self.baseIP:
                message = data.decode()
                # Misma resolución que el PositionLog: analyze_logs cruza ambos
                # por ventana de tiempo y con 0.1s no se podía ordenar el orden
                # real de dos mensajes del mismo décimo de segundo.
                timeLog = round(time.time() - self.startTime, 3)

                robotFound = False
                for robot in self.robots.values():
                    if robot.IP == peer:
                        name, id = robot.name, robot.id
                        robotFound = True
                        break

                if not robotFound:
                    name, id = peer, "-1"

                self.addConcoleLog(timeLog, id, name, message)

                if self._calib is not None and id == self._calib.get('robotID'):
                    self._calibOnMessage(message)

                parts = message.split('|')
                command = parts[0]

                if command == 'REQUEST_POSITION':
                    if robotFound:
                        self.sendPositionToRobot(peer, id)
                    if len(parts) >= 5 and parts[1] == 'BUG2':
                        self.log(f'Solicitud GT {name}: {parts[2]} paso={parts[3]} dist={parts[4]}mm')
                    else:
                        self.log(f'Solicitud de posición de {name}')

                elif command == 'LEADER_POSITION':
                    if len(parts) >= 5:
                        leaderID = parts[1]
                        leaderX, leaderY, leaderAngle = float(parts[2]), float(parts[3]), float(parts[4])
                        self.updateRobotPosition(leaderID, leaderX, leaderY, leaderAngle)
                        # El líder difunde a ~4Hz (×2) → loguear cada frame inunda
                        # la terminal. Throttle a 1/3s (la pose igual queda en el
                        # PositionLog completo).
                        now = time.time()
                        if now - getattr(self, '_lastLeaderLogTime', 0) >= 3.0:
                            self._lastLeaderLogTime = now
                            self.log(f'Líder {leaderID} @ ({leaderX:.0f},{leaderY:.0f}) '
                                     f'{leaderAngle:.0f}° [log 1/3s]')
                        if self.simMode:
                            # En el lab esto viaja por broadcast WiFi robot→robots;
                            # en localhost la base lo retransmite a los seguidores
                            for robot in self.robots.values():
                                if robot.id != leaderID and robot.IP:
                                    self.sendInstruction(robot.IP, [message], False)

                elif command == 'EKF_POSE':
                    # Telemetría pasiva del EKF del firmware (2Hz). No se
                    # reenvía ni se actúa sobre ella: se guarda para que
                    # addPositionLog la escriba junto a la pose de ArUco del
                    # mismo instante. Así una corrida normal deja los datos para
                    # medir la deriva del EKF sin dejarlo controlar nada.
                    if robotFound and len(parts) >= 4:
                        try:
                            self.robots[id].ekfPose = (float(parts[1]),
                                                       float(parts[2]),
                                                       float(parts[3]))
                            self.robots[id].ekfStamp = time.time()
                        except ValueError:
                            pass

                elif command == 'COLOR_QUERY' and self.simMode:
                    # APDS virtual: el supervisor de Webots conoce los colores
                    # del mundo y responde COLOR_RESPONSE directo al robot
                    if robotFound:
                        self.simVision.sendControl(f'COLOR_QUERY.{id}')

                elif command == 'CHECK_OBSTACLE':
                    continue

                else:
                    self.log(f'Mensaje de {name}: {message}')


    @runOnThread
    def sendPositionToRobot(self, robotIP, robotID):
        """
        Envía la posición actual de un robot específico vía UDP.

        Parámetros:
        - robotIP (str): IP del robot.
        - robotID (str): ID del robot.
        """
        if robotID not in self.robots:
            print(f"Robot {robotID} no encontrado")
            return

        robot = self.robots[robotID]

        # Reintento corto en vez de rendirse en el primer frame malo.
        #
        # getPose() mira SOLO la detección del frame actual, y el ArUco titila
        # según la posición en la arena (reflejo especular del acrílico, zonas de
        # sombra). Antes, un único frame sin detección justo cuando llegaba el
        # pedido hacía que la base no contestara nada, y el robot se comía el
        # timeout completo del firmware (5s) + 500ms de espera antes de reintentar.
        # O sea que un titileo de 30ms costaba 5.5s de inmovilidad: es la causa de
        # los robots que "quedan estáticos" en ciertas zonas (2026-07-29).
        # Acá se espera a la próxima detección buena, que suele llegar en 1-3
        # frames, y se responde con una pose REAL — no interpolada.
        deadline = time.time() + self.poseWaitTimeout
        started = time.time()
        x, y, angle = robot.getPose()
        while x == -1 and time.time() < deadline:
            time.sleep(0.02)
            x, y, angle = robot.getPose()

        waited = time.time() - started
        if x == -1:
            self.log(f'⚠ {robot.name}: sin detección ArUco tras '
                     f'{waited * 1000:.0f}ms — sin responder, el robot va a '
                     f'reintentar')
            return

        message = f'POSITION_RESPONSE|{x}|{y}|{angle}'
        self.sendInstruction(robotIP, [message], False)
        # El titileo recuperado se loguea para poder medirlo después: si esto
        # aparece seguido, el problema de iluminación/reflejo es real y vale
        # atacarlo en el montaje, no solo tolerarlo acá.
        recovered = f' (recuperada tras {waited * 1000:.0f}ms de titileo)' if waited > 0.03 else ''
        self.log(f'Posición enviada a {robot.name}: x={x}, y={y}, '
                 f'angle={angle}{recovered}')


    # =========================================================================
    # CONGREGACIÓN Y NAVEGACIÓN GLOBAL
    # =========================================================================

    def startCongregation(self, leaderID, spacing=300.0):
        """
        Inicia congregación con un líder designado (anillo de estacionamiento).

        Endurecida (2026-07): la Base asigna los slots del anillo (2π·idx/n, el
        mismo fan que calcula el firmware) con la misma lógica wall-safe +
        anti-cruce que startFormation circulo, en vez del orden por ID ciego a
        paredes:
          - valida que TODOS los slots caen dentro del área visible (inset), y
            aborta pidiendo centrar el líder si el anillo no cabe;
          - asigna el slot por bearing del follower alrededor del líder, así el
            robot que ya está a la derecha recibe el slot derecho (mínimo cruce).
        Cambio solo en la Base: el firmware sigue calculando 2π·idx/n para cada
        idx, no requiere reflasheo.
        """
        if leaderID not in self.robots:
            print(f"Error: Robot líder {leaderID} no encontrado")
            return
        lx, ly, _lang = self.robots[leaderID].getPose()
        if lx == -1:
            print(f"Líder {leaderID} no visible por la cámara")
            return

        followers = sorted([rid for rid in self.robots if rid != leaderID])
        n = len(followers)

        # Escalar el anillo con N para que los robots no se solapen: cada slot
        # necesita ~MIN_ARC de arco (huella del robot + margen). Para pocos
        # seguidores (≤7) domina el 300mm por defecto; recién con enjambres
        # grandes (10 robots → r≈358mm) el anillo crece. Genérico lab+sim.
        MIN_ARC = 200.0
        if n > 1:
            spacing = max(spacing, n * MIN_ARC / (2 * math.pi))

        # Slot del anillo idx → posición absoluta (mismo 2π·idx/n del firmware)
        def slotPos(idx):
            ang = 2 * math.pi * idx / max(1, n)
            return lx + spacing * math.cos(ang), ly + spacing * math.sin(ang)

        # Validar que el anillo cabe en el área visible (frame: px × mm/px)
        maxX = self.cameraResolution[1] * self.mmPixel
        maxY = self.cameraResolution[0] * self.mmPixel
        inset = 250.0
        # AVISO, no veto: quien decide el slot es cada robot, que conoce la arena
        # por NAV_CONFIG|ARENA y corrige el ángulo si le queda contra una pared.
        # Además, para n==1 el firmware usa el bearing líder→robot y no este
        # abanico, así que abortar con esta fórmula cancelaba congregaciones
        # perfectamente viables (visto 2026-07-27).
        if not all(inset <= sx <= maxX - inset and inset <= sy <= maxY - inset
                   for sx, sy in (slotPos(i) for i in range(n))):
            print(f'⚠ El anillo nominal (r={spacing:.0f}mm) roza los bordes con el '
                  f'líder en ({lx:.0f},{ly:.0f}); cada robot ajustará su slot. '
                  f'Para menos rodeos, acercá el líder al centro.')

        # Asignación anti-cruce POR POSICIÓN: cada follower al slot LIBRE cuya
        # posición absoluta esté más cerca (mínima distancia de viaje). Se empareja
        # sobre las posiciones de slot (el mismo 2π·idx/n que ejecuta el firmware),
        # no sobre bearings: comparar ángulos cruzaba si la convención de marco de
        # la cámara difería del atan2 del firmware (visto 2026-07-23).
        slotXY = {idx: slotPos(idx) for idx in range(n)}
        pairs = sorted(
            (math.dist(self.robots[rid].getPose()[:2], slotXY[idx]), rid, idx)
            for rid in followers for idx in range(n))
        assign, takenSlots = {}, set()
        for _d, rid, idx in pairs:
            if rid not in assign and idx not in takenSlots:
                assign[rid] = idx
                takenSlots.add(idx)

        self.congregationActive = True
        self.leaderID = leaderID
        self.sendInstruction(self.robots[leaderID].IP,
                             [f'CONGREGATION|{leaderID}|0|{n}'], False)
        for rid in followers:
            idx = assign[rid]
            self.sendInstruction(self.robots[rid].IP,
                                 [f'NAV_CONFIG|PARKING_DIST|{spacing:.0f}',
                                  f'CONGREGATION|{leaderID}|{idx}|{n}'], False)
            print(f"  {self.robots[rid].name}: slot {idx}/{n} (anti-cruce por posición)")

        print(f"Congregación iniciada. Líder: {self.robots[leaderID].name}, "
              f"{n} seguidor(es)")


    def startFormation(self, args):
        """
        Inicia una formación: FORMATION.<figura> <líderID>
        Figuras: linea (fila perpendicular al heading del líder), cuna (V detrás
        del líder), circulo (distribución angular, como la congregación).

        La base asigna los índices de slot conociendo dónde está cada follower
        (mínimo cruce de trayectorias): para linea/cuna se ordenan por su
        coordenada lateral respecto al heading del líder, para circulo por su
        bearing alrededor del líder — el follower que ya está a la derecha
        recibe el slot derecho.
        """
        parts = args.split()
        if len(parts) not in (2, 3) or parts[0] not in ('linea', 'cuna', 'circulo'):
            print('Formato: FORMATION.linea|cuna|circulo líderID [espaciado_mm]')
            return
        shape, leaderID = parts[0], parts[1]
        spacing = float(parts[2]) if len(parts) == 3 else 300.0
        if leaderID not in self.robots:
            print(f'Error: Robot líder {leaderID} no encontrado')
            return
        lx, ly, lang = self.robots[leaderID].getPose()
        if lx == -1:
            print(f'Líder {leaderID} no visible por la cámara')
            return

        followers = sorted([rid for rid in self.robots if rid != leaderID])
        n = len(followers)
        rad = math.radians(lang)

        def slotOffset(shape, idx, axisDeg):
            """Réplica de formation_slot del robot — para validar límites."""
            if shape == 'circulo':
                ang = 2 * math.pi * idx / max(1, n)
                return spacing * math.cos(ang), spacing * math.sin(ang)
            pa = rad + math.pi / 2 + math.radians(axisDeg)
            k = idx // 2 + 1
            side = 1 if idx % 2 == 0 else -1
            ox, oy = side * k * spacing * math.cos(pa), side * k * spacing * math.sin(pa)
            if shape == 'cuna':
                ox -= k * spacing * math.cos(rad)
                oy -= k * spacing * math.sin(rad)
            return ox, oy

        # Validar que TODOS los slots caigan dentro del área visible (con
        # margen para staging+robot). El frame define la arena: px × mm/px.
        maxX = self.cameraResolution[1] * self.mmPixel
        maxY = self.cameraResolution[0] * self.mmPixel
        inset = 250.0

        def fits(axisDeg):
            for idx in range(n):
                ox, oy = slotOffset(shape, idx, axisDeg)
                if not (inset <= lx + ox <= maxX - inset and
                        inset <= ly + oy <= maxY - inset):
                    return False
            return True

        axis = 0.0
        if not fits(0.0):
            if shape == 'linea' and fits(90.0):
                axis = 90.0
                print('⚠ La fila perpendicular no cabe — usando el eje del '
                      'heading del líder (columna)')
            else:
                print(f'✗ La formación {shape} no cabe donde está el líder '
                      f'({lx:.0f},{ly:.0f}) — movelo lejos de los bordes')
                return

        pa = rad + math.pi / 2 + math.radians(axis)
        px, py = math.cos(pa), math.sin(pa)   # eje efectivo de la fila

        def followerKey(rid):
            fx, fy, _ = self.robots[rid].getPose()
            if fx == -1:
                return 0.0
            if shape == 'circulo':
                return math.atan2(fy - ly, fx - lx) % (2 * math.pi)
            return (fx - lx) * px + (fy - ly) * py

        def slotKey(idx):
            if shape == 'circulo':
                return 2 * math.pi * idx / max(1, n)
            return (1 if idx % 2 == 0 else -1) * (idx // 2 + 1)

        rankedFollowers = sorted(followers, key=followerKey)
        rankedSlots = sorted(range(n), key=slotKey)

        self.congregationActive = True
        self.leaderID = leaderID
        self.sendInstruction(self.robots[leaderID].IP,
                             [f'FORMATION|{shape}|{leaderID}|0|{n}|{axis:.0f}'], False)
        for rank, rid in enumerate(rankedFollowers):
            idx = rankedSlots[rank]
            self.sendInstruction(self.robots[rid].IP,
                                 [f'NAV_CONFIG|PARKING_DIST|{spacing:.0f}',
                                  f'FORMATION|{shape}|{leaderID}|{idx}|{n}|{axis:.0f}'],
                                 False)
            print(f'  {self.robots[rid].name}: slot {idx} ({shape}, {spacing:.0f}mm)')
        print(f'Formación {shape} iniciada. Líder: {self.robots[leaderID].name}, '
              f'{n} seguidor(es)')


    def sendToGlobalPosition(self, robotID, targetX, targetY):
        """
        Envía un robot a una posición global específica.

        Parámetros:
        - robotID (str): ID del robot.
        - targetX (float): Coordenada X objetivo en mm.
        - targetY (float): Coordenada Y objetivo en mm.
        """
        if robotID not in self.robots:
            print(f"Error: Robot {robotID} no encontrado")
            return

        robot = self.robots[robotID]
        instruction = f'POSITIONGT|{targetX}|{targetY}'
        self.sendInstruction(robot.IP, [instruction], True)
        print(f"Robot {robot.name} enviado a posición: x={targetX}, y={targetY}")


    def updateRobotPosition(self, robotID, x, y, angle):
        """
        Actualiza la posición de un robot en el registro interno.

        Parámetros:
        - robotID (str): ID del robot.
        - x, y (float): Coordenadas en mm.
        - angle (float): Ángulo en grados.
        """
        self.robotPositions[robotID] = {
            'x': x, 'y': y, 'angle': angle,
            'timestamp': time.time()
        }


    def getDistanceBetweenRobots(self, robotID1, robotID2):
        """
        Calcula la distancia euclidiana entre dos robots.

        Retorna:
        - float: Distancia en mm, o -1 si algún robot no existe o no tiene pose.
        """
        if robotID1 not in self.robots or robotID2 not in self.robots:
            return -1

        x1, y1, _ = self.robots[robotID1].previousPose
        x2, y2, _ = self.robots[robotID2].previousPose

        if x1 == -1 or x2 == -1:
            return -1

        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


    def isCongregationComplete(self, threshold=100):
        """
        Verifica si todos los robots están cerca del líder.

        Parámetros:
        - threshold (float): Distancia máxima en mm para considerar "cerca".

        Retorna:
        - bool: True si todos están dentro del umbral respecto al líder.
        """
        if not self.congregationActive or self.leaderID is None:
            return False

        for robotID in self.robots:
            if robotID == self.leaderID:
                continue
            distance = self.getDistanceBetweenRobots(self.leaderID, robotID)
            if distance == -1 or distance > threshold:
                return False

        return True


    # =========================================================================
    # ENTRADA DE USUARIO
    # =========================================================================

    # =========================================================================
    # CALIBRACIÓN POR ROBOT (gyro yawScale + encoder PPR, cámara como patrón)
    # =========================================================================

    def _calibOnMessage(self, message):
        """Hook desde readUdpConnection: captura eventos del robot en calibración."""
        c = self._calib
        if c is None:
            return
        m = re.search(r'TURN IMU: objetivo=(-?[\d.]+)° real=(-?[\d.]+)°', message)
        if m:
            c['imuReals'].append(float(m.group(2)))
            c['lastEvent'] = time.time()
        elif 'Giro completado' in message or 'Movimiento completado' in message:
            c['completions'] += 1
            c['lastEvent'] = time.time()
        elif 'interrumpido' in message or 'failsafe' in message:
            c['aborted'] = True


    def _calibSampleWindow(self, robotID, duration):
        """
        Muestrea el ángulo ArUco (unwrap continuo desde 0) y la posición durante
        `duration` segundos. Retorna (samples, accum, prevAngle) para encadenar
        ventanas consecutivas sin perder la continuidad del unwrap.
        """
        samples = []          # (t, angleAcumulado, x, y)
        t0 = time.time()
        accum, prevAngle = 0.0, None
        while time.time() - t0 < duration:
            pose = self.currentArucoDetections.get(robotID)
            if pose is not None:
                x, y, a = pose
                if prevAngle is not None:
                    d = a - prevAngle
                    if d > 180.0:
                        d -= 360.0
                    elif d < -180.0:
                        d += 360.0
                    accum += d
                prevAngle = a
                samples.append((time.time(), accum, x, y))
            time.sleep(0.03)
        return samples, accum, prevAngle


    def _calibManeuver(self, robotID, robotIP, instruction, timeout=60):
        """
        Mide una maniobra contra la cámara: captura baseline con el robot quieto,
        envía el comando, trackea hasta que queda quieto de nuevo.

        Retorna (deltaAngle, x0, y0, x1, y1) o None si falla (marker perdido,
        timeout o maniobra interrumpida). Los extremos son promedios de ventanas
        de reposo (≈0.8s), inmunes al jitter de ArUco.
        """
        c = self._calib
        c['imuReals'] = []
        c['completions'] = 0
        c['aborted'] = False
        c['lastEvent'] = time.time()

        def windowMean(window):
            n = len(window)
            return (sum(s[1] for s in window) / n,
                    sum(s[2] for s in window) / n,
                    sum(s[3] for s in window) / n)

        # 1) Baseline ANTES de enviar el comando (robot quieto)
        base, accum, prevAngle = self._calibSampleWindow(robotID, 0.8)
        if not base:
            print('[Calib] Robot no visible al capturar baseline — abortando')
            return None
        a0, x0, y0 = windowMean(base)

        # 2) Enviar comando y trackear el movimiento, continuando el unwrap
        self.sendInstruction(robotIP, [instruction], False)
        samples = []
        t0 = time.time()
        lastSeen = time.time()
        while True:
            now = time.time()
            pose = self.currentArucoDetections.get(robotID)
            if pose is not None:
                x, y, a = pose
                d = a - prevAngle
                if d > 180.0:
                    d -= 360.0
                elif d < -180.0:
                    d += 360.0
                accum += d
                prevAngle = a
                samples.append((now, accum, x, y))
                lastSeen = now

            if c['aborted']:
                print('[Calib] Maniobra interrumpida (obstáculo/failsafe) — abortando')
                return None
            if now - lastSeen > 3.0:
                print('[Calib] Marker perdido >3s — abortando')
                return None
            if now - t0 > timeout:
                print('[Calib] Timeout de maniobra — abortando')
                return None

            # Fin: la maniobra (y su corrección) reportó completado, sin eventos
            # nuevos hace 2s, y llevamos >1.5s desde el envío
            if (c['completions'] > 0 and c['completions'] >= len(c['imuReals'])
                    and now - c['lastEvent'] > 2.0 and now - t0 > 1.5):
                break
            time.sleep(0.03)

        last = [s for s in samples if samples[-1][0] - s[0] <= 0.8]
        a1, x1, y1 = windowMean(last)
        return (a1 - a0, x0, y0, x1, y1)


    @runOnThread
    def startCalibration(self, robotID):
        """
        Rutina de calibración completa de un robot usando la cámara como patrón:

        Fase 1 (gyro): YAW_SCALE=1.0 temporal, 3×TURN|360, escala = giro físico
                       (ArUco) / giro reportado (IMU) → NAV_CONFIG|YAW_SCALE|x|SAVE
        Fase 2 (encoders): PPR nominal temporal, 3×MOVE|500 (con TURN|180 entre
                       avances), PPR = nominal × comandado/medido → SETPPR|x|SAVE

        Requiere: robot visible, área despejada (una evasión aborta la rutina).
        """
        if self._calib is not None:
            print('[Calib] Ya hay una calibración activa')
            return
        if robotID not in self.robots:
            print(f"[Calib] Robot '{robotID}' no encontrado")
            return
        if robotID not in self.currentArucoDetections:
            print(f'[Calib] Robot {robotID} no visible por la cámara')
            return

        robotIP = self.robots[robotID].IP
        nominalPPR = 574.0
        # Rango aceptado del PPR calculado: el MISMO que valida el firmware en
        # SETPPR (100-5000). Antes era [400,800] y rechazaba robots legítimos —
        # hay unidades cuya relación de engranes calibra por encima de 800.
        minPPR, maxPPR = 100.0, 5000.0
        # Avance mínimo para dar la maniobra por buena. Solo sirve para detectar
        # "no se movió" / marker perdido, y NO debe acotar el PPR: durante la
        # fase 2 el robot corre con el PPR nominal, así que uno cuyo PPR real sea
        # alto avanza poco a propósito (mide 500·nominal/real ≈ 250mm si el real
        # es ~1150). Con el mínimo viejo de 250mm esos robots se rechazaban por
        # la razón equivocada, culpando al avance en vez del rango.
        minCalibDistance = 100.0
        self._calib = {'robotID': robotID, 'imuReals': [], 'completions': 0,
                       'aborted': False, 'lastEvent': time.time()}
        try:
            print(f'[Calib] === Calibrando robot {robotID} ({self.robots[robotID].name}) ===')
            self.sendInstruction(robotIP, ['CONFIG|DEBUG|1'], False)
            time.sleep(0.3)
            # Desactivar los 3 IR: durante la calibración no queremos NINGUNA
            # evasión (el IR izquierdo fantasma o un reflejo abortarían la rutina)
            for s in ('L', 'R', 'C'):
                self.sendInstruction(robotIP, [f'SENSOR_MASK|{s}|1'], False)
            print('[Calib] Sensores IR desactivados durante la rutina')
            time.sleep(0.5)

            # --- Fase 1: escala del gyro ---
            self.sendInstruction(robotIP, ['NAV_CONFIG|YAW_SCALE|1.0'], False)
            time.sleep(0.5)
            camTotal, imuTotal = 0.0, 0.0
            for n in range(3):
                result = self._calibManeuver(robotID, robotIP, 'TURN|360')
                if result is None:
                    return
                deltaCam = result[0]
                deltaImu = sum(self._calib['imuReals'])
                if abs(deltaImu) < 300:
                    print(f'[Calib] IMU reportó {deltaImu:.1f}° (esperado ~360) — abortando')
                    return
                camTotal += deltaCam
                imuTotal += deltaImu
                print(f'[Calib] Giro {n+1}/3: cámara={deltaCam:+.1f}° IMU={deltaImu:+.1f}°')

            yawScale = camTotal / imuTotal
            if not 0.9 <= yawScale <= 1.1:
                print(f'[Calib] yaw_scale={yawScale:.4f} fuera de rango [0.9,1.1] — abortando')
                return
            self.sendInstruction(robotIP, [f'NAV_CONFIG|YAW_SCALE|{yawScale:.4f}|SAVE'], False)
            print(f'[Calib] ✓ yaw_scale={yawScale:.4f} guardado en flash')
            time.sleep(0.5)

            # --- Fase 2: PPR de encoders ---
            self.sendInstruction(robotIP, [f'SETPPR|{nominalPPR:.0f}|TEMP'], False)
            time.sleep(0.5)
            distances = []
            for n in range(3):
                result = self._calibManeuver(robotID, robotIP, 'MOVE|500')
                if result is None:
                    return
                _, x0, y0, x1, y1 = result
                dist = math.hypot(x1 - x0, y1 - y0)
                if dist < minCalibDistance:
                    print(f'[Calib] Avance midió {dist:.0f}mm (mínimo {minCalibDistance:.0f}) '
                          f'— el robot no se movió o se perdió el marker; abortando')
                    return
                distances.append(dist)
                print(f'[Calib] Avance {n+1}/3: cámara={dist:.1f}mm')
                # Volver sobre sus pasos para no salir del área (yaw ya calibrado)
                result = self._calibManeuver(robotID, robotIP, 'TURN|180')
                if result is None:
                    return

            measured = sum(distances) / len(distances)
            newPPR = nominalPPR * 500.0 / measured
            if not minPPR <= newPPR <= maxPPR:
                print(f'[Calib] PPR={newPPR:.1f} fuera de rango '
                      f'[{minPPR:.0f},{maxPPR:.0f}] — abortando')
                return
            self.sendInstruction(robotIP, [f'SETPPR|{newPPR:.1f}|SAVE'], False)
            print(f'[Calib] ✓ PPR={newPPR:.1f} guardado en flash (medido {measured:.1f}mm/500mm)')
            print(f'[Calib] === Robot {robotID} calibrado: yaw_scale={yawScale:.4f}, PPR={newPPR:.1f} ===')
        finally:
            # Reactivar IR (default firmware). El IR izquierdo fantasma sigue
            # presente: re-enviar SENSOR_MASK|L|1 si se va a navegar tras calibrar.
            for s in ('L', 'R', 'C'):
                self.sendInstruction(robotIP, [f'SENSOR_MASK|{s}|0'], False)
            print('[Calib] Sensores IR reactivados (re-enviar SENSOR_MASK|L|1 si hace falta)')
            self._calib = None


    @runOnThread
    def inputInstruction(self):
        """
        Maneja la entrada de instrucciones desde la consola en tiempo real.

        Formato: 'robotId.instrucción' o comandos especiales:
            BROADCAST.instrucción
            BROADCAST.MEET|x|y[|radio]  (congregación sobre un punto, sin líder;
                                         radio 150-600mm, por defecto lo calcula
                                         el firmware según cuántos robots hay)
            CONGREGATION.leaderID
            GOTO.robotID x y
            STATUS.(cualquier cosa)
            BREAK
        """
        while True:
            try:
                instructionRaw = input('').strip()
            except EOFError:
                break   # stdin cerrado (proceso lanzado sin consola) = BREAK
            if instructionRaw == 'BREAK':
                break

            try:
                robotId, instruction = map(str.strip, instructionRaw.split('.', 1))
            except ValueError:
                print("Formato inválido. Use 'robotId.instrucción'")
                continue

            self.warnIfOutsideFov(instruction)

            if robotId == 'BROADCAST':
                self.sendInstructionBroadcast([instruction])
            elif robotId in self.robots:
                robotIP = self.robots[robotId].IP
                self.sendInstruction(robotIP, [instruction], True)
            elif robotId == 'CONGREGATION':
                self.startCongregation(instruction)
            elif robotId == 'FORMATION':
                self.startFormation(instruction)
            elif robotId == 'CALIBRATE':
                self.startCalibration(instruction)
            elif robotId == 'OCCLUDE':
                # Solo sim: tapa la cámara virtual n segundos (OCCLUDE.10)
                if self.simMode:
                    self.simVision.sendControl(f'OCCLUDE.{instruction}')
                    print(f'Cámara sim ocluida por {instruction}s')
                else:
                    print('OCCLUDE solo existe en modo --sim')
            elif robotId == 'GOTO':
                parts = instruction.split()
                if len(parts) == 3:
                    targetRobotID = parts[0]
                    targetX = float(parts[1])
                    targetY = float(parts[2])
                    self.sendToGlobalPosition(targetRobotID, targetX, targetY)
                else:
                    print("Formato: GOTO.robotID x y")
            elif robotId == 'STATUS':
                print(f"Detecciones ArUco activas: {list(self.currentArucoDetections.keys())}")
                for rid, robot in self.robots.items():
                    x, y, angle = robot.getPose()
                    if x != -1:
                        print(f"  Robot {rid} ({robot.name}): x={x}, y={y}, angle={angle}°")
                    else:
                        print(f"  Robot {rid} ({robot.name}): no visible")
                if self.congregationActive:
                    print(f"Congregación activa. Líder: {self.leaderID}")
                    print(f"Completa: {self.isCongregationComplete()}")
            else:
                print(f"Robot ID '{robotId}' no encontrado.")

        self.threadInputAlive = False


# =============================================================================
# PUNTO DE ENTRADA
# =============================================================================

base = Base()


def main():
    """
    Uso:
        python AttaBot_Base.py               # modo lab (cámara C920 + WiFi)
        python AttaBot_Base.py --sim         # visión y robots desde Webots
        python AttaBot_Base.py --sim --headless   # sin ventana de debug
        python AttaBot_Base.py --sim --robots 2   # sin prompt interactivo

    En modo sim: iniciar la base ANTES que Webots (la base toma el puerto 6060
    y base_camera.py, al encontrarlo ocupado, entra en modo solo-cámara).
    """
    configurationFilePath = 'configSystem.json'
    base.simMode = '--sim' in sys.argv
    if base.simMode:
        print('=== MODO SIMULACIÓN: visión y robots desde Webots ===')
    if '--robots' in sys.argv:
        base.numRobots = int(sys.argv[sys.argv.index('--robots') + 1])
        print(f'Cantidad de robots en la prueba: {base.numRobots}')
    else:
        base.numRobots = int(input('Cantidad de robots en la prueba: '))
    base.readConfigFile(configurationFilePath)
    if '--headless' in sys.argv:
        base.debug = False
    base.cameraProcessing()


if __name__ == '__main__':
    main()