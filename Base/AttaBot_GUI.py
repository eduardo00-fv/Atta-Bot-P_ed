"""Interfaz de control del enjambre AttaBot.

Muestra la cámara con las anotaciones de ArUco, el mapa de cobertura, una tabla
con el estado de cada robot y un panel de comandos agrupados por para qué sirven.

El despacho de comandos NO vive acá: se delega en Base.dispatch(), el mismo que
usa la consola. Tener dos copias fue el motivo de que la GUI se quedara sin
FORMATION, CALIBRATE ni OCCLUDE durante meses.
"""
import os
import sys
import time

# cv2 sobreescribe QT_QPA_PLATFORM_PLUGIN_PATH al importar, apuntando a sus
# propios plugins Qt (incompatibles con PyQt5).  Solución: importar cv2 primero,
# luego corregir la ruta y el backend antes de importar PyQt5.
import cv2

for _sp in sys.path:
    _candidate = os.path.join(_sp, 'PyQt5', 'Qt5', 'plugins')
    if os.path.isdir(_candidate):
        os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = _candidate
        break

if os.environ.get('WAYLAND_DISPLAY'):
    os.environ['QT_QPA_PLATFORM'] = 'wayland'
else:
    os.environ['QT_QPA_PLATFORM'] = 'xcb'
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QLineEdit, QPushButton, QTextEdit, QComboBox, QFrame, QTabWidget,
    QInputDialog, QSizePolicy, QTableWidget, QTableWidgetItem, QHeaderView,
    QCheckBox, QSplitter, QAbstractItemView,
)
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QSize, QTimer
from PyQt5.QtGui import QPixmap, QImage, QFont, QColor


# ── Paleta ───────────────────────────────────────────────────────────────────
# Oscura porque la interfaz convive con el video de una cámara cenital y un
# fondo claro alrededor del frame cansa la vista en sesiones largas de lab.
BG      = '#1b1d21'
PANEL   = '#24262b'
BORDE   = '#34373d'
TEXTO   = '#e6e6e6'
TENUE   = '#9aa0a8'
ACENTO  = '#4a9eff'
OK      = '#3ecf8e'
ALERTA  = '#f5a623'
ERROR   = '#ff5f57'

HOJA = f"""
QMainWindow, QWidget {{ background: {BG}; color: {TEXTO}; }}
QFrame#panel {{ background: {PANEL}; border: 1px solid {BORDE};
                border-radius: 6px; }}
QLabel {{ color: {TEXTO}; }}
QLabel#tenue {{ color: {TENUE}; font-size: 11px; }}
QLineEdit {{ background: {BG}; color: {TEXTO}; border: 1px solid {BORDE};
             border-radius: 4px; padding: 5px 8px;
             selection-background-color: {ACENTO}; }}
QLineEdit:focus {{ border-color: {ACENTO}; }}
QComboBox {{ background: {BG}; color: {TEXTO}; border: 1px solid {BORDE};
             border-radius: 4px; padding: 4px 8px; }}
QComboBox QAbstractItemView {{ background: {PANEL}; color: {TEXTO};
                               selection-background-color: {ACENTO}; }}
QPushButton {{ background: {PANEL}; color: {TEXTO}; border: 1px solid {BORDE};
               border-radius: 4px; padding: 5px 10px; }}
QPushButton:hover {{ border-color: {ACENTO}; color: {ACENTO}; }}
QPushButton:pressed {{ background: {BORDE}; }}
QPushButton#primario {{ background: {ACENTO}; color: #0d1117;
                        border: none; font-weight: 600; }}
QPushButton#primario:hover {{ background: #6fb4ff; color: #0d1117; }}
QPushButton#peligro:hover {{ border-color: {ERROR}; color: {ERROR}; }}
QTextEdit {{ background: #16181c; color: {TEXTO}; border: 1px solid {BORDE};
             border-radius: 6px; }}
QTabWidget::pane {{ border: 1px solid {BORDE}; border-radius: 6px;
                    background: {PANEL}; }}
QTabBar::tab {{ background: transparent; color: {TENUE};
                padding: 6px 14px; border: none; }}
QTabBar::tab:selected {{ color: {ACENTO};
                         border-bottom: 2px solid {ACENTO}; }}
QTableWidget {{ background: #16181c; color: {TEXTO}; gridline-color: {BORDE};
                border: 1px solid {BORDE}; border-radius: 6px; }}
QHeaderView::section {{ background: {PANEL}; color: {TENUE}; border: none;
                        border-bottom: 1px solid {BORDE}; padding: 5px; }}
QCheckBox {{ color: {TENUE}; }}
QSplitter::handle {{ background: {BORDE}; }}
"""


# ── Lo que la GUI necesita saber del firmware ────────────────────────────────
# Se lee del código del controlador en vez de copiarse a mano. Copiarlo ya salió
# mal dos veces: la lista de estados quedó en otro orden que el enum y el panel
# mostraba TURN donde el robot decía MOVE. Si el firmware cambia, esto lo sigue
# solo; si no se puede leer, es preferible quedarse sin nombres que inventarlos.
FIRMWARE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'Controller', 'AttaBot')


def _leerEnumEstados():
    """Nombres de RobotState en el orden del enum, o [] si no se puede leer.

    Devolver [] no es un fallo silencioso: el panel entonces muestra el número
    crudo que mandó el robot, que es información correcta aunque menos legible.
    """
    import re
    try:
        with open(os.path.join(FIRMWARE, 'utils.h'), encoding='utf-8') as f:
            cuerpo = re.search(r'enum\s+RobotState\s*\{(.*?)\}', f.read(),
                               re.S).group(1)
    except (OSError, AttributeError):
        return []
    cuerpo = re.sub(r'//.*|/\*.*?\*/', '', cuerpo, flags=re.S)
    return [t.split('=')[0].strip() for t in cuerpo.split(',') if t.strip()]


def _leerClavesNavConfig():
    """Claves que acepta NAV_CONFIG, leídas del handler de comandos.ino."""
    import re
    try:
        with open(os.path.join(FIRMWARE, 'comandos.ino'), encoding='utf-8') as f:
            cuerpo = re.search(r'void\s+HandleNavConfig\s*\(.*?\n\}',
                               f.read(), re.S).group(0)
    except (OSError, AttributeError):
        return []
    vistas = []
    for c in re.findall(r'arguments\[1\]\s*==\s*"([A-Z_]+)"', cuerpo):
        if c not in vistas:
            vistas.append(c)
    return vistas


class AttaBotGUI(QMainWindow):
    """Ventana principal de control.

    Los frames y los mensajes llegan desde hilos de fondo, así que entran por
    signals de Qt: tocar widgets desde otro hilo cuelga la aplicación.
    """

    frameSignal = pyqtSignal(object, object)   # (frame cámara BGR, mapa BGR)
    logSignal = pyqtSignal(str)

    # Columnas del panel de estado, con la clave del GET_STATUS que las llena.
    COLUMNAS = [
        ('Robot', None), ('Visto', None), ('Pose', 'Pos'), ('Estado', 'State'),
        ('Nav', 'NAV'), ('IR L-C-R', 'Sensors'), ('Máscaras', 'Mask'),
        ('Yaw', 'Yaw'), ('IMU', 'IMU'), ('Deriva EKF', None),
    ]

    ESTADOS = _leerEnumEstados()

    def __init__(self, base):
        super().__init__()
        self.base = base
        self._cmdHistory = []
        self._historyIdx = -1
        self._buildUI()
        self.frameSignal.connect(self._onFrame)
        self.logSignal.connect(self._onLog)
        self.setWindowTitle('AttaBot — control de enjambre')
        self.setStyleSheet(HOJA)
        self.resize(1280, 860)

        # El panel de estado se refresca solo. Pide GET_STATUS a los robots cada
        # tantos segundos y redibuja con lo último que llegó; sin esto habría que
        # apretar un botón para saber si un robot sigue vivo.
        self._auto = QTimer(self)
        self._auto.timeout.connect(self._refreshStatus)
        self._auto.start(2000)

    # ── construcción de la interfaz ──────────────────────────────────────────

    def _buildUI(self):
        root = QWidget()
        self.setCentralWidget(root)
        vbox = QVBoxLayout(root)
        vbox.setSpacing(8)
        vbox.setContentsMargins(10, 10, 10, 10)

        split = QSplitter(Qt.Vertical)
        split.setChildrenCollapsible(False)

        arriba = QWidget()
        camRow = QHBoxLayout(arriba)
        camRow.setSpacing(8)
        camRow.setContentsMargins(0, 0, 0, 0)
        self._camLabel = self._makeFrameLabel('Cámara')
        self._mapLabel = self._makeFrameLabel('Mapa de cobertura')
        camRow.addWidget(self._camLabel, stretch=1)
        camRow.addWidget(self._mapLabel, stretch=1)
        split.addWidget(arriba)

        abajo = QWidget()
        col = QVBoxLayout(abajo)
        col.setSpacing(8)
        col.setContentsMargins(0, 0, 0, 0)
        col.addWidget(self._buildStatusTable())
        col.addWidget(self._buildCommandPanel())
        col.addWidget(self._buildLog())
        split.addWidget(abajo)

        # El video pesa menos que los controles: la cámara se mira de reojo y el
        # trabajo real pasa en la tabla y los comandos. El splitter deja que el
        # usuario le devuelva el alto cuando quiere ver el frame en detalle.
        split.setStretchFactor(0, 2)
        split.setStretchFactor(1, 3)
        vbox.addWidget(split)

    def _buildStatusTable(self):
        marco = QFrame()
        marco.setObjectName('panel')
        caja = QVBoxLayout(marco)
        caja.setContentsMargins(10, 8, 10, 10)
        caja.setSpacing(6)

        fila = QHBoxLayout()
        titulo = QLabel('Estado del enjambre')
        titulo.setFont(QFont('', 10, QFont.Bold))
        fila.addWidget(titulo)
        fila.addStretch()
        self._autoChk = QCheckBox('refrescar cada 2s')
        self._autoChk.setChecked(True)
        fila.addWidget(self._autoChk)
        ahora = QPushButton('Actualizar')
        ahora.clicked.connect(lambda: self._refreshStatus(force=True))
        fila.addWidget(ahora)
        caja.addLayout(fila)

        self._tabla = QTableWidget(0, len(self.COLUMNAS))
        self._tabla.setHorizontalHeaderLabels([c[0] for c in self.COLUMNAS])
        self._tabla.verticalHeader().setVisible(False)
        self._tabla.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._tabla.setSelectionMode(QAbstractItemView.NoSelection)
        self._tabla.setFixedHeight(140)
        self._tabla.setFont(QFont('Monospace', 9))
        # Cada columna al ancho de su contenido y la primera absorbe lo que
        # sobra: con Stretch parejo la pose quedaba cortada en '(412,903) …',
        # que es justo el dato que uno mira.
        cab = self._tabla.horizontalHeader()
        cab.setSectionResizeMode(QHeaderView.ResizeToContents)
        cab.setStretchLastSection(True)
        caja.addWidget(self._tabla)
        return marco

    def _buildCommandPanel(self):
        marco = QFrame()
        marco.setObjectName('panel')
        caja = QVBoxLayout(marco)
        caja.setContentsMargins(10, 8, 10, 10)
        caja.setSpacing(8)

        fila = QHBoxLayout()
        fila.setSpacing(8)
        fila.addWidget(QLabel('Destino:'))
        self._robotCombo = QComboBox()
        self._robotCombo.setMinimumWidth(140)
        self._robotCombo.addItem('BROADCAST')
        fila.addWidget(self._robotCombo)

        self._cmdInput = QLineEdit()
        self._cmdInput.setPlaceholderText(
            'Comando crudo — ej. GT|500|300. ↑↓ recorre el historial, Enter envía')
        self._cmdInput.returnPressed.connect(self._send)
        self._cmdInput.installEventFilter(self)
        fila.addWidget(self._cmdInput, stretch=1)

        enviar = QPushButton('Enviar')
        enviar.setObjectName('primario')
        enviar.setFixedWidth(90)
        enviar.clicked.connect(self._send)
        fila.addWidget(enviar)
        caja.addLayout(fila)

        pestanas = QTabWidget()
        for nombre, botones in self._gruposDeComandos():
            pestanas.addTab(self._makeButtonGrid(botones), nombre)
        pestanas.setFixedHeight(140)
        caja.addWidget(pestanas)
        return marco

    def _gruposDeComandos(self):
        """Los comandos agrupados por para qué sirven.

        Cada botón es (etiqueta, ayuda, acción). La acción es un texto, y ahí
        importa el sufijo: si termina en '|' se deja escrito en el campo para que
        el usuario complete los argumentos, y si no, se manda tal cual. Los
        callables abren un diálogo.

        Hasta el 2026-08-01 acá había 16 comandos de los 29 que entiende el
        firmware. Faltaban MEET, todo el enjambre y toda la configuración en vivo.
        """
        return [
            ('Movimiento', [
                ('Avanzar', 'MOVE|<mm>', 'MOVE|'),
                ('Girar', 'TURN|<grados>', 'TURN|'),
                ('Esperar', 'WAIT|<ms>', 'WAIT|'),
                ('Random walk', 'RANDOMW|<ms>', 'RANDOMW|'),
                ('Ir a punto', 'GT|<x>|<y>', 'GT|'),
                ('Ir a global', 'GOTO.<robot> x y', self._dlgGoto),
                ('Abortar nav', 'corta navegación y búsqueda', 'ABORT_NAV'),
                ('Parar', 'vuelve a STOP', 'RESET'),
            ]),
            ('Enjambre', [
                ('MEET', 'congregación sobre un punto, sin líder', self._dlgMeet),
                ('Congregación', 'CONGREGATION.<líder>', self._dlgCongregacion),
                ('Formación', 'línea, cuña o círculo', self._dlgFormacion),
                ('Dispersar', 'DISPERSE|<mm de separación>', 'DISPERSE|'),
                ('Cancelar congr.', '', 'CANCEL_CONGREGATION'),
                ('Buscar objeto', 'SEARCH_OBJECT|<color>', 'SEARCH_OBJECT|'),
                ('Leer color', 'lee el APDS9960 y reporta RGBC', 'COLOR_READ'),
            ]),
            ('Sensores', [
                ('Máscaras IR', 'ignorar un sensor defectuoso', self._dlgMascara),
                ('Umbral central', 'SENSOR_THRESHOLD|C|<0-255>', self._dlgUmbral),
                ('Limpiar evasión', '', 'CLEAR_EVASION'),
                ('Reset evasión', 'borra todo el rastro de evasión', 'RESET_EVASION'),
                ('Autotest', 'motores e IMU en banco', 'SELFTEST'),
                ('Estado', 'GET_STATUS', 'GET_STATUS'),
                ('Yaw', 'GET_YAW', 'GET_YAW'),
            ]),
            ('Calibración', [
                ('Config. nav', 'arena, ruedas, yaw, parking, umbrales',
                 self._dlgNavConfig),
                ('Leer PPR', 'GETPPR', 'GETPPR'),
                ('Guardar PPR', 'SETPPR|<pulsos>[|SAVE]', 'SETPPR|'),
                ('PID', 'PID|<kp>|<ki>|<kd>', 'PID|'),
                ('Kalman PID', 'KFPID|<q>|<r>|<p>', 'KFPID|'),
                ('EKF nav', 'navegar con el EKF en vez del ArUco', self._dlgEkfNav),
                ('Calibrar', 'CALIBRATE.<robot>', self._dlgCalibrar),
                ('Recalibrar origen', 'fija el origen con el marker 5', '__ORIGIN__'),
            ]),
        ]

    def _makeButtonGrid(self, botones):
        cont = QWidget()
        rejilla = QGridLayout(cont)
        rejilla.setContentsMargins(10, 10, 10, 10)
        rejilla.setSpacing(6)
        for i, (etiqueta, ayuda, accion) in enumerate(botones):
            btn = QPushButton(etiqueta)
            if ayuda:
                btn.setToolTip(ayuda)
            if etiqueta in ('Parar', 'Abortar nav'):
                btn.setObjectName('peligro')
            if callable(accion):
                btn.clicked.connect(lambda _, f=accion: f())
            elif accion.endswith('|'):
                btn.clicked.connect(lambda _, c=accion: self._fillInput(c))
            else:
                btn.clicked.connect(lambda _, c=accion: self._quickSend(c))
            rejilla.addWidget(btn, i // 4, i % 4)
        rejilla.setRowStretch(rejilla.rowCount(), 1)
        return cont

    def _buildLog(self):
        marco = QFrame()
        marco.setObjectName('panel')
        caja = QVBoxLayout(marco)
        caja.setContentsMargins(10, 8, 10, 10)
        caja.setSpacing(6)

        fila = QHBoxLayout()
        titulo = QLabel('Mensajes')
        titulo.setFont(QFont('', 10, QFont.Bold))
        fila.addWidget(titulo)
        fila.addStretch()
        self._filtroDebug = QCheckBox('mostrar DEBUG')
        self._filtroDebug.setChecked(True)
        fila.addWidget(self._filtroDebug)
        limpiar = QPushButton('Limpiar')
        limpiar.clicked.connect(lambda: self._log.clear())
        fila.addWidget(limpiar)
        caja.addLayout(fila)

        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setMinimumHeight(120)
        self._log.setFont(QFont('Monospace', 9))
        caja.addWidget(self._log)
        return marco

    @staticmethod
    def _makeFrameLabel(placeholder):
        lbl = QLabel(placeholder)
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setMinimumSize(QSize(480, 270))
        lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        lbl.setStyleSheet(f'background:#101216; color:{TENUE};'
                          f'border:1px solid {BORDE}; border-radius:6px;')
        return lbl

    # ── panel de estado ──────────────────────────────────────────────────────

    def refreshRobots(self):
        """Repuebla el selector de destino con los robots registrados."""
        prev = self._robotCombo.currentText()
        self._robotCombo.clear()
        self._robotCombo.addItem('BROADCAST')
        for rid, r in sorted(self.base.robots.items()):
            self._robotCombo.addItem(f'{rid} ({r.name})', userData=rid)
        idx = self._robotCombo.findText(prev)
        if idx >= 0:
            self._robotCombo.setCurrentIndex(idx)

    def _refreshStatus(self, force=False):
        """Pide GET_STATUS y redibuja la tabla con lo último que llegó.

        El pedido va por broadcast en vez de robot por robot: son N datagramas
        contra uno, y a 2Hz con diez robots eso ya se nota en la misma red por la
        que viajan las órdenes de navegación.
        """
        if not (force or self._autoChk.isChecked()):
            return
        try:
            self.base.sendInstructionBroadcast(['GET_STATUS'])
        except Exception:
            pass                      # sin red todavía; la tabla igual se dibuja
        self._drawStatus()

    def _drawStatus(self):
        robots = sorted(self.base.robots.items())
        self._tabla.setRowCount(len(robots))
        ahora = time.time()
        for fila, (rid, robot) in enumerate(robots):
            st = getattr(robot, 'status', {}) or {}
            edad = ahora - getattr(robot, 'statusStamp', 0.0)
            x, y, ang = robot.getPose()
            visible = x != -1

            deriva = '—'
            if robot.ekfPose and visible:
                dx = robot.ekfPose[0] - x
                dy = robot.ekfPose[1] - y
                deriva = f'{(dx * dx + dy * dy) ** 0.5:.0f} mm'

            estado = st.get('State', '')
            if estado.isdigit() and int(estado) < len(self.ESTADOS):
                estado = self.ESTADOS[int(estado)]

            valores = [
                f'{rid} {robot.name}',
                'sí' if visible else 'NO',
                f'({x:.0f},{y:.0f}) {ang:.0f}°' if visible else '—',
                estado,
                'activa' if st.get('NAV') == '1' else '—',
                st.get('Sensors', '—'),
                st.get('Mask', '—'),
                st.get('Yaw', '—'),
                'ok' if st.get('IMU') == '1' else 'NO',
                deriva,
            ]
            for celda, texto in enumerate(valores):
                item = QTableWidgetItem(texto)
                item.setTextAlignment(Qt.AlignCenter)
                self._pintarCelda(item, celda, texto, st, edad)
                self._tabla.setItem(fila, celda, item)

    def _pintarCelda(self, item, celda, texto, st, edad):
        """Color por celda: lo que está mal tiene que saltar sin leer.

        Un robot sin STATUS reciente se atenúa entero en vez de mostrar datos
        viejos como si fueran de ahora — que es el error que hace perder tiempo
        cuando un robot se cae a mitad de una corrida.
        """
        if not st or edad > 6.0:
            item.setForeground(QColor(TENUE))
            return
        if celda == 1 and texto == 'NO':
            item.setForeground(QColor(ERROR))
        elif celda == 5 and texto not in ('—', '0-0-0', 'L0-C0-R0'):
            item.setForeground(QColor(ALERTA))
        elif celda == 6 and texto not in ('—', 'L0-C0-R0'):
            item.setForeground(QColor(ALERTA))
        elif celda == 8:
            item.setForeground(QColor(OK if texto == 'ok' else ERROR))
        elif celda == 4 and texto == 'activa':
            item.setForeground(QColor(ACENTO))

    # ── envío de comandos ────────────────────────────────────────────────────

    def _selectedRobotId(self):
        data = self._robotCombo.currentData()
        return data if data is not None else 'BROADCAST'

    def _send(self):
        cmd = self._cmdInput.text().strip()
        if not cmd:
            return
        self._dispatch(self._selectedRobotId(), cmd)
        if not self._cmdHistory or self._cmdHistory[0] != cmd:
            self._cmdHistory.insert(0, cmd)
        self._historyIdx = -1
        self._cmdInput.clear()

    def _quickSend(self, cmd):
        if cmd == '__ORIGIN__':
            self.base.recalibrateOrigin()
            self.logSignal.emit('[ORIGIN] Origen recalibrado')
            return
        self._dispatch(self._selectedRobotId(), cmd)

    def _fillInput(self, text):
        self._cmdInput.setText(text)
        self._cmdInput.setFocus()
        self._cmdInput.setCursorPosition(len(text))

    def _dispatch(self, robotId, instruction):
        """Delega en Base.dispatch(), el mismo despachador que usa la consola."""
        self.logSignal.emit(
            f'<span style="color:{ACENTO};"><b>&gt;&gt; {robotId}.{instruction}'
            f'</b></span>')
        try:
            self.base.dispatch(robotId, instruction, log=self.logSignal.emit)
        except Exception as e:
            self.logSignal.emit(
                f'<span style="color:{ERROR};">error: {e}</span>')

    # ── diálogos ─────────────────────────────────────────────────────────────

    def _pedirTexto(self, titulo, etiqueta, valor=''):
        texto, ok = QInputDialog.getText(self, titulo, etiqueta, text=valor)
        return texto.strip() if ok and texto.strip() else None

    def _elegir(self, titulo, etiqueta, opciones):
        op, ok = QInputDialog.getItem(self, titulo, etiqueta, opciones, 0, False)
        return op if ok else None

    def _dlgMeet(self):
        """MEET|x|y[|radio] — congregación sobre un punto, sin líder.

        Se manda por broadcast siempre: la gracia de MEET es que los N robots
        resuelven el mismo reparto de slots desde el mismo mensaje. Mandárselo a
        uno solo no congrega nada.
        """
        punto = self._pedirTexto('MEET', 'Punto de encuentro  x y  (mm):', '2200 850')
        if not punto:
            return
        partes = punto.split()
        if len(partes) < 2:
            self.logSignal.emit('MEET: hacen falta x e y')
            return
        radio = self._pedirTexto('MEET', 'Radio del anillo en mm (vacío = lo '
                                         'calcula el firmware):')
        cmd = f'MEET|{partes[0]}|{partes[1]}' + (f'|{radio}' if radio else '')
        self._dispatch('BROADCAST', cmd)

    # Los cuatro de abajo van por BASE.<verbo>: son cosas que ORQUESTA la base,
    # no mensajes que se reenvíen tal cual a un robot. Ver la tabla _BASE_CMDS
    # de AttaBot_Base, que es la misma que alimenta el autocompletado y la ayuda.

    def _dlgCongregacion(self):
        lider = self._pedirTexto('Congregación', 'ID del robot líder:')
        if lider:
            self._dispatch('BASE', f'CONGREGATION|{"|".join(lider.split())}')

    def _dlgFormacion(self):
        figura = self._elegir('Formación', 'Figura:', ['linea', 'cuna', 'circulo'])
        if not figura:
            return
        resto = self._pedirTexto('Formación', 'ID del líder [espaciado en mm]:')
        if resto:
            self._dispatch('BASE', f'FORMATION|{figura}|{"|".join(resto.split())}')

    def _dlgGoto(self):
        destino = self._pedirTexto('Ir a global', 'robotID  x  y :')
        if destino:
            self._dispatch('BASE', f'GOTO|{"|".join(destino.split())}')

    def _dlgCalibrar(self):
        robot = self._pedirTexto('Calibrar', 'ID del robot:')
        if robot:
            self._dispatch('BASE', f'CALIBRATE|{robot.strip()}')

    def _dlgMascara(self):
        """SENSOR_MASK|<L|C|R>|<0|1> — ignorar un sensor.

        Se usa seguido y a mano: Atta_1 tiene el infrarrojo derecho fantasma, así
        que en cada sesión de lab hay que enmascararlo antes de navegar.
        """
        sensor = self._elegir('Máscaras IR', 'Sensor:',
                              ['L — izquierdo', 'C — central', 'R — derecho'])
        if not sensor:
            return
        accion = self._elegir('Máscaras IR', f'¿Qué hago con {sensor[0]}?',
                              ['ignorar (1)', 'volver a usar (0)'])
        if accion:
            self._quickSend(f'SENSOR_MASK|{sensor[0]}|{"1" if "ignorar" in accion else "0"}')

    def _dlgUmbral(self):
        valor = self._pedirTexto(
            'Umbral del IR central',
            'Proximidad 0-255 (mayor = detecta más cerca):')
        if not valor:
            return
        guardar = self._elegir('Umbral del IR central', '¿Persistir en NVS?',
                               ['solo por ahora', 'guardar'])
        sufijo = '|SAVE' if guardar == 'guardar' else ''
        self._quickSend(f'SENSOR_THRESHOLD|C|{valor}{sufijo}')

    def _dlgEkfNav(self):
        op = self._elegir('EKF nav', 'Fuente de pose para navegar:',
                          ['ArUco crudo (0)', 'EKF fusionado (1)'])
        if op:
            self._quickSend(f'EKF_NAV|{"1" if "EKF" in op else "0"}')

    # Qué significa cada clave y en qué rango la acepta el firmware. Los rangos
    # importan: fuera de rango el robot descarta el valor EN SILENCIO, sin
    # contestar nada, así que la única defensa es mostrarlos al escribir.
    NAV_CONFIG_AYUDA = {
        'SEGMENT_DIST':      ('largo del tramo de avance', '50–400 mm'),
        'ARRIVAL_THRESHOLD': ('cuándo se da por llegado', '5–200 mm'),
        'PARKING_DIST':      ('radio de congregación', '150–600 mm'),
        'WHEEL_DIST':        ('media distancia entre ruedas', '20–100 mm'),
        'REALIGN':           ('error de rumbo que dispara recorrección', '1–90°'),
        'GOAL_DEADBAND':     ('zona muerta alrededor de la meta', '0–300 mm'),
        'ARENA':             ('límites del escenario: ancho alto', '>100 mm c/u'),
        'YAW_SCALE':         ('escala del gyro', '0.9–1.1'),
    }

    # Único parámetro que el firmware escribe en NVS: a los demás el sufijo SAVE
    # les entra por un oído y les sale por el otro.
    NAV_CONFIG_PERSISTE = {'YAW_SCALE'}

    def _dlgNavConfig(self):
        """NAV_CONFIG|<clave>|<valor>[|SAVE] — configuración en vivo."""
        claves = _leerClavesNavConfig()
        if not claves:
            self.logSignal.emit(
                f'<span style="color:{ERROR};">no se pudo leer comandos.ino; '
                f'escribí el NAV_CONFIG a mano</span>')
            return
        etiquetas = []
        for c in claves:
            desc, rango = self.NAV_CONFIG_AYUDA.get(c, ('', ''))
            etiquetas.append(f'{c} — {desc} ({rango})' if desc else c)
        elegida = self._elegir('Configuración de navegación', 'Parámetro:',
                               etiquetas)
        if not elegida:
            return
        clave = elegida.split(' —')[0].strip()
        _, rango = self.NAV_CONFIG_AYUDA.get(clave, ('', ''))
        pregunta = f'Valor para {clave}'
        valor = self._pedirTexto('Configuración de navegación',
                                 f'{pregunta} [{rango}]:' if rango
                                 else f'{pregunta}:')
        if not valor:
            return
        sufijo = ''
        if clave in self.NAV_CONFIG_PERSISTE:
            guardar = self._elegir('Configuración de navegación',
                                   '¿Persistir en NVS?',
                                   ['solo por ahora', 'guardar'])
            sufijo = '|SAVE' if guardar == 'guardar' else ''
        valor = '|'.join(valor.split())          # ARENA lleva dos números
        self._quickSend(f'NAV_CONFIG|{clave}|{valor}{sufijo}')

    # ── historial del campo de comandos ──────────────────────────────────────

    def eventFilter(self, obj, event):
        from PyQt5.QtCore import QEvent
        if obj is self._cmdInput and event.type() == QEvent.KeyPress:
            key = event.key()
            if key == Qt.Key_Up and self._cmdHistory:
                self._historyIdx = min(self._historyIdx + 1,
                                       len(self._cmdHistory) - 1)
                self._cmdInput.setText(self._cmdHistory[self._historyIdx])
                return True
            if key == Qt.Key_Down:
                self._historyIdx = max(self._historyIdx - 1, -1)
                self._cmdInput.setText(
                    self._cmdHistory[self._historyIdx]
                    if self._historyIdx >= 0 else '')
                return True
        return super().eventFilter(obj, event)

    # ── slots ────────────────────────────────────────────────────────────────

    @pyqtSlot(object, object)
    def _onFrame(self, cam_frame, results_frame):
        if cam_frame is not None:
            self._camLabel.setPixmap(
                self._npToPixmap(cam_frame, self._camLabel.size()))
        if results_frame is not None:
            self._mapLabel.setPixmap(
                self._npToPixmap(results_frame, self._mapLabel.size()))

    @pyqtSlot(str)
    def _onLog(self, msg):
        if 'DEBUG' in msg and not self._filtroDebug.isChecked():
            return
        self._log.append(msg)
        barra = self._log.verticalScrollBar()
        barra.setValue(barra.maximum())

    @staticmethod
    def _npToPixmap(frame, target):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QImage(rgb.tobytes(), w, h, ch * w, QImage.Format_RGB888)
        return QPixmap.fromImage(qimg).scaled(
            target, Qt.KeepAspectRatio, Qt.SmoothTransformation)


def launch(base_instance):
    """Arranca la aplicación: pregunta cuántos robots, lanza la cámara en un
    hilo de fondo y corre el event loop de Qt en el principal."""
    import threading

    app = QApplication.instance() or QApplication(sys.argv)

    numRobots, ok = QInputDialog.getInt(
        None, 'AttaBot', 'Cantidad de robots en la prueba:', 1, 1, 12)
    if not ok:
        return

    # Mismo anclaje que AttaBot_Base.main(): las rutas del programa son
    # relativas (configSystem.json y los directorios Videos/PositionLogs/
    # ConsoleLogs/Logs), así que sin esto la GUI solo arranca desde Base/.
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    base_instance.numRobots = numRobots
    base_instance.readConfigFile('configSystem.json')

    gui = AttaBotGUI(base_instance)
    base_instance.gui = gui
    gui.show()

    cam_thread = threading.Thread(target=base_instance.cameraProcessing,
                                  daemon=True)
    cam_thread.start()

    sys.exit(app.exec_())


if __name__ == '__main__':
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from AttaBot_Base import base
    launch(base)
