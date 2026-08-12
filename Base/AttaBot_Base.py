import os
os.environ['QT_QPA_PLATFORM'] = 'xcb'  # forzar xcb — cv2 no tiene plugin wayland
import cv2, json, math, re, sys, time, socket, threading, csv, queue, platform
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


# Telemetría de alta frecuencia: no son comandos del operador y taparían el
# ConsoleLog. POSE sale por robot y por cuadro (hasta 20Hz), POSITION_RESPONSE
# por cada pedido del robot, y LEADER_POSITION la retransmite la Base a 4Hz en
# simulación. Ver Base.logCommand.
_NO_LOG_CMD = frozenset(('POSE', 'POSITION_RESPONSE', 'LEADER_POSITION',
                         'NEIGHBOR_POSITIONS'))


# =============================================================================
# VOCABULARIO DE COMANDOS
# =============================================================================
# Fuente ÚNICA para el despachador, el autocompletado y la ayuda. Antes cada
# cara tenía su propia lista y se desincronizaban: la GUI entendía BROADCAST,
# CONGREGATION, GOTO y STATUS pero no FORMATION, CALIBRATE ni OCCLUDE durante
# meses. Agregar un comando acá lo hace aparecer en los tres lugares a la vez.
#
# Gramática:  DESTINO.VERBO|arg|arg
#   BASE.<verbo>       la Base ejecuta algo (orquesta, consulta, calibra)
#   <id>.<verbo>       se envía tal cual por UDP a ese robot
#   BROADCAST.<verbo>  se envía tal cual a todos
#
# La separación por DESTINO no es cosmética: CONGREGATION, FORMATION, GOTO y GT
# existen TAMBIÉN como comandos del firmware, así que '1.CONGREGATION' sería
# ambiguo (¿lo orquesta la base o se lo mando crudo al robot?). Con BASE. no hay
# colisión posible.

# Comandos que viajan al robot. Se excluyen a propósito los de telemetría y los
# que van en sentido robot→base (POSE, CHECK_OBSTACLE, MESSAGE_BASE...): no son
# cosas que un operador escriba.
#
# El string es la AYUDA para el humano, no una gramática: '0|1' y 'L|C|R' son
# enumeraciones, no varios argumentos. Las firmas exactas — las que salen de
# leer los Handle*() del firmware — están en comandos_schema.py, y
# `python Base/comandos_schema.py` avisa si esta tabla y aquélla se separan.
# Vale la pena correrlo al tocar un handler: cuatro entradas de acá estuvieron
# equivocadas y el firmware descarta EN SILENCIO lo que no entiende, así que
# seguir la ayuda daba un robot quieto y ningún mensaje de error.
_ROBOT_CMDS = {
    'MOVE':             'mm',
    'TURN':             'grados',
    # Un solo nombre para navegar a un punto. GOTO, POSITIONGT y BUG2 caían en
    # el MISMO handler del firmware — no eran variantes, eran cuatro nombres
    # para el mismo código, y había que leer el firmware para saberlo.
    'GT':               'x|y[|segmento_mm]',
    'RANDOMW':          '[segmento_mm]',
    'MEET':             'x|y[|radio]',
    # Decía 'slot|x|y', que sonaba plausible y no era: el firmware lee líder,
    # índice y total. Quien mandaba coordenadas ponía el índice en la X.
    'CONGREGATION':     'liderID|indiceSeguidor[|total]',
    'FORMATION':        'figura|liderID|idx|n|eje',
    'DISPERSE':         '[separacion_mm]',
    'CANCEL_CONGREGATION': '',
    'ABORT_NAV':        '',
    # Figuraba sin argumentos; el firmware exige el color a buscar.
    'SEARCH_OBJECT':    'rojo|verde|azul',
    'COLOR_READ':       '',
    'RESET':            '',
    'WAIT':             'ms',
    'GET_STATUS':       '',
    'GET_YAW':          '',
    'GETPPR':           '',
    'SETPPR':           'valor|TEMP|SAVE',
    'PID':              'kp|ki|kd[|SAVE]',
    # Decía 'q|r'. Son TRES y en este orden: HandleKalmanPID lee R, H y Q.
    'KFPID':            'R|H|Q',
    'NAV_CONFIG':       'clave|valor[|SAVE]',
    'SENSOR_MASK':      'L|C|R|0o1',
    # Decía 'valor' a secas y el firmware exige la 'C' delante, así que
    # 'SENSOR_THRESHOLD|25' no hacía nada ni avisaba.
    'SENSOR_THRESHOLD': 'C|valor[|SAVE]',
    'SELFTEST':         '[pwm]',
    'EKF_NAV':          '0|1',
    'CLEAR_EVASION':    '',
    'RESET_EVASION':    '',
    'CONFIG':           'SAVE|id',
    'SEND_COUNT_MESSAGE': '',
}

# Comandos que ejecuta la Base. El id del robot es un ARGUMENTO, no el destino.
_BASE_CMDS = {
    'STATUS':       '',
    'CALIBRATE':    'robotID',
    'CONGREGATION': 'liderID[|espaciado_mm]',
    'FORMATION':    'linea|cuna|circulo|liderID[|espaciado_mm]',
    # GOTO salió el 2026-08-05: 'BASE.GOTO|1|1200|850' era exactamente
    # '1.GT|1200|850' más un print, y con el campo de comandos aceptando destino
    # explícito ya no aportaba nada.
    'OCCLUDE':      'segundos   (solo --sim)',
    'HELP':         '[verbo]',
}

# Formas viejas verbo-primero, para no romper la memoria muscular ni la GUI.
# Mapean a la forma canónica y avisan una vez por comando.
_LEGACY_VERBS = frozenset(('STATUS', 'CALIBRATE', 'CONGREGATION', 'FORMATION',
                           'OCCLUDE'))

# Agrupación de _ROBOT_CMDS para las pestañas de la GUI. Es metadata de
# PRESENTACIÓN, no otra lista de comandos: la GUI recorre _ROBOT_CMDS y consulta
# acá a qué pestaña va cada uno. Lo que no figure cae en 'Otros', así que un
# comando nuevo nunca desaparece de la interfaz — a lo sumo queda mal agrupado,
# que se ve a simple vista. Ese es justo el fallo que se quiere evitar: la GUI
# llegó a mostrar 16 de 29 comandos porque tenía su propia lista escrita a mano.
_CMD_GROUP = {
    'Movimiento':  ('MOVE', 'TURN', 'WAIT', 'RESET'),
    'Navegación':  ('GT', 'RANDOMW', 'ABORT_NAV'),
    'Enjambre':    ('MEET', 'DISPERSE', 'CONGREGATION', 'FORMATION',
                    'CANCEL_CONGREGATION', 'SEARCH_OBJECT', 'COLOR_READ'),
    'Sensores':    ('SENSOR_MASK', 'SENSOR_THRESHOLD', 'CLEAR_EVASION',
                    'RESET_EVASION'),
    'Calibración': ('GETPPR', 'SETPPR', 'PID', 'KFPID', 'NAV_CONFIG',
                    'SELFTEST', 'EKF_NAV'),
    'Diagnóstico': ('GET_STATUS', 'GET_YAW', 'CONFIG', 'SEND_COUNT_MESSAGE'),
}


def videoWriter(frameResolution, numRobots, pathVideo, processInterval, frameQueue,
                captureFps=None):
    """
    Graba un video en el disco con los cuadros que llegan a través de una cola.

    Lee pares de fotogramas de `frameQueue`, los une en uno solo y los guarda en un
    archivo AVI cuyo nombre incluye la cantidad de robots y la fecha y hora actuales.

    Corre en un HILO, no en un proceso. La codificación XVID libera el GIL (medido:
    el trabajo Python del hilo principal no se degrada con el grabador a full, 0.98x),
    así que un hilo hace el mismo trabajo sin pagar el pickling de dos frames BGR por
    cuadro — 5.5 MB en lab y 16 MB en sim, que eran 4.4 y 10.9 ms de la ventana de
    procesamiento. A cambio, quien encola debe pasar frames que no vaya a mutar
    después (ver addFrame).

    Parámetros:
    frameResolution (tuple): Resolución original de los fotogramas de entrada (alto, ancho).
    numRobots (int): Número de robots, usado para el nombre del archivo de video.
    pathVideo (str): Directorio donde se guardará el video generado.
    processInterval (float): Intervalo de procesamiento en segundos para calcular los FPS.
    frameQueue (queue.Queue): Cola de fotogramas `[frame, resultsFrame]`; None termina.

    Retorna:
    None
    """
    currentTime = datetime.now().strftime(r'%d-%m_%H-%M')
    videoName = f'Video_{currentTime}_Robots_{numRobots}.avi'
    pathVideo = os.path.join(pathVideo, videoName)
    resolution = (frameResolution[1], frameResolution[0] * 2)

    # El ritmo real del video es el de la CÁMARA: con la compuerta de
    # frame_processing_interval por debajo del período de captura, se procesa y
    # se graba exactamente un cuadro por cada cuadro que entrega la cámara.
    #
    # La fórmula vieja era '1/processInterval - 1', un número que no salía de
    # ningún lado: con el intervalo en 0.045 declaraba 21.2 fps contra 19.6
    # reales — 8% de error, o 45 segundos de desfase a los 9 minutos de corrida.
    # De ahí venía la regla de "ubicar los instantes por número de cuadro y no
    # por el reloj del reproductor".
    fps = float(captureFps) if captureFps else 1.0 / processInterval
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    video = cv2.VideoWriter(pathVideo, fourcc, fps, resolution)

    escritos = 0
    primero = None
    while True:
        frames = frameQueue.get()

        if frames is None:
            break

        frame, resultsFrame = frames

        # Solo grabación — el display lo maneja la GUI o el proceso principal
        results = cv2.vconcat([frame, resultsFrame])
        video.write(results)
        if primero is None:
            primero = time.time()
        escritos += 1

    video.release()

    # Ningún fps constante puede ser exacto: un cuadro que la cámara pierde
    # tampoco queda en el video, así que el archivo dura menos que la corrida.
    # Por eso se reporta el ritmo medido — es el factor con el que corregir si
    # hace falta cruzar el reloj del reproductor con el de los logs. El sello de
    # tiempo impreso sobre cada cuadro sigue siendo la referencia exacta.
    if escritos >= 2 and primero is not None:
        transcurrido = time.time() - primero
        real = escritos / transcurrido if transcurrido > 0 else fps
        print(f'✓ Video: {escritos} cuadros, {real:.2f} fps reales '
              f'(declarado {fps:.2f})')
        if abs(real - fps) > 0.05 * fps:
            print(f'  ⚠ se va a reproducir {100 * (fps / real - 1):+.0f}% de '
                  f'velocidad — para ubicar un instante usá el reloj impreso '
                  f'en el cuadro, no el del reproductor')


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
        # Último GET_STATUS parseado a {clave: valor}, para el panel de la GUI.
        self.status = {}
        self.statusStamp = 0.0
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
        # FPS que la cámara declara tras negociar el formato. Es el ritmo real
        # al que se graba el video; None en sim, donde no hay cámara.
        self.captureFps = None
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
        # Verbos de la gramática vieja ya avisados: el recordatorio sale una vez
        # por verbo y no en cada comando, que sería ruido en medio de una corrida.
        self._legacyWarned = set()
        self.broadcastIP = ''
        self.port = int
        self.threadInputAlive = True
        self.pathVideo = ''
        self.pathPositionLogs = ''
        self.pathConsolelog = ''
        # Handles de los CSV, vivos mientras dura la corrida (ver _openCsvLog)
        self._timeLogFile = self._timeLogWriter = None
        self._positionLogFile = self._positionLogWriter = None
        self._consoleLogFile = self._consoleLogWriter = None
        # El ConsoleLog es el único que se escribe desde varios hilos: lo que
        # llega por UDP y, desde que se loguean los comandos, también lo que
        # mandan la GUI, la consola y la rutina de calibración.
        self._consoleLogLock = threading.Lock()
        self.cameraResolution = []
        self.newCameraMatriz = None
        self.roi = None
        self.frameQueue = None
        self.videoThread = threading.Thread()
        self.congregationActive = False
        self.leaderID = None
        self.robotPositions = {}
        # --- ArUco ---
        self.arucoDetector = None
        self.markerSizeMm = 80.0          # valor por defecto, sobreescrito desde JSON
        self.currentArucoDetections = {}    # raw: {robot_id: (x_mm, y_mm, angle_deg)}
        # Última salida cruda de detectMarkers, para que drawArucoDebug dibuje sin
        # volver a detectar sobre el mismo frame (ver detectArucoMarkers).
        self._lastCorners = ()
        self._lastIds = None
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
        # Throttle del aviso de envío fallido (ver sendInstruction)
        self._lastSendErrorLog = 0.0


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
        if parts[0] not in ('GT', 'MEET') or len(parts) < 3:
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
        # detectMarkers cuesta 18ms de los 50 que dura la ventana de procesamiento,
        # y drawArucoDebug lo repetía sobre este mismo frame — con debug_enable en
        # true (la config del lab) eso era el 36% del presupuesto gastado dos veces.
        # Se guarda el resultado para que el dibujo lo reuse en vez de re-detectar.
        self._lastCorners, self._lastIds = corners, ids

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

        Reusa la detección que acaba de hacer detectArucoMarkers sobre este mismo
        frame (processFrame llama a una y después a la otra, sin leer la cámara en
        el medio), en vez de volver a correr detectMarkers. El solvePnP de acá sí
        se repite a propósito: son 0.09ms para 4 markers y guardarse los rvec/tvec
        sería estado extra para no ahorrar nada medible.

        Parámetros:
        - frame (ndarray): Frame BGR donde dibujar las anotaciones.

        Retorna:
        - ndarray: Frame anotado.
        """
        corners, ids = self._lastCorners, self._lastIds

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

        # Buffer de 2 y no de 1: decodificar el MJPEG cuesta ~20ms y con un solo
        # buffer el driver no tenía dónde poner el cuadro que llegaba mientras
        # tanto, así que lo tiraba. Medido 2026-08-05 en bucle apretado: con
        # buffer=1 read() da 62.7ms (15.9 FPS), con buffer=2 da 32.3ms (30.9).
        # Sigue siendo chico a propósito, para no acumular cuadros viejos.
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        # MJPEG permite 1080p @ 30 FPS por USB; sin esto V4L2 usa YUYV (~5 FPS)
        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        # 20 y no 30: el lazo solo procesa un cuadro cada frame_processing_interval,
        # así que a 30 FPS se decodificaban 30 por segundo para usar 15 y tirar el
        # resto. A 20 el período de la cámara (50ms) queda justo arriba de la
        # compuerta (45ms) y se procesa CADA cuadro que llega. La C920 soporta
        # 20.000 fps exactos a 1280x720 MJPG.
        self.camera.set(cv2.CAP_PROP_FPS, 20)
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
        # Ritmo con el que se declara el .avi: es el de la cámara, no el de la
        # compuerta de procesamiento. Ver videoWriter.
        self.captureFps = actual_fps if actual_fps and actual_fps > 1 else None

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

    def cameraCorrection(self, frame):
        """
        Desdistorsiona y recorta la imagen de la cámara.

        Parámetros:
        - frame (ndarray): La imagen de entrada en formato BGR.

        Returns:
        - frame (ndarray): Imagen corregida en formato BGR.
        """
        x, y, w, h = self.roi
        return cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)[y:y+h, x:x+w]


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
        """
        if self.simMode:
            # Las detecciones vienen del feed CAM (ya en mm/grados del lab)
            raw = self.simVision.snapshot()
        else:
            frame = self.cameraCorrection(frame)
            raw = self.detectArucoMarkers(frame)

        self.currentArucoDetections = raw

        if self.debug:
            self.cameraDebug(frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.debug = 'off'

        return frame


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
                        not self.videoThread.is_alive()) and executed:
                    self.cleanup()
                    break
                continue

            lastProcessedTime = time.time()
            frame = self.processFrame(frame)

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

        self.frameQueue = queue.Queue()
        args = (
            resolution,
            self.numRobots,
            self.pathVideo,
            self.processInterval,
            self.frameQueue,
            self.captureFps,
        )
        self.videoThread = threading.Thread(target=videoWriter, args=args, daemon=True)
        self.videoThread.start()
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
            # El centinela va al final de la cola: el grabador termina de escribir
            # lo que quede pendiente y recién ahí suelta el archivo. Antes se
            # drenaba la cola después de encolarlo, con lo que el drenaje podía
            # comerse el propio centinela y dejar el .avi sin cerrar.
            self.frameQueue.put(None)
            self.videoThread.join(timeout=10.0)
            if self.videoThread.is_alive():
                print('⚠ el grabador de video no terminó en 10s — '
                      'el .avi puede quedar truncado')

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
        # El grabador ya no es un proceso, así que no recibe una copia implícita
        # por pickle: comparte memoria con este loop. `resultsFrame` es el ÚNICO
        # que se muta en el sitio (el mapa de cobertura se va pintando encima
        # frame a frame), así que hay que congelarlo o el video mostraría la
        # cobertura del momento de codificar, no la del cuadro. `frame` es un
        # arreglo nuevo en cada iteración y no hace falta copiarlo.
        # La copia se comparte con la GUI: _npToPixmap convierte y copia de
        # inmediato, nunca se queda con el arreglo.
        mapSnapshot = resultsFrame.copy()
        if self.gui is not None:
            self.gui.frameSignal.emit(frame.copy(), mapSnapshot)
        if self.frameQueue is not None:
            self.frameQueue.put([frame, mapSnapshot])


    # =========================================================================
    # LOGGING
    # =========================================================================

    def _openCsvLog(self, path, header):
        """
        Abre un CSV de log, escribe su cabecera y deja el handle vivo.

        Los tres logs se escriben fila a fila desde el loop de cámara o desde el
        hilo de UDP, y abrir y cerrar el archivo en cada fila costaba 25µs contra
        los 5µs de escribir sobre un handle ya abierto: con 10 robots eran 0.25ms
        de cada ventana de 50ms gastados en syscalls.

        Se hace flush() por fila para que la durabilidad no cambie — igual que
        antes, los datos quedan en manos del sistema operativo apenas se escriben
        y una corrida interrumpida conserva todo lo logueado hasta ese instante.
        Por eso tampoco se cierran los handles en cleanup(): no habría nada que
        salvar, y el hilo de UDP sigue vivo y podría escribir sobre un archivo ya
        cerrado. Los cierra el sistema al terminar el proceso.
        """
        f = open(path, 'w', newline='')
        writer = csv.writer(f)
        writer.writerow(header)
        f.flush()
        return f, writer


    def createTimeLog(self):
        """Crea un archivo CSV de registro de tiempos de procesamiento."""
        currentTime = datetime.now().strftime(r'%d-%m_%H-%M')
        logName = f'Time_Log_{currentTime}_Robots_{self.numRobots}.csv'
        os.makedirs('Logs', exist_ok=True)
        self.pathTimeLogs = os.path.join('Logs', logName)
        self._timeLogFile, self._timeLogWriter = self._openCsvLog(
            self.pathTimeLogs, ['time', 'processingTime'])


    def addTimeLog(self, timeLog, processingTime):
        """Agrega una entrada al registro de tiempo de procesamiento."""
        self._timeLogWriter.writerow([timeLog, processingTime])
        self._timeLogFile.flush()


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
        self._positionLogFile, self._positionLogWriter = self._openCsvLog(
            self.pathPositionLogs, header)


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
        self._positionLogWriter.writerow(row)
        self._positionLogFile.flush()


    def createConcoleLog(self):
        """Crea un archivo CSV de registro de mensajes UDP recibidos."""
        currentTime = datetime.now().strftime(r'%d-%m_%H-%M')
        logName = f'Console_Log_{self.logTag}{currentTime}_Robots_{self.numRobots}.csv'
        self.pathConsolelog = os.path.join(self.pathConsolelog, logName)
        header = ['time', 'idrobot', 'robot', 'message']
        self._consoleLogFile, self._consoleLogWriter = self._openCsvLog(
            self.pathConsolelog, header)


    def addConcoleLog(self, timeLog, id, name, message):
        """Agrega una entrada al registro de la consola UDP."""
        with self._consoleLogLock:
            self._consoleLogWriter.writerow([timeLog, id, name, message])
            self._consoleLogFile.flush()


    def logCommand(self, dest, instruction):
        """
        Registra en el ConsoleLog un comando que la Base ENVÍA.

        Hasta ahora el log solo tenía lo que decían los robots, así que las fases
        de una corrida había que inferirlas: analyze_logs deduce el arranque del
        MEET por el primer REQUEST_POSITION, porque durante el random walk el
        robot nunca pide su pose. El proxy funciona, pero con el comando en el
        log la fase se SABE en vez de deducirse, y encima queda registrado qué se
        mandó y a quién — hoy eso solo vive en la terminal del operador.

        Las filas llevan el prefijo `CMD|`, que ningún consumidor existente mira:
        analyze_logs filtra por `CHECK_OBSTACLE`, por 'soy líder' y por
        `REQUEST_POSITION` exacto, y scan_logs por patrones de texto del robot.
        Así los logs nuevos se siguen leyendo con el código de siempre.

        Parámetros:
        - dest (str): IP del robot destino, o 'BROADCAST'. Si no corresponde a
          ningún robot conocido la fila va con idrobot -1, igual que hace
          readUdpConnection con los peers que no reconoce.
        - instruction (str): el comando tal cual salió por el socket.
        """
        if self._consoleLogWriter is None:
            return      # la corrida todavía no arrancó: no hay log ni startTime
        if instruction.split('|')[0] in _NO_LOG_CMD:
            return
        robot = next((r for r in self.robots.values() if r.IP == dest), None)
        rid, name = (robot.id, robot.name) if robot else ('-1', dest)
        self.addConcoleLog(round(time.time() - self.startTime, 3), rid, name,
                           f'CMD|{instruction}')


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
                self.logCommand('BROADCAST', instruction)
                print(f"(Broadcast sim) Mensaje enviado: {instruction}")
            return
        for instruction in instructions:
            self.sock.sendto(instruction.encode(), (self.broadcastIP, self.port))
            self.logCommand('BROADCAST', instruction)
            print(f"(Broadcast) Mensaje enviado: {instruction}")


    def sendInstruction(self, ip, instructions, printing):
        """
        Envía instrucciones a un robot específico por IP (o 'ip:puerto' en sim).

        Es SÍNCRONO. Antes cada llamada arrancaba un hilo, y con el loop de
        posiciones mandando una POSE por robot por cuadro eso eran 80-200 hilos
        por segundo, a 109µs cada uno, para un sendto de 30 bytes por UDP que no
        bloquea. El orden de llegada no era el motivo: medido, 400 envíos
        consecutivos llegaban los 400 en orden con hilos y sin ellos.

        Lo único que el hilo aportaba de verdad era aislar al llamador de un
        OSError — robot apagado, interfaz caída —, y eso no es hipotético: sin
        aislamiento, un envío fallido dentro de sendPositions se lleva puesto el
        loop de cámara y termina la corrida. De eso se encarga ahora el except de
        acá, que además deja un mensaje legible en vez del traceback de un hilo
        muerto.

        Parámetros:
        - ip (str): Dirección IP del robot.
        - instructions (list): Lista de instrucciones a enviar.
        - printing (bool): Si True, imprime confirmación en consola.
        """
        try:
            for instruction in instructions:
                self.sock.sendto(instruction.encode(), self._robotAddr(ip))
                self.logCommand(ip, instruction)
                if printing:
                    # El barrido para resolver el nombre estaba fuera del if y se
                    # descartaba: el loop de posiciones manda con printing=False.
                    name = next((robot.name for robot in self.robots.values()
                                 if robot.IP == ip), ip)
                    self.log(f'Mensaje enviado a {name}: {instruction}')
        except OSError as e:
            # Un robot caído se reintenta a la cadencia del loop de posiciones,
            # así que sin throttle esto son ~20 líneas por segundo por robot.
            now = time.time()
            if now - self._lastSendErrorLog >= 2.0:
                self._lastSendErrorLog = now
                self.log(f'⚠ no se pudo enviar a {ip}: {e} [log 1/2s]')


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
                    # El firmware manda 'REQUEST_POSITION' pelado (estados.ino).
                    # Acá se parseaba además un 'REQUEST_POSITION|BUG2|...' que
                    # ningún firmware emite desde que se borró ese algoritmo:
                    # era una rama que no podía ejecutarse nunca.
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

                elif command == 'STATUS':
                    # Respuesta a GET_STATUS: campos 'clave:valor' separados por
                    # '|'. Se guardan crudos en el robot para que la GUI arme su
                    # panel sin volver a parsear. Antes esta respuesta caía al log
                    # como una línea de 300 caracteres que había que leer a ojo.
                    if robotFound:
                        estado = {}
                        for campo in parts[1:]:
                            if ':' in campo:
                                k, v = campo.split(':', 1)
                                estado[k] = v
                        self.robots[id].status = estado
                        self.robots[id].statusStamp = time.time()

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
            """Réplica de formation_slot del robot — para validar límites.

            Vale para linea y cuna, que el firmware calcula con esta misma
            fórmula. NO vale para circulo: allá el firmware usa
            SafeRingSlotAngle, que agranda el radio hasta 2.5x y confina los
            slots al arco más largo libre de paredes. Por eso el círculo no se
            valida con esto (ver más abajo).
            """
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

        # Validar que TODOS los slots caigan dentro de la ARENA (con margen para
        # staging+robot). Antes el límite era cameraResolution × mmPixel, que no
        # es ni la arena ni el FOV: es la escala del mapa de display. En el lab
        # daba 2170x1221mm contra una arena de 2400x1750, así que el techo caía
        # en y=971 — apenas por encima del centro (y=875) — y CUALQUIER slot
        # colocado más arriba que un líder centrado se rechazaba. La pose real
        # sale de solvePnP, no de esa escala: hay robots medidos en y=1594.
        # Mismo error que en GT el 2026-07-29; ver arenaMm().
        maxX, maxY = self.arenaMm()
        inset = 250.0

        def culpables(axisDeg):
            """Slots que se salen, con cuánto se pasan. Vacío = cabe."""
            fuera = []
            for idx in range(n):
                ox, oy = slotOffset(shape, idx, axisDeg)
                sx, sy = lx + ox, ly + oy
                exceso = max(inset - sx, sx - (maxX - inset),
                             inset - sy, sy - (maxY - inset))
                if exceso > 0:
                    fuera.append((idx, sx, sy, exceso))
            return fuera

        axis = 0.0
        if shape == 'circulo':
            # El círculo NO se rechaza. SafeRingSlotAngle ya garantiza en el
            # firmware que ningún slot toque la pared: agranda el radio hasta
            # 2.5x y reparte los slots en el arco libre más largo, con su propio
            # margen de 200mm. Validarlo acá contra un anillo plano de radio
            # nominal solo producía rechazos falsos — negaba círculos que el
            # robot habría colocado bien. Se avisa, eso sí, porque el radio real
            # puede terminar siendo bastante mayor que el pedido.
            aprietan = culpables(0.0)
            if aprietan:
                print(f'ℹ El anillo de {spacing:.0f}mm no entra entero donde '
                      f'está el líder ({lx:.0f},{ly:.0f}): el firmware va a '
                      f'agrandar el radio o juntar los slots en el arco libre.')
        else:
            fuera = culpables(0.0)
            if fuera and shape == 'linea' and not culpables(90.0):
                axis = 90.0
                print('⚠ La fila perpendicular no cabe — usando el eje del '
                      'heading del líder (columna)')
            elif fuera:
                # Decir QUÉ slot falla y por cuánto. Sin esto, un rechazo por
                # 9mm (pasó el 2026-08-05) es indistinguible de uno por medio
                # metro, y no hay forma de saber cuánto mover al líder.
                print(f'✗ La formación {shape} no cabe con el líder en '
                      f'({lx:.0f},{ly:.0f}). Arena {maxX:.0f}x{maxY:.0f}mm, '
                      f'margen {inset:.0f}mm:')
                for idx, sx, sy, exceso in fuera:
                    print(f'    slot {idx} caería en ({sx:.0f},{sy:.0f}) — '
                          f'se pasa {exceso:.0f}mm')
                print(f'  Movelo al menos {max(f[3] for f in fuera):.0f}mm '
                      f'hacia el centro.')
                return

        pa = rad + math.pi / 2 + math.radians(axis)
        px, py = math.cos(pa), math.sin(pa)   # eje efectivo de la fila

        # Los invisibles van al FINAL, no al medio. Antes devolvían 0.0, que es
        # una coordenada lateral perfectamente válida: un robot que la cámara no
        # veía se colaba entre los visibles y les corría el slot a todos. La
        # asignación anti-cruce se degradaba en silencio justo cuando más falta
        # hacía. Ahora los visibles se reparten sus slots correctamente y los
        # invisibles ocupan los que sobran, en orden de id (determinista).
        invisibles = [rid for rid in followers
                      if self.robots[rid].getPose()[0] == -1]

        def followerKey(rid):
            fx, fy, _ = self.robots[rid].getPose()
            if fx == -1:
                return float('inf')
            if shape == 'circulo':
                return math.atan2(fy - ly, fx - lx) % (2 * math.pi)
            return (fx - lx) * px + (fy - ly) * py

        def slotKey(idx):
            if shape == 'circulo':
                return 2 * math.pi * idx / max(1, n)
            return (1 if idx % 2 == 0 else -1) * (idx // 2 + 1)

        rankedFollowers = sorted(followers, key=followerKey)
        rankedSlots = sorted(range(n), key=slotKey)

        if invisibles:
            nombres = ', '.join(self.robots[r].name for r in invisibles)
            print(f'⚠ {nombres} sin marker visible: se les asigna el slot que '
                  f'sobra, no el más cercano. Pueden cruzarse con los demás.')

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


    def _completer(self, texto, estado):
        """Autocompletado por TAB, sensible a en qué parte del comando estás.

        readline parte la línea por sus delimitadores; acá se los quitamos todos
        (delims = '') para recibir la línea entera y decidir según tenga punto o
        no. Sin eso, el '.' y el '|' cortan el token y las opciones salen mal.
        """
        linea = readline.get_line_buffer().lstrip()
        if '.' not in linea:
            destinos = sorted(self.robots) + ['BROADCAST.', 'BASE.']
            opciones = [d if d.endswith('.') else d + '.'
                        for d in destinos if d.startswith(texto)]
        else:
            destino, resto = linea.split('.', 1)
            tabla = _BASE_CMDS if destino.upper() == 'BASE' else _ROBOT_CMDS
            if '|' in resto:
                # Ya está en los argumentos: no hay nada que completar, pero se
                # muestra la forma esperada como recordatorio.
                verbo = resto.split('|', 1)[0].upper()
                if verbo in tabla and estado == 0:
                    sys.stdout.write(f'\n  {verbo}|{tabla[verbo]}\n')
                    sys.stdout.flush()
                    readline.redisplay()
                return None
            prefijo = resto.upper()
            opciones = [destino + '.' + v for v in sorted(tabla)
                        if v.startswith(prefijo)]
        return opciones[estado] if estado < len(opciones) else None


    def _setupReadline(self):
        """Completado por TAB + historial que sobrevive entre corridas."""
        self._histPath = os.path.join(os.path.expanduser('~'),
                                      '.attabot_history')
        try:
            readline.read_history_file(self._histPath)
        except (OSError, PermissionError):
            pass                      # primera corrida: todavía no existe
        readline.set_history_length(1000)
        readline.set_completer(self._completer)
        readline.set_completer_delims('')     # la línea entera es el token
        readline.parse_and_bind('tab: complete')


    @runOnThread
    def inputInstruction(self):
        """
        Maneja la entrada de instrucciones desde la consola en tiempo real.

        Gramática: DESTINO.VERBO|arg|arg
            BASE.<verbo>       lo ejecuta la Base (BASE.HELP los lista)
            <id>.<verbo>       se envía a ese robot     — 1.MOVE|500
            BROADCAST.<verbo>  se envía a todos         — BROADCAST.DISPERSE|600
            BREAK              termina la corrida

        TAB autocompleta destinos y verbos. Las formas viejas verbo-primero
        (CALIBRATE.1, GOTO.1 x y) siguen aceptándose con un aviso.
        """
        self._setupReadline()
        while True:
            try:
                instructionRaw = input('> ').strip()
            except EOFError:
                break   # stdin cerrado (proceso lanzado sin consola) = BREAK
            if not instructionRaw:
                continue
            if instructionRaw == 'BREAK':
                break
            # 'HELP' suelto es lo que uno teclea cuando no se acuerda de nada,
            # y es justo el momento en que exigirle el prefijo es más inútil.
            if instructionRaw.upper().split('|')[0] == 'HELP':
                self._dispatchBase(instructionRaw, print)
                continue

            try:
                robotId, instruction = map(str.strip, instructionRaw.split('.', 1))
            except ValueError:
                print(f"Formato: DESTINO.VERBO|args  (ej. 1.MOVE|500). "
                      f"HELP lista todo.")
                continue

            self.dispatch(robotId, instruction)

        try:
            readline.write_history_file(self._histPath)
        except (OSError, PermissionError):
            pass
        self.threadInputAlive = False

    def dispatch(self, robotId, instruction, log=print):
        """Ejecuta un 'robotId.instrucción' venga de donde venga.

        La consola y la GUI comparten este método a propósito. Antes cada una
        tenía su propia cadena de if/elif sobre el mismo formato, y la de la GUI
        ya se había quedado atrás: entendía BROADCAST, CONGREGATION, GOTO y
        STATUS, pero no FORMATION, CALIBRATE ni OCCLUDE. Nadie lo notaba porque
        quien usa la GUI termina tecleando el comando crudo.

        `log` es lo único que cambia entre las dos caras: la consola imprime y la
        GUI emite una señal hacia su panel de mensajes.
        """
        robotId, instruction = self._normalizeCommand(robotId, instruction, log)

        if robotId == 'BASE':
            self._dispatchBase(instruction, log)
        elif robotId == 'BROADCAST':
            self.warnIfOutsideFov(instruction)
            self._warnUnknownVerb(instruction, log)
            self.sendInstructionBroadcast([instruction])
        elif robotId in self.robots:
            self.warnIfOutsideFov(instruction)
            self._warnUnknownVerb(instruction, log)
            self.sendInstruction(self.robots[robotId].IP, [instruction], True)
        else:
            log(f"Destino '{robotId}' desconocido. Usá el id de un robot "
                f"({', '.join(sorted(self.robots))}), BROADCAST o BASE. "
                f"Probá BASE.HELP")


    def _warnUnknownVerb(self, instruction, log):
        """Avisa antes de mandar un verbo que el firmware no va a reconocer.

        El firmware descarta en SILENCIO lo que no entiende: no contesta nada, y
        el robot simplemente no hace nada. Eso ya costó tiempo con un
        'MVE1.MOVE1.MOVE|500' del 05-08 que parecía un robot colgado. Con GOTO,
        POSITIONGT y BUG2 recién retirados, teclearlos por costumbre es
        probable, así que conviene decirlo en vez de dejar el silencio.

        Solo avisa: igual se envía, porque la tabla podría quedar corta frente a
        un firmware más nuevo y bloquear no seria peor que el silencio.
        """
        verbo = instruction.split('|')[0].strip().upper()
        if verbo and verbo not in _ROBOT_CMDS:
            log(f"⚠ '{verbo}' no es un comando del firmware — se envía igual, "
                f"pero el robot lo va a descartar sin avisar. BASE.HELP los lista.")


    def _normalizeCommand(self, target, instruction, log):
        """Traduce las formas viejas verbo-primero a la gramática DESTINO.VERBO.

        'CALIBRATE.1' y 'GOTO.1 1200 850' ponían el VERBO donde ahora va el
        DESTINO. Se aceptan igual para no romper la memoria muscular a mitad de
        sesión, pero avisan una vez por verbo y traducen a la forma canónica.
        """
        if target in self.robots or target not in _LEGACY_VERBS:
            return target, instruction

        args = '' if target == 'STATUS' else instruction.strip()
        canonico = target + ('|' + '|'.join(args.split()) if args else '')
        if target not in self._legacyWarned:
            self._legacyWarned.add(target)
            log(f"⚠ '{target}.{instruction}' es la forma vieja — ahora se "
                f"escribe 'BASE.{canonico}'. Sigue andando por ahora.")
        return 'BASE', canonico


    def _dispatchBase(self, instruction, log):
        """Comandos que ejecuta la Base (no viajan por UDP tal cual)."""
        parts = [p.strip() for p in instruction.split('|')]
        verbo, args = parts[0].upper(), [p for p in parts[1:] if p != '']

        def formato():
            log(f'Formato: BASE.{verbo}|{_BASE_CMDS.get(verbo, "")}')

        if verbo == 'HELP':
            self._printHelp(args[0].upper() if args else None, log)

        elif verbo == 'STATUS':
            log(f'Detecciones ArUco activas: '
                f'{list(self.currentArucoDetections.keys())}')
            for rid, robot in self.robots.items():
                x, y, angle = robot.getPose()
                if x != -1:
                    log(f'  Robot {rid} ({robot.name}): '
                        f'x={x:.1f}, y={y:.1f}, angle={angle:.1f}°')
                else:
                    log(f'  Robot {rid} ({robot.name}): no visible')
            if self.congregationActive:
                log(f'Congregación activa. Líder: {self.leaderID}')
                log(f'Completa: {self.isCongregationComplete()}')

        elif verbo == 'CALIBRATE':
            if len(args) != 1:
                return formato()
            self.startCalibration(args[0])

        elif verbo == 'CONGREGATION':
            if not args:
                return formato()
            try:
                if len(args) > 1:
                    self.startCongregation(args[0], float(args[1]))
                else:
                    self.startCongregation(args[0])
            except ValueError:
                formato()

        elif verbo == 'FORMATION':
            # startFormation sigue parseando por espacios; se traduce acá para
            # que el vocabulario del operador sea uniforme con '|'.
            if len(args) < 2:
                return formato()
            self.startFormation(' '.join(args))

        elif verbo == 'OCCLUDE':
            if not self.simMode:
                return log('OCCLUDE solo existe en modo --sim')
            if not args:
                return formato()
            self.simVision.sendControl(f'OCCLUDE.{args[0]}')
            log(f'Cámara sim ocluida por {args[0]}s')

        else:
            log(f"BASE no conoce '{verbo}'. Probá BASE.HELP")


    def _printHelp(self, verbo, log):
        """Ayuda desde el mismo vocabulario que alimenta el autocompletado."""
        if verbo:
            if verbo in _BASE_CMDS:
                log(f'BASE.{verbo}|{_BASE_CMDS[verbo]}')
            elif verbo in _ROBOT_CMDS:
                log(f'<id>.{verbo}|{_ROBOT_CMDS[verbo]}      '
                    f'(o BROADCAST.{verbo}|...)')
            else:
                log(f"No conozco '{verbo}'.")
            return

        log('Gramática:  DESTINO.VERBO|arg|arg')
        log('  BASE.<verbo>       lo ejecuta la Base')
        log('  <id>.<verbo>       se envía a ese robot')
        log('  BROADCAST.<verbo>  se envía a todos')
        log(f'\nDestinos: {", ".join(sorted(self.robots))}, BROADCAST, BASE')
        log('\nDe la Base:')
        for k, v in sorted(_BASE_CMDS.items()):
            log(f'  BASE.{k}' + (f'|{v}' if v else ''))
        log(f'\nA los robots ({len(_ROBOT_CMDS)}):')
        nombres = sorted(_ROBOT_CMDS)
        for i in range(0, len(nombres), 4):
            log('  ' + '  '.join(f'{n:<20}' for n in nombres[i:i + 4]).rstrip())
        log('\nDetalle de uno:  BASE.HELP|MOVE')


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
    # Las rutas del programa son relativas (configSystem.json y los directorios
    # Videos/PositionLogs/ConsoleLogs/Logs), asi que la base solo corria desde
    # Base/. Anclarlas al directorio del script deja lanzarla desde cualquier
    # lado: sin esto, correrla desde la raiz del repo no fallaba al escribir sino
    # que os.makedirs creaba los directorios ahi y desparramaba la corrida.
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

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