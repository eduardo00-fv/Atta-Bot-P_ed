/***************************************************************************************
 * utils.h - VERSIÓN COMPLETA REFACTORIZADA
 * 
 * Este archivo contiene TODAS las estructuras de datos y clases auxiliares
 * que no dependen directamente de hardware (motores, sensores, WiFi)
 * 
 * Proyecto: AttaBot - Sistema de Robot Enjambre
 * Autor: 
 * Fecha: 2025
 ***************************************************************************************/

#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <deque>
#include <map>
#include <array>

// ============================================================================
// ESTRUCTURAS DE DATOS BÁSICAS
// ============================================================================

/***************************************************************************************
 * Estructura que representa la pose de un robot en un espacio bidimensional.
 ***************************************************************************************/
struct pose {
  float x, y, angle;

  pose(float x, float y, float angle) : x(x), y(y), angle(angle) {}
};


/***************************************************************************************
 * Estructura que representa los parámetros del controlador PID.
 ***************************************************************************************/
struct pidConstants {
  float kp, ki, kd;

  pidConstants(float p, float i, float d) : kp(p), ki(i), kd(d) {}
};


/***************************************************************************************
 * Estructura que representa un filtro de Kalman.
 ***************************************************************************************/
struct kalmanFilter {
  float R; // Valor R filtro de Kalman
  float H; // Valor de ganancia filtro de Kalman
  float Q; // Valor Q filtro de Kalman
  float P; // Valor de ajuste de Kalman
  float K; // Valor ganancia K de Kalman
  float predictedValue; // Valor predicho

  kalmanFilter(float r, float h, float q)
      : R(r), H(h), Q(q), P(0), K(0), predictedValue(0) {}

  void Reset() {
    P = 0;
    K = 0;
    predictedValue = 0;
  }

  float Calculate(float val) {
    K = (P * H) / (H * P * H + R);
    predictedValue = predictedValue + K * (val - H * predictedValue);
    P = (1 - K * H) * P + Q;
    return predictedValue;
  }
};


/***************************************************************************************
 * Estructura para un controlador PID con filtro de Kalman.
 ***************************************************************************************/
struct pidController {
  kalmanFilter kf;
  pidConstants pidConst;
  const float samplingTime;
  const float minWorkCycleLimit;
  const float maxWorkCycleLimit;
  float offsetSumError;
  float sumError;
  float previousError;
  float error;
  float differentialError;

  pidController(kalmanFilter k, pidConstants p, float tiempo, float limiteMin, float limiteMax)
      : kf(k), pidConst(p), sumError(0), previousError(0), error(0), offsetSumError(0),
        samplingTime(tiempo), minWorkCycleLimit(limiteMin), maxWorkCycleLimit(limiteMax) {}

  void Reset() {
    sumError = 0;
    previousError = 0;
    kf.Reset();
  }

  int Calculate(const float reference, const float currentValue) {
    error = reference - currentValue;
    sumError += error * samplingTime;
    sumError = constrain(sumError, -maxWorkCycleLimit, maxWorkCycleLimit);
    float pidPwm = (pidConst.kp * error) + (pidConst.ki * sumError);
    differentialError = (abs(pidPwm) < minWorkCycleLimit) ? (error - previousError) / samplingTime : 0;
    pidPwm += pidConst.kd * differentialError;
    pidPwm = kf.Calculate(pidPwm);

    // Anti-windup mejorado: si estamos en saturación, no acumular error
    float constrainedPwm = constrain(pidPwm, -maxWorkCycleLimit, maxWorkCycleLimit);
    if (abs(constrainedPwm) >= maxWorkCycleLimit && abs(pidPwm) > abs(constrainedPwm)) {
      // Estamos saturados y queremos ir más allá: revertir acumulación
      sumError -= error * samplingTime;
    }

    previousError = error;
    return static_cast<int>(constrainedPwm);
  }
};


/***************************************************************************************
 * Estructura que almacena los valores de bias para sensores IMU.
 *
 * Los valores se persisten mediante Preferences (namespace "attabot-config"),
 * por lo que no se necesitan campos de integridad (header/checksum).
 * La validez se determina verificando si la clave existe en Preferences.
 ***************************************************************************************/
struct biasStore {
  int32_t biasGyroX  = 0;
  int32_t biasGyroY  = 0;
  int32_t biasGyroZ  = 0;
  int32_t biasAccelX = 0;
  int32_t biasAccelY = 0;
  int32_t biasAccelZ = 0;
  int32_t biasCPassX = 0;
  int32_t biasCPassY = 0;
  int32_t biasCPassZ = 0;

  // Retorna true si al menos el bias de giroscopio X fue guardado anteriormente.
  bool IsValid() const {
    return biasGyroX != 0 || biasGyroY != 0 || biasGyroZ != 0;
  }

  void Clear() {
    biasGyroX  = 0; biasGyroY  = 0; biasGyroZ  = 0;
    biasAccelX = 0; biasAccelY = 0; biasAccelZ = 0;
    biasCPassX = 0; biasCPassY = 0; biasCPassZ = 0;
  }
};



// ============================================================================
// ENUMERACIONES
// ============================================================================

/***************************************************************************************
 * Enumeración: RobotState
 * Define todos los estados posibles de la máquina de estados del robot.
 ***************************************************************************************/
enum RobotState {
    WAIT = 0,
    READ_INSTRUCTION,
    MOVE,
    TURN,
    STOP,
    REVERSE,
    RANDOM_WALK,
    MESSAGE_BASE,
    IDENTIFY_OBSTACLE,
    ACTIVE_EVASION,
    REQUEST_POSITION,
    RESUME_AFTER_EVASION,
    SEARCH_APPROACH
};


/***************************************************************************************
 * Búsqueda semántica de objetos por color (SEARCH_OBJECT|<color>).
 *
 * El robot patrulla con RANDOM_WALK; al detectar un obstáculo con el sensor
 * central se aproxima lento hasta el alcance del APDS9960, lee el color RGBC
 * y decide: coincide → OBJECT_FOUND y se detiene; no coincide → evade con el
 * patrón de retroceso estándar y sigue patrullando.
 * Prototipo validado en Webots (AttaBot-Sim) el 2026-07-05.
 ***************************************************************************************/
struct SearchState {
    bool active = false;
    char targetColor[12] = "";
    unsigned long approachStart = 0;

    void Reset() {
        active = false;
        targetColor[0] = '\0';
        approachStart = 0;
    }
};


// ============================================================================
// ESTRUCTURAS DE CONTROL Y NAVEGACIÓN
// ============================================================================

/***************************************************************************************
 * Calcula distancia euclidiana entre dos puntos
 ***************************************************************************************/
inline float CalculateDistance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

/***************************************************************************************
 * Estructura: InterruptionContext
 * 
 * Almacena el contexto de movimiento cuando el robot es interrumpido por obstáculos.
 * Permite reanudar el movimiento después de evasión.
 ***************************************************************************************/
struct InterruptionContext {
    bool wasInterrupted = false;
    RobotState previousState = WAIT;
    float remainingValue = 0;
    int leftPulsesBeforeStop = 0;
    int rightPulsesBeforeStop = 0;
    
    void Clear() {
        wasInterrupted = false;
        previousState = WAIT;
        remainingValue = 0;
        leftPulsesBeforeStop = 0;
        rightPulsesBeforeStop = 0;
    }
    
    bool HasRemainingMovement(float minimumDistance = 20) {
        return wasInterrupted && abs(remainingValue) > minimumDistance;
    }
};


/***************************************************************************************
 * Estructura: EvasionTracker
 * 
 * Rastrea las evasiones consecutivas para detectar situaciones de bloqueo
 * y activar comportamientos de escape más agresivos.
 ***************************************************************************************/
struct EvasionTracker {
    int consecutiveEvasions = 0;
    unsigned long lastEvasionTime = 0;
    const int maxConsecutiveEvasions = 3;
    const unsigned long evasionResetTime = 5000;  // 5 segundos
    bool forceRetreat = false;
    
    void RecordEvasion() {
        unsigned long now = millis();
        
        // Si han pasado más de 5 segundos, resetear contador
        if (now - lastEvasionTime > evasionResetTime) {
            consecutiveEvasions = 0;
        }
        
        consecutiveEvasions++;
        lastEvasionTime = now;
        
        // Activar retroceso forzado si superamos el límite
        if (consecutiveEvasions >= maxConsecutiveEvasions) {
            forceRetreat = true;
        }
    }
    
    void Reset() {
        consecutiveEvasions = 0;
        forceRetreat = false;
        lastEvasionTime = millis();
    }
    
    bool ShouldRetreat() {
        return forceRetreat;
    }
    
    bool IsInCriticalState() {
        return consecutiveEvasions >= maxConsecutiveEvasions - 1;
    }
};


/***************************************************************************************
 * Estructura: CongregationState
 * 
 * Maneja el estado de congregación del robot.
 * Consolida todas las variables relacionadas con el comportamiento de congregación.
 ***************************************************************************************/
struct CongregationState {
    String leaderID = "-1";
    bool isLeader = false;
    bool positionReceived = false;
    bool hasGlobalTarget = false;
    float globalTargetX = 0;
    float globalTargetY = 0;
    unsigned long lastRequestTime = 0;
    bool waitingForResponse = false;
    // La respuesta de la base llega en ~30ms o no llega: es un UDP en LAN contra
    // una detección del frame actual. Los 5000ms que había acá castigaban con 5s
    // de inmovilidad cada titileo del ArUco (la base no respondía si el marcador
    // estaba apagado en ese frame). Con el reintento corto del lado de la base
    // este timeout ya casi no se alcanza; queda en 1500ms para que, cuando se
    // alcance de verdad, el robot reintente rápido en vez de quedarse plantado.
    const unsigned long requestTimeout = 1500;
    int followerIndex = 0;    // slot asignado por la Base (0-based)
    int totalFollowers = 1;   // total de seguidores en la congregación
    float parkingDist = 300;  // mm del líder al slot — NAV_CONFIG|PARKING_DIST
    float slotX = 0, slotY = 0;  // slot final (etapa 2 de la aproximación)
    bool stagingDone = false;    // true al alcanzar el waypoint de aproximación
    float slotAngle = 0;         // rad, latcheado al primer LEADER_POSITION
    float slotRadius = 300;      // mm — el anillo puede CRECER si el líder quedó
                                 // contra una pared (SafeRingSlotAngle)
    bool slotAngleSet = false;   // evita que el slot salte de lado en camino
    // El slot de n==1 se deriva de la pose PROPIA (bearing líder→robot), así que
    // no puede latcharse hasta tener una pose fresca de cámara. El 2026-07-27 se
    // latcheaba con el primer LEADER_POSITION, 0.4s ANTES del primer
    // POSITION_RESPONSE: quedaba anclado a una lectura vieja y el robot salía
    // hacia el lado opuesto, sin recalcular nunca.
    bool poseFresh = false;      // hay pose de cámara posterior al CONGREGATION
    // Formación (FORMATION): "" = congregación clásica (círculo). "linea"/"cuna"
    // usan slot perpendicular al heading del líder; "circulo" = igual que la
    // congregación. formationAxis rota el eje de la fila (fallback de la Base
    // cuando la fila no cabe en la arena).
    String formationShape = "";
    float  formationAxis  = 0;

    void Reset() {
        leaderID = "-1";
        isLeader = false;
        positionReceived = false;
        hasGlobalTarget = false;
        globalTargetX = 0;
        globalTargetY = 0;
        lastRequestTime = 0;
        waitingForResponse = false;
        followerIndex = 0;
        totalFollowers = 1;
        slotX = 0;
        slotY = 0;
        stagingDone = false;
        slotAngle = 0;
        slotRadius = 300;
        slotAngleSet = false;
        poseFresh = false;
        formationShape = "";
        formationAxis  = 0;
    }

    bool IsActive() {
        return leaderID != "-1";
    }
    
    bool HasTimedOut() {
        return waitingForResponse && (millis() - lastRequestTime > requestTimeout);
    }
    
    void StartRequest() {
        lastRequestTime = millis();
        waitingForResponse = true;
    }
    
    void CompleteRequest() {
        waitingForResponse = false;
        lastRequestTime = 0;
    }
};


/***************************************************************************************
 * Estructura: DisperseState
 * -------------------------------------------------------------------------------------
 * Dispersión de enjambre (comando DISPERSE|mm): cada robot se repele de sus
 * vecinos (recibidos por NEIGHBOR_POSITIONS, 1 Hz desde la Base) hasta tener al
 * más cercano a >= target. Port 1:1 del controller de sim validado en Webots.
 *   - target = 0  → inactivo.
 *   - Turno secuencial por id (solo salta el menor id entre los muy-cercanos) +
 *     histéresis 80mm + timeout anti-deadlock: evita la tormenta de evasiones IR
 *     y el deadlock por jitter en la frontera vistos en los E2E de sim.
 *   - dminHist: mediana de 3 lecturas para que el jitter no dispare/cancele la
 *     confirmación de settle.
 * ⚠ Constantes de arena (ARENA_*) = FOV útil del lab (2.4 x 1.55 m). Ajustar si
 *   cambia el montaje de la cámara.
 ***************************************************************************************/
struct DisperseState {
    // 11 = enjambre de hasta 12 robots. Con el valor viejo (8) MEET topaba en 9
    // y el experimento de 10 robots se rompía en silencio: cada robot conservaba
    // los 8 vecinos que llegaron primero, o sea un insumo DISTINTO por robot, y
    // el reparto de slots dejaba de coincidir entre ellos (que es justo lo que
    // lo hace funcionar sin negociar). Cuesta 3 String + 6 float de RAM.
    static const int MAX_NEIGHBORS = 11;
    float target  = 0;           // mm de separación objetivo (0 = inactivo)
    bool  settled = false;
    int   blocked = 0;           // rondas esperando a un id menor (anti-deadlock)
    float dminHist[3] = {0, 0, 0};
    int   dminIdx = 0;           // posición de escritura (ring buffer)
    int   dminCount = 0;         // lecturas válidas (tope 3)
    // Vecinos del último NEIGHBOR_POSITIONS (excluye al propio robot)
    String nId[MAX_NEIGHBORS];
    float  nX[MAX_NEIGHBORS];
    float  nY[MAX_NEIGHBORS];
    int    nCount = 0;
    // Pose PROPIA tal como venía en ese mismo mensaje. La manda la Base junto a
    // las demás; se guarda aparte porque MEET reparte los slots con el greedy de
    // cercanía y todos los robots tienen que correrlo sobre EL MISMO barrido: si
    // cada uno usara su robotPose para sí mismo (más fresca) y la del mensaje
    // para los demás, dos podrían elegir el mismo slot.
    float  selfX = 0, selfY = 0;
    bool   selfSeen = false;

    bool IsActive() { return target > 0; }

    void Reset() {
        target = 0;
        settled = false;
        blocked = 0;
        dminIdx = 0;
        dminCount = 0;
        nCount = 0;
        selfSeen = false;
    }

    // Mediana móvil de las últimas 3 dmin — espejo de sorted(hist)[len//2] del
    // sim (len 1→valor, 2→el mayor, 3→la mediana). Filtra outliers de medición.
    float SmoothDmin(float dmin) {
        dminHist[dminIdx] = dmin;
        dminIdx = (dminIdx + 1) % 3;
        if (dminCount < 3) dminCount++;
        float v[3] = { dminHist[0], dminHist[1], dminHist[2] };
        for (int i = 1; i < dminCount; i++) {   // insertion sort de los válidos
            float key = v[i];
            int j = i - 1;
            while (j >= 0 && v[j] > key) { v[j + 1] = v[j]; j--; }
            v[j + 1] = key;
        }
        return v[dminCount / 2];
    }
};


/***************************************************************************************
 * Estructura: ObstacleState
 * 
 * Consolida el estado de todos los sensores de obstáculos.
 ***************************************************************************************/
struct ObstacleState {
    bool leftObstacle = false;
    bool centralObstacle = false;
    bool rightObstacle = false;
    bool robotDetected = false;
    String fromRobotID = "";
    int obstacleSensors = 0;  // Bitmap: [left][central][right]
    
    void Clear() {
        leftObstacle = false;
        centralObstacle = false;
        rightObstacle = false;
        robotDetected = false;
        fromRobotID = "";
        obstacleSensors = 0;
    }
    
    bool HasAnyObstacle() {
        return leftObstacle || centralObstacle || rightObstacle;
    }
    
    void UpdateBitmap() {
        obstacleSensors = (leftObstacle << 2) | (centralObstacle << 1) | rightObstacle;
    }
    
    bool IsFrontalObstacle() {
        return centralObstacle || (leftObstacle && rightObstacle);
    }
    
    String GetObstaclePattern() {
        if (obstacleSensors == 0b100) return "LEFT";
        if (obstacleSensors == 0b010) return "CENTER";
        if (obstacleSensors == 0b001) return "RIGHT";
        if (obstacleSensors == 0b110) return "LEFT+CENTER";
        if (obstacleSensors == 0b011) return "CENTER+RIGHT";
        if (obstacleSensors == 0b111) return "ALL";
        return "NONE";
    }
};


/***************************************************************************************
 * Estructura: MovementMetrics
 * 
 * Agrupa todas las métricas relacionadas con el movimiento del robot.
 ***************************************************************************************/
struct MovementMetrics {
    volatile int leftPulseCount = 0;
    volatile int rightPulseCount = 0;
    int pastLeftPulseCount = 0;
    int pastRightPulseCount = 0;
    float currentLeftSpeed = 0.0;
    float currentRightSpeed = 0.0;
    unsigned long previousMillis = 0;
    unsigned long steadyStatePreviousMillis = 0;
    // Sube en cada Reset(). Quien integre los contadores (EkfTick) necesita
    // distinguir "los pusieron en cero" de "el robot retrocedió": las dos cosas
    // hacen bajar el conteo, y confundirlas rompía la odometría del EKF.
    unsigned long resetEpoch = 0;

    void Reset() {
        leftPulseCount = 0;
        rightPulseCount = 0;
        pastLeftPulseCount = 0;
        pastRightPulseCount = 0;
        currentLeftSpeed = 0.0;
        currentRightSpeed = 0.0;
        resetEpoch++;
    }
    
    float GetAverageSpeed() {
        return (currentLeftSpeed + currentRightSpeed) / 2.0;
    }
    
    float GetAverageDistance(float mmPerPulse) {
        return ((pastLeftPulseCount + pastRightPulseCount) / 2.0) * mmPerPulse;
    }
    
    bool IsStationary() {
        return abs(currentLeftSpeed) < 0.1 && abs(currentRightSpeed) < 0.1;
    }
};


/***************************************************************************************
 * Estructura: LedController
 * 
 * Control no bloqueante de LEDs WS2812.
 * Permite animaciones sin usar delay().
 ***************************************************************************************/
struct LedController {
    enum State { OFF, SOLID, BLINKING };
    State currentState = OFF;
    uint8_t red = 0, green = 0, blue = 0, brightness = 0;
    unsigned long lastUpdate = 0, interval = 500;
    bool blinkState = false;
    
    void setSolid(uint8_t r, uint8_t g, uint8_t b, uint8_t bright = 255) {
        red = r;
        green = g;
        blue = b;
        brightness = bright;
        currentState = SOLID;
    }
    
    void setBlink(uint8_t r, uint8_t g, uint8_t b, uint8_t bright = 255, unsigned long intervalMs = 250) {
        red = r;
        green = g;
        blue = b;
        brightness = bright;
        interval = intervalMs;
        currentState = BLINKING;
        lastUpdate = 0;
    }
    
    void setOff() {
        currentState = OFF;
    }
    
    bool IsBlinking() {
        return currentState == BLINKING;
    }
    
    bool IsSolid() {
        return currentState == SOLID;
    }

    void update();
};


// ============================================================================
// FUNCIONES AUXILIARES INLINE
// ============================================================================

/***************************************************************************************
 * Normaliza ángulos al rango [-180, 180]
 ***************************************************************************************/
inline float NormalizeAngle(float angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}



/***************************************************************************************
 * Calcula ángulo hacia un objetivo
 ***************************************************************************************/
inline float CalculateAngleToTarget(float x1, float y1, float x2, float y2) {
    return atan2(y2 - y1, x2 - x1) * RAD_TO_DEG;
}

/***************************************************************************************
 * Verifica si un valor está en un rango
 ***************************************************************************************/
inline bool InRange(float value, float min, float max) {
    return value >= min && value <= max;
}

/***************************************************************************************
 * Estructura: ReactiveNav
 *
 * Navegación iterativa unificada con evasión reactiva de obstáculos.
 * Reemplaza Bug2State para GT y congregación: un solo camino de código
 * que maneja ambos comportamientos — el objetivo puede ser fijo (GT) o
 * actualizable (congregación con líder en movimiento).
 *
 * Algoritmo por paso:
 *   1. Calcular ángulo hacia objetivo
 *   2. Modificar ángulo si hay obstáculo en IR (capa reactiva)
 *   3. Encolar TURN + WAIT + MOVE + WAIT + REQUEST_POSITION
 *   4. Al recibir nueva posición, repetir
 *
 * La dirección de evasión se elige automáticamente hacia el lado del objetivo
 * para "doblar alrededor" del obstáculo en la dirección correcta.
 ***************************************************************************************/
struct ReactiveNav {
    bool  isActive    = false;
    bool  pendingInit = false;   // true entre comando GT/CONGREGATION y primer POSITION_RESPONSE
    float goalX = 0, goalY = 0;

    // Parámetros configurables
    float arrivalThreshold  = 50;    // mm para declarar llegada
    float segmentDistance   = 250;   // mm máximo por segmento normal
    float avoidSegment      = 120;   // mm por segmento cuando hay obstáculo
    float avoidFrontAngle   = 90.0f; // grados a girar si obstáculo frontal
    float avoidSideAngle    = 35.0f; // grados de bias si obstáculo lateral
    // Re-apuntar solo si el rumbo se desvía MÁS que esto. Tiene que quedar por
    // ENCIMA del ruido angular de la fuente de pose: el ángulo crudo de ArUco
    // mide σ≈3.8° (medido 2026-07-27 sobre un robot quieto), así que con el
    // valor viejo de 5° uno de cada cuatro ciclos giraba por puro ruido —y el
    // giro se calculaba con esa misma lectura ruidosa, metiendo error real.
    float realignThreshold  = 12.0f; // grados
    // Histéresis del goal: ignorar reubicaciones menores a esto. CONGREGATION
    // reescribe el goal a ~3Hz con la pose cruda del líder; sin banda muerta,
    // su jitter (rango 80mm) se traduce en re-apuntados constantes.
    float goalDeadband      = 40.0f; // mm

    unsigned long startTime = 0;
    const unsigned long maxNavTime = 180000; // 3 min timeout

    void Start(float gx, float gy) {
        goalX = gx;  goalY = gy;
        isActive    = true;
        pendingInit = false;
        startTime   = millis();
    }

    void Reset() {
        isActive    = false;
        pendingInit = false;
        goalX = 0;  goalY = 0;
    }

    bool HasReached(float x, float y) const {
        return CalculateDistance(x, y, goalX, goalY) < arrivalThreshold;
    }

    bool HasTimedOut() const {
        return (millis() - startTime) > maxNavTime;
    }
};


/***************************************************************************************
 * EKF descentralizado — estado [x, y, θ] en el marco de la cámara/ArUco.
 *
 * Es la pose de respaldo del robot: mientras la cámara conteste manda el ArUco,
 * y cuando deja de contestar la navegación sigue con esta estimación en vez de
 * quedarse parada esperando (ver StateRequestPosition).
 *
 * Predict(d, dθ): propaga con odometría — d en mm de encoders, dθ en grados
 *   del gyro (ya escalado con yawScale). Llamar cada tick de lectura de IMU.
 *   d viene firmado: negativo al retroceder.
 * UpdateAruco(x, y, θ): corrige con la pose ArUco de POSITION_RESPONSE.
 *   La primera llamada inicializa el filtro. Descarta mediciones que no pasan
 *   el gate de Mahalanobis y se re-ancla si las rechaza MAX_GATE_REJECTS veces
 *   seguidas. Devuelve la innovación de posición (mm) — qué tan lejos venía la
 *   predicción de la medición.
 * IsHealthy() / PosSigma(): hay que consultarlos antes de navegar a ciegas.
 *
 * θ interno en radianes; grados solo en las fronteras.
 ***************************************************************************************/
static inline void Mat3Mult(const float A[3][3], const float B[3][3], float R[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
}

// Inversa de 3x3 por cofactores. Devuelve false si la matriz es singular o el
// resultado no es finito, en vez de dividir por cero: sin este cortocircuito un
// determinante degenerado metia NaN en P y el filtro quedaba muerto para siempre
// (190 muestras EKF_POSE|nan de las corridas del 30-07).
static inline bool Mat3Inverse(const float A[3][3], float R[3][3]) {
    float a = A[0][0], b = A[0][1], c = A[0][2];
    float d = A[1][0], e = A[1][1], f = A[1][2];
    float g = A[2][0], h = A[2][1], i = A[2][2];
    float det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    if (!isfinite(det) || fabsf(det) < 1e-9f) return false;
    R[0][0] = (e * i - f * h) / det; R[0][1] = (c * h - b * i) / det; R[0][2] = (b * f - c * e) / det;
    R[1][0] = (f * g - d * i) / det; R[1][1] = (a * i - c * g) / det; R[1][2] = (c * d - a * f) / det;
    R[2][0] = (d * h - e * g) / det; R[2][1] = (b * g - a * h) / det; R[2][2] = (a * e - b * d) / det;
    for (int r = 0; r < 3; r++)
        for (int cc = 0; cc < 3; cc++)
            if (!isfinite(R[r][cc])) return false;
    return true;
}

struct EKFState {
    bool initialized = false;
    float x = 0, y = 0;   // mm
    float th = 0;         // rad
    float P[3][3] = {{0}};

    // R medido sobre los logs del 30-07 con el robot quieto (n=281 muestras):
    // el jitter del ArUco es 4.4mm RMS y 7.5mm p95, no los 30mm que se traían de
    // la sim. Con R inflado el filtro ignoraba la cámara — la corrección se comía
    // el 21% del error y no alcanzaba a compensar la deriva. Los outliers gruesos
    // NO se cubren inflando R: para eso está el gate de Mahalanobis de
    // UpdateAruco, que además sabe re-anclarse.
    // Q es provisional: la deriva de odometría medida en los logs viejos está
    // contaminada por el bug del REVERSE, así que hay que re-medirla con
    // `analyze_logs.py --ekf` sobre una sesión ya con el fix.
    static constexpr float R_POS_SIGMA   = 8.0f;    // mm — jitter ArUco medido
    static constexpr float R_ANG_SIGMA   = 4.0f;    // grados — jitter ArUco medido
    static constexpr float Q_DIST_FRAC   = 0.08f;   // fracción de d por tick
    static constexpr float Q_DIST_FLOOR  = 0.5f;    // mm por tick
    static constexpr float Q_ANG_DRIFT   = 0.15f;   // grados por tick
    static constexpr float Q_ANG_SCALE   = 0.02f;   // fracción de |dθ|

    // Gate de rechazo: una medición cuya innovación no cabe en la incertidumbre
    // declarada (distancia de Mahalanobis, 3 g.d.l.) se descarta como misread de
    // la cámara. Pero tras MAX_GATE_REJECTS seguidos el que está mal es el
    // filtro, no la cámara, y se re-ancla: sin esa salida el gate se convierte
    // en la trampa que ya conocemos de max_pose_jump, ignorando para siempre.
    static constexpr float GATE_CHI2      = 25.0f;
    static constexpr int   MAX_GATE_REJECTS = 3;
    int gateRejects = 0;

    static float WrapRad(float a) {
        while (a >  PI) a -= 2.0f * PI;
        while (a < -PI) a += 2.0f * PI;
        return a;
    }

    // Des-ancla el filtro: la próxima corrección ArUco lo re-inicializa desde
    // cero. Necesario porque Predict() sigue integrando odometría aunque no
    // haya correcciones (entre comandos, durante evasiones, con la nav parada),
    // y esa deriva se acumulaba de sesión en sesión: el 2026-07-27 se midió un
    // estado a 78m de la realidad, con y=-4289mm fuera de la arena.
    void Reset() {
        initialized = false;
        gateRejects = 0;
    }

    void Init(float px, float py, float angleDeg) {
        x = px;  y = py;  th = WrapRad(angleDeg * DEG_TO_RAD);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) P[i][j] = 0;
        P[0][0] = P[1][1] = R_POS_SIGMA * R_POS_SIGMA;
        P[2][2] = (R_ANG_SIGMA * DEG_TO_RAD) * (R_ANG_SIGMA * DEG_TO_RAD);
        initialized = true;
        gateRejects = 0;
    }

    void Predict(float d, float dthDeg) {
        if (!initialized) return;
        float dth = dthDeg * DEG_TO_RAD;
        float c = cosf(th), s = sinf(th);   // θ previo — también para F
        x += d * c;
        y += d * s;
        th = WrapRad(th + dth);

        float F[3][3] = {{1, 0, -d * s}, {0, 1, d * c}, {0, 0, 1}};
        float Ft[3][3] = {{1, 0, 0}, {0, 1, 0}, {-d * s, d * c, 1}};
        float FP[3][3], FPFt[3][3];
        Mat3Mult(F, P, FP);
        Mat3Mult(FP, Ft, FPFt);
        float sd  = Q_DIST_FRAC * fabsf(d) + Q_DIST_FLOOR;
        float sth = (Q_ANG_DRIFT + Q_ANG_SCALE * fabsf(dthDeg)) * DEG_TO_RAD;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) P[i][j] = FPFt[i][j];
        P[0][0] += sd * sd;
        P[1][1] += sd * sd;
        P[2][2] += sth * sth;
    }

    float UpdateAruco(float zx, float zy, float zthDeg) {
        if (!initialized || !IsHealthy()) {
            Init(zx, zy, zthDeg);
            return 0.0f;
        }
        float nu[3] = {zx - x, zy - y, WrapRad(zthDeg * DEG_TO_RAD - th)};
        float innov = sqrtf(nu[0] * nu[0] + nu[1] * nu[1]);
        float S[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) S[i][j] = P[i][j];
        S[0][0] += R_POS_SIGMA * R_POS_SIGMA;
        S[1][1] += R_POS_SIGMA * R_POS_SIGMA;
        S[2][2] += (R_ANG_SIGMA * DEG_TO_RAD) * (R_ANG_SIGMA * DEG_TO_RAD);

        float Sinv[3][3], K[3][3];
        if (!Mat3Inverse(S, Sinv)) {
            Init(zx, zy, zthDeg);
            return innov;
        }

        float md2 = 0.0f;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) md2 += nu[i] * Sinv[i][j] * nu[j];
        if (md2 > GATE_CHI2) {
            gateRejects++;
            if (gateRejects < MAX_GATE_REJECTS) return innov;
            Init(zx, zy, zthDeg);
            return innov;
        }
        gateRejects = 0;

        Mat3Mult(P, Sinv, K);   // H = I → K = P·S⁻¹

        x  += K[0][0] * nu[0] + K[0][1] * nu[1] + K[0][2] * nu[2];
        y  += K[1][0] * nu[0] + K[1][1] * nu[1] + K[1][2] * nu[2];
        th  = WrapRad(th + K[2][0] * nu[0] + K[2][1] * nu[1] + K[2][2] * nu[2]);

        float IK[3][3], newP[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                IK[i][j] = (i == j ? 1.0f : 0.0f) - K[i][j];
        Mat3Mult(IK, P, newP);
        // Simetrizar: (I-K)·P acumula asimetría en float de 32 bits y una P
        // asimétrica es la que termina dando determinantes degenerados.
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                P[i][j] = 0.5f * (newP[i][j] + newP[j][i]);

        if (!IsHealthy()) Init(zx, zy, zthDeg);
        return innov;
    }

    // El estado sirve para navegar: ni NaN ni infinitos, y con una covarianza
    // que no se disparó. Quien vaya a usar la pose a ciegas tiene que preguntar
    // esto antes — un EKF enfermo manda al robot a cualquier lado con confianza.
    bool IsHealthy() const {
        if (!isfinite(x) || !isfinite(y) || !isfinite(th)) return false;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                if (!isfinite(P[i][j])) return false;
        return P[0][0] >= 0.0f && P[1][1] >= 0.0f && P[2][2] >= 0.0f;
    }

    // Desviación estándar de la posición estimada, en mm. Es la medida de cuánto
    // vale la pena confiarle la navegación mientras la cámara no responde.
    float PosSigma() const {
        float v = P[0][0] + P[1][1];
        return (v > 0.0f && isfinite(v)) ? sqrtf(v) : 0.0f;
    }

    float AngleDeg() const {
        float deg = th * RAD_TO_DEG;
        while (deg < 0) deg += 360.0f;
        return deg;
    }
};

#endif // UTILS_H