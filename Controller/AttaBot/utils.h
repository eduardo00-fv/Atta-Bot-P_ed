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
    BUG2_SEEK,
    BUG2_WALL_FOLLOW
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

struct Bug2State {
    // === Sub-estados internos ===
    enum SubState { IDLE, GOAL_SEEK, WALL_FOLLOW };
    SubState subState = IDLE;
    
    // === Puntos clave del algoritmo ===
    float startX = 0, startY = 0;     // Punto de inicio de la navegación
    float goalX = 0, goalY = 0;       // Punto objetivo
    float hitX = 0, hitY = 0;         // Punto donde chocó con el obstáculo
    float hitDistanceToGoal = 0;      // Distancia al objetivo desde hitPoint
    
    // === Línea M (recta Start -> Goal) ===
    // Ecuación: A*x + B*y + C = 0
    float lineA = 0, lineB = 0, lineC = 0;
    
    // === Configuración ===
    bool isActive = false;
    bool pendingInit = false;   // true entre BUG2 recibido y primer POSITION_RESPONSE
    float arrivalThreshold = 50;          // mm para considerar "llegó"
    float mLineThreshold = 150;           // mm de tolerancia para cruzar línea M
    int wallFollowDirection = 1;          // 1 = seguir pared derecha, -1 = izquierda
    bool directionAutoSet = false;        // true cuando la dirección ya fue determinada
    float seekSegmentDistance = 250;      // mm por segmento en GOAL_SEEK
    float wallFollowSegment = 200;        // mm por segmento de avance al seguir pared
    float wallFollowTurnAngle = 30;       // grados por giro al seguir pared

    // === Detección de loops y timeout ===
    unsigned long navigationStartTime = 0;
    const unsigned long maxNavigationTime = 180000;  // 3 minutos máximo
    float loopCheckX = 0, loopCheckY = 0;            // Posición al entrar a WALL_FOLLOW
    bool loopCheckSet = false;
    int wallFollowSteps = 0;                         // Pasos en WALL_FOLLOW
    const int maxWallFollowSteps = 100;              // Máximo antes de abortar
    const int minStepsBeforeLoopCheck = 8;           // Mín de pasos antes de verificar loop
    float loopThreshold = 100;                       // mm, si vuelve al hitPoint = loop
    int lostWallSteps = 0;                           // Pasos consecutivos sin detectar pared
    const int maxLostWallSteps = 3;                  // Máximo sin pared → volver a GOAL_SEEK
    
    // === Métodos ===
    
    void Start(float sx, float sy, float gx, float gy) {
        startX = sx;  startY = sy;
        goalX = gx;   goalY = gy;
        isActive = true;
        subState = GOAL_SEEK;
        navigationStartTime = millis();
        wallFollowSteps = 0;
        lostWallSteps = 0;
        loopCheckSet = false;
        
        // Calcular coeficientes de la Línea M: Ax + By + C = 0
        lineA = goalY - startY;
        lineB = startX - goalX;
        lineC = goalX * startY - startX * goalY;
        
        // Normalizar para que la distancia sea en unidades reales
        float norm = sqrt(lineA * lineA + lineB * lineB);
        if (norm > 0.001) {
            lineA /= norm;
            lineB /= norm;
            lineC /= norm;
        }
    }
    
    void Reset() {
        isActive = false;
        pendingInit = false;
        subState = IDLE;
        startX = 0; startY = 0;
        goalX = 0;  goalY = 0;
        hitX = 0;   hitY = 0;
        hitDistanceToGoal = 0;
        lineA = 0; lineB = 0; lineC = 0;
        wallFollowSteps = 0;
        lostWallSteps = 0;
        loopCheckSet = false;
        navigationStartTime = 0;
        wallFollowDirection = 1;
        directionAutoSet = false;
    }
    
    void RecordHitPoint(float x, float y) {
        hitX = x;
        hitY = y;
        hitDistanceToGoal = CalculateDistance(x, y, goalX, goalY);
        subState = WALL_FOLLOW;
        wallFollowSteps = 0;
        lostWallSteps = 0;
        loopCheckX = x;
        loopCheckY = y;
        loopCheckSet = true;
        directionAutoSet = false;
    }
    
    // Distancia de un punto a la línea M
    float DistanceToMLine(float x, float y) {
        return abs(lineA * x + lineB * y + lineC);
    }
    
    // ¿El robot está sobre la Línea M?
    bool IsOnMLine(float x, float y) {
        return DistanceToMLine(x, y) < mLineThreshold;
    }
    
    // ¿El robot está más cerca del objetivo que cuando chocó?
    bool IsCloserThanHitPoint(float x, float y) {
        float currentDist = CalculateDistance(x, y, goalX, goalY);
        return currentDist < (hitDistanceToGoal - arrivalThreshold * 0.5);
    }
    
    // Condición Bug 2 para dejar de seguir pared.
    // La proyección sobre la línea M reemplaza al guard de minStepsBeforeLoopCheck:
    // el robot debe estar 100mm+ adelante del hitPoint en la dirección Start→Goal.
    // Esto permite salir antes en obstáculos pequeños (<8 pasos) sin salir
    // prematuramente en el hitPoint mismo (donde currProj ≈ hitProj).
    bool ShouldLeaveWall(float x, float y) {
        if (!IsOnMLine(x, y)) return false;
        if (!IsCloserThanHitPoint(x, y)) return false;

        float dx = goalX - startX;
        float dy = goalY - startY;
        float len = sqrt(dx * dx + dy * dy);
        if (len < 1.0f) return false;
        dx /= len;
        dy /= len;
        float hitProj  = (hitX - startX) * dx + (hitY - startY) * dy;
        float currProj = (x   - startX) * dx + (y   - startY) * dy;
        return currProj > hitProj + 100.0f;
    }
    
    bool HasReachedGoal(float x, float y) {
        return CalculateDistance(x, y, goalX, goalY) < arrivalThreshold;
    }
    
    bool HasTimedOut() {
        return (millis() - navigationStartTime) > maxNavigationTime;
    }
    
    // Detectar si el robot dio una vuelta completa al obstáculo
    bool HasCompletedLoop(float x, float y) {
        if (!loopCheckSet || wallFollowSteps < minStepsBeforeLoopCheck) return false;
        return CalculateDistance(x, y, loopCheckX, loopCheckY) < loopThreshold;
    }
    
    bool HasExceededMaxSteps() {
        return wallFollowSteps >= maxWallFollowSteps;
    }
};

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
    const unsigned long requestTimeout = 5000;
    int followerIndex = 0;    // slot asignado por la Base (0-based)
    int totalFollowers = 1;   // total de seguidores en la congregación
    float parkingDist = 300;  // mm del líder al slot — NAV_CONFIG|PARKING_DIST
    float slotX = 0, slotY = 0;  // slot final (etapa 2 de la aproximación)
    bool stagingDone = false;    // true al alcanzar el waypoint de aproximación
    float slotAngle = 0;         // rad, latcheado al primer LEADER_POSITION
    bool slotAngleSet = false;   // evita que el slot salte de lado en camino

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
        slotAngleSet = false;
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
    
    void Reset() {
        leftPulseCount = 0;
        rightPulseCount = 0;
        pastLeftPulseCount = 0;
        pastRightPulseCount = 0;
        currentLeftSpeed = 0.0;
        currentRightSpeed = 0.0;
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
 * Port 1:1 de sim/ekf_sim.py (validado en sim 2026-07-04: 36mm de error medio
 * con 25% de oclusión vs 53mm de odometría pura; termina GT a ciegas).
 *
 * predict(d, dθ): propaga con odometría — d en mm de encoders, dθ en grados
 *   del gyro (ya escalado con yawScale). Llamar cada tick de lectura de IMU.
 * updateAruco(x, y, θ): corrige con la pose ArUco de POSITION_RESPONSE.
 *   La primera llamada inicializa el filtro. Devuelve la innovación de
 *   posición (mm) — qué tan lejos venía la predicción de la medición.
 *
 * θ interno en radianes; grados solo en las fronteras.
 ***************************************************************************************/
static inline void Mat3Mult(const float A[3][3], const float B[3][3], float R[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
}

static inline void Mat3Inverse(const float A[3][3], float R[3][3]) {
    float a = A[0][0], b = A[0][1], c = A[0][2];
    float d = A[1][0], e = A[1][1], f = A[1][2];
    float g = A[2][0], h = A[2][1], i = A[2][2];
    float det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    R[0][0] = (e * i - f * h) / det; R[0][1] = (c * h - b * i) / det; R[0][2] = (b * f - c * e) / det;
    R[1][0] = (f * g - d * i) / det; R[1][1] = (a * i - c * g) / det; R[1][2] = (c * d - a * f) / det;
    R[2][0] = (d * h - e * g) / det; R[2][1] = (b * g - a * h) / det; R[2][2] = (a * e - b * d) / det;
}

struct EKFState {
    bool initialized = false;
    float x = 0, y = 0;   // mm
    float th = 0;         // rad
    float P[3][3] = {{0}};

    // Ruido — mismos valores que sim/ekf_sim.py; calibrar en lab (Bloque A)
    static constexpr float R_POS_SIGMA   = 30.0f;   // mm — jitter ArUco
    static constexpr float R_ANG_SIGMA   = 2.0f;    // grados — jitter ArUco
    static constexpr float Q_DIST_FRAC   = 0.02f;   // fracción de d por tick
    static constexpr float Q_DIST_FLOOR  = 0.1f;    // mm por tick
    static constexpr float Q_ANG_DRIFT   = 0.05f;   // grados por tick
    static constexpr float Q_ANG_SCALE   = 0.005f;  // fracción de |dθ|

    static float WrapRad(float a) {
        while (a >  PI) a -= 2.0f * PI;
        while (a < -PI) a += 2.0f * PI;
        return a;
    }

    void Init(float px, float py, float angleDeg) {
        x = px;  y = py;  th = WrapRad(angleDeg * DEG_TO_RAD);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) P[i][j] = 0;
        P[0][0] = P[1][1] = R_POS_SIGMA * R_POS_SIGMA;
        P[2][2] = (R_ANG_SIGMA * DEG_TO_RAD) * (R_ANG_SIGMA * DEG_TO_RAD);
        initialized = true;
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
        if (!initialized) {
            Init(zx, zy, zthDeg);
            return 0.0f;
        }
        float nu[3] = {zx - x, zy - y, WrapRad(zthDeg * DEG_TO_RAD - th)};
        float S[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) S[i][j] = P[i][j];
        S[0][0] += R_POS_SIGMA * R_POS_SIGMA;
        S[1][1] += R_POS_SIGMA * R_POS_SIGMA;
        S[2][2] += (R_ANG_SIGMA * DEG_TO_RAD) * (R_ANG_SIGMA * DEG_TO_RAD);

        float Sinv[3][3], K[3][3];
        Mat3Inverse(S, Sinv);
        Mat3Mult(P, Sinv, K);   // H = I → K = P·S⁻¹

        x  += K[0][0] * nu[0] + K[0][1] * nu[1] + K[0][2] * nu[2];
        y  += K[1][0] * nu[0] + K[1][1] * nu[1] + K[1][2] * nu[2];
        th  = WrapRad(th + K[2][0] * nu[0] + K[2][1] * nu[1] + K[2][2] * nu[2]);

        float IK[3][3], newP[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                IK[i][j] = (i == j ? 1.0f : 0.0f) - K[i][j];
        Mat3Mult(IK, P, newP);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) P[i][j] = newP[i][j];

        return sqrtf(nu[0] * nu[0] + nu[1] * nu[1]);
    }

    float AngleDeg() const {
        float deg = th * RAD_TO_DEG;
        while (deg < 0) deg += 360.0f;
        return deg;
    }
};

#endif // UTILS_H