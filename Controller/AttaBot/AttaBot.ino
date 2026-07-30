#include "utils.h"
// Versiones con las que se compila hoy (2026-07-29), sobre ESP32 Arduino core
// 3.3.11. Actualizarlas al subir una librería: cuando el comentario miente, un
// cambio de librería se confunde con una falla de hardware.
// Ya no está ESP32Servo — el servo frontal se retiró (2026-07-29, no se usaba)
// porque su PWM a 50Hz/10-bit se llevaba el mux de reloj del LEDC y dejaba a
// los motores sin PWM: el robot no giraba ni avanzaba. Si algún día vuelve un
// servo, leer primero Controller/Readme.md — necesita 16 bits de ancho de
// timer y ESP32Servo 3.2.1 no deja ponérselos (attach() pisa el default).
#include <Adafruit_APDS9960.h> // v1.3.1
#include <ArduinoOTA.h>
#include <FastLED.h>    // v3.10.5
#include <ICM_20948.h>  // v1.3.2 — con ICM_20948_USE_DMP activo en ICM_20948_C.h
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

// ============================================================================
// CONFIGURACIÓN DE DEBUG
// ============================================================================

// Descomentar solo para debug
#define DebugSerial
#ifdef DebugSerial
#define DebugSerialPrint(x) Serial.print(x)
#define DebugSerialPrintln(x) Serial.println(x)
#define DebugSerialPrintf(x, ...) Serial.printf(x, ##__VA_ARGS__)
#else
#define DebugSerialPrint(x)
#define DebugSerialPrintln(x)
#define DebugSerialPrintf(x, ...)
#endif

// ============================================================================
// DEFINICIÓN DE PINES
// ============================================================================

#define leftMotorForward 12
#define leftMotorBackward 14
#define rightMotorForward 13
#define rightMotorBackward 15

#define leftEncoderC1 32
#define leftEncoderC2 35
#define rightEncoderC1 23
#define rightEncoderC2 25

#define enableLeftInfraredSensor 5
#define leftInfraredSensor 33
// GPIO 26 libre: era el servo frontal, retirado el 2026-07-29.
#define enableRightInfraredSensor 18
#define rightInfraredSensor 27

#define batteryStatus 19
#define ledPin 2
#define AD0_VAL 1
#define NUM_LEDS 1

#define pwm_freq 1000
#define pwm_resolution 14

// ============================================================================
// CONSTANTES DEL SISTEMA
// ============================================================================

// WiFi
const char *ssid = "Atta-Bot";
const char *password = "attabot1234";
const unsigned int localPort = 6060;
char receivedPacket[255];

// Constantes del robot
float pulsesPerRev = 574;
const float wheelCircumference = PI * 44.5;
float millimetersPerPulse = wheelCircumference / pulsesPerRev;
float centerToWheelDistance = 41.5;  // configurable vía NAV_CONFIG|WHEEL_DIST
float yawScale = 1.0f;  // escala del gyro por robot (físico/IMU, calibrada con
                        // ArUco; ±2.4% medido) — NAV_CONFIG|YAW_SCALE|x|SAVE

// NO USAR radians() EN ESTE SKETCH — usar DegToRad/DegToArc.
//
// FastLED hace `#undef radians` (fl/stl/undef.h) y después `using fl::radians`
// (FastLED.h:226), así que radians() no es la macro de Arduino sino esto:
//     template<typename T> constexpr T radians(T deg) {
//         return deg * static_cast<T>(0.017453292519943295);
//     }
// La constante se castea al TIPO DEL ARGUMENTO. Con un ángulo entero,
// static_cast<int>(0.01745…) es 0, así que radians(180) devuelve 0 y el giro
// queda en un no-op silencioso. Con float anda bien, y de ahí el cuadro que
// costó semanas: el robot navegaba hacia el objetivo (nav usa floats) pero no
// esquivaba obstáculos (`int avoidanceAngle`), el RANDOM_WALK no giraba y
// TURN|grados no hacía nada en ningún robot. Llegó con FastLED 3.10.5
// (2026-07-27); antes la macro de Arduino promovía todo a double.
// Estos helpers usan la constante literal en float y toman float a propósito:
// fuerzan la promoción en el call site y no dependen ni de FastLED ni de
// DEG_TO_RAD. Diagnosticado el 2026-07-29, ver Controller/Readme.md.
inline float DegToRad(float degrees) {
  return degrees * 0.017453292519943295f;
}

// Grados de giro en el lugar → arco (mm) que recorre cada rueda. Única vía
// permitida para esa conversión, que antes vivía repetida en 9 lugares.
inline float DegToArc(float degrees) {
  return DegToRad(degrees) * centerToWheelDistance;
}

// Muestreo y velocidad
const unsigned int samplingTime = 10;
const float samplingTimeS = samplingTime * 0.001;
const unsigned int SteadyStateTime = 800;
const float distanceOffset = 1 * millimetersPerPulse;
const float baseSpeed = millimetersPerPulse / samplingTimeS;
const float maxSpeed = baseSpeed * 5;
const float minSpeed = 12;
const int speedReductionThreshold = 16;

// Control PID
const int maxPWMValue = (1 << pwm_resolution) - 1;
const int minPWMValue = maxPWMValue * 0.20;
pidConstants pidSpeed(110, 375, 2);
kalmanFilter kfPID(6.0, 1.0, 1.0);

// Sensores de obstáculos
const int observationPeriod = 28800;
const int observationTime = 1600;
const int numberOfCycles = observationPeriod / observationTime;
const int lateralCycle = random(numberOfCycles);
const int centralCycle =
    (lateralCycle + random(1, numberOfCycles)) % numberOfCycles;
const unsigned minObstacleTime = 1350;

// Detección de robots
const float robotDistanceMargin = 260;
const float maxRobotAngleMargin = 80;
const int obstacleWaitTime = 600;
const int reverseDistance = -40;

// Escape de deadlock (ACTIVE_EVASION tras N evasiones): giro de rodeo hacia el
// interior de la arena en vez del giro 180° ciego. Espeja escape_turn() de la
// réplica de sim (attabot_firmware.py). El lado se elige proyectando el tramo
// por cada lado y tomando el que queda más adentro (DISP_ARENA como límites).
const float escapeTurnDeg = 95.0f;   // ° del giro comprometido
const float escapeMoveMm  = 300.0f;  // mm del tramo de rodeo
// Arena del escenario en curso (marco cámara). NO es constante: la base la
// envía con NAV_CONFIG|ARENA|w|h al registrar cada robot, porque cambia por
// escenario (4 robots = 2400×1750, 10 robots = 3800×2800).
float arenaWidthMm  = 2400.0f;
float arenaHeightMm = 1750.0f;

// Debug
int debugUdp = 0;
int debugCounter = 0;
char direction = '+';
const char *debugMessage =
    "DEBUG: %d, ID: %s, Direccion: %c, val: Izq|Der, Encoder: %d|%d, Vel: "
    "%.2f|%.2f, Pwm: %d|%d, ErrorP: %.2f|%.2f, ErrorI: %.2f|%.2f, Dis: "
    "%.2f|%.2f, Tiempo: %d";

// Random Walk
const std::array<int, 7> possibleAngles = {30, 45, 60, 75, 90, 135, 180};
const std::array<int, 4> possibleAdvances = {200, 250, 300, 350};
enum possibleDirections { TURN_POS = 0, MOVE_FORWARD, TURN_NEG };

// Filtro de saltos bruscos en actualización de pose
const float max_pose_jump =
    500; // Máximo salto permitido en mm por actualización
const float max_angle_jump = 179; // Máximo salto permitido en grados
const int max_pose_jump_rejections =
    3; // Rechazos consecutivos antes de re-sincronizar con la cámara
int poseJumpRejections = 0;

// IMU
const float gravity = 9806.65;
const float conversionFactor = 8192.0;
float yaw;
float imuGravity;
bool imuAvailable = false;  // true solo si setupIMU() completó exitosamente

// LEDs
int maxBrightness = 140;

// Batería
volatile unsigned long lowBatteryTime = 0;
int minLowBatteryTime = 200;

// Contador de mensajes
int countMessages = 0;

// Sensor frontal (APDS9960)
bool frontSensorInitialized = false;
unsigned long lastFrontSensorAttempt = 0;
const unsigned long frontSensorRetryInterval =
    5000; // Reintentar cada 5 segundos
volatile bool lateralSensorsEnabled = false;

// ============================================================================
// VARIABLES GLOBALES REFACTORIZADAS (usando estructuras de utils.h)
// ============================================================================

InterruptionContext intContext;
EvasionTracker evasionTracker;
CongregationState congregation;
unsigned long lastLeaderCast = 0;  // throttle del broadcast LEADER_POSITION del líder

// Suavizado de la pose que DIFUNDE el líder. La cámara le mete σ≈10mm y σ≈3.8°
// (medido 2026-07-27 sobre un robot quieto) y ese ruido se propagaba tal cual al
// goal de cada seguidor. Promedio móvil corto; el ángulo se promedia por
// seno/coseno para no romperse en el wrap de 360°. Si el líder se mueve de
// verdad (salto > LEADER_SMOOTH_RESET) la ventana se reinicia, así el suavizado
// no introduce retardo cuando hace falta seguirlo.
const int   LEADER_SMOOTH_N     = 4;
const float LEADER_SMOOTH_RESET = 100.0f;  // mm
float leaderSmX[LEADER_SMOOTH_N], leaderSmY[LEADER_SMOOTH_N];
float leaderSmS[LEADER_SMOOTH_N], leaderSmC[LEADER_SMOOTH_N];
int   leaderSmCount = 0, leaderSmIdx = 0;

// Slot del anillo de congregación, seguro contra paredes y SIN colisiones
// entre slots. Determinista con datos que TODOS los seguidores comparten (pose
// del líder, n, arena vía NAV_CONFIG|ARENA): cada uno calcula el MISMO anillo
// y toma el ángulo de su índice — descentralizado sin negociación.
//
// Por qué no corregir solo el ángulo propio: la versión greedy rotaba cada
// slot invasor hacia el lado libre por separado y los ENCIMABA — medido
// 2026-07-27 con el líder a 311mm de la pared: 5 de 9 slots quedaron a
// 31-62mm entre sí y el enjambre nunca asentó. Acá los n slots se reparten
// parejos sobre el arco seguro más largo; si el arco no alcanza para n
// cuerpos (MIN_ARC c/u), el radio crece hasta que sí.
//
// n==1 (useBearing): se respeta el bearing líder→robot si cae en el arco
// (mínimo recorrido); si no, el extremo del arco más cercano.
// Escribe el radio efectivo en *outR (puede ser > nominal).
float SafeRingSlotAngle(float lx, float ly, int idx, int n, float nominalR,
                        float bearing, bool useBearing, float *outR) {
  const float MARGIN = 200.0f;    // holgura slot-pared (cuerpo 75 + margen)
  const float MIN_ARC = 250.0f;   // mm de arco por robot (mismo valor que Base)
  const float growth[6] = {1.0f, 1.2f, 1.4f, 1.7f, 2.0f, 2.5f};
  const int NS = 72;              // muestreo del círculo cada 5°
  const float STEP = 2.0f * PI / NS;
  float fallback = useBearing ? bearing : (2.0f * PI * idx) / max(1, n);
  *outR = nominalR;

  for (int gi = 0; gi < 6; gi++) {
    float R = nominalR * growth[gi];
    bool safe[NS];
    int nSafe = 0;
    for (int k = 0; k < NS; k++) {
      float sx = lx + R * cosf(k * STEP);
      float sy = ly + R * sinf(k * STEP);
      safe[k] = (sx >= MARGIN && sy >= MARGIN &&
                 sx <= arenaWidthMm - MARGIN && sy <= arenaHeightMm - MARGIN);
      if (safe[k]) nSafe++;
    }
    if (nSafe == NS) {            // círculo completo libre: abanico/bearing puro
      *outR = R;
      return fallback;
    }
    if (nSafe == 0) continue;
    // Arco contiguo seguro más largo (recorrido circular 2·NS)
    int bestStart = 0, bestLen = 0, curStart = -1, curLen = 0;
    for (int k = 0; k < 2 * NS; k++) {
      if (safe[k % NS]) {
        if (curLen == 0) curStart = k % NS;
        curLen++;
        if (curLen > bestLen && curLen <= NS) {
          bestLen = curLen;
          bestStart = curStart;
        }
      } else {
        curLen = 0;
      }
    }
    float arcLen = bestLen * STEP;
    if (R * arcLen >= n * MIN_ARC || gi == 5) {
      *outR = R;
      float a0 = bestStart * STEP;
      if (useBearing) {
        float rel = fmodf(bearing - a0 + 4.0f * PI, 2.0f * PI);
        if (rel <= arcLen) return bearing;
        return ((2.0f * PI - rel) < (rel - arcLen)) ? a0 : a0 + arcLen;
      }
      return a0 + (idx + 0.5f) * arcLen / max(1, n);
    }
  }
  return fallback;
}

// Agrega la pose actual del líder y devuelve la promediada por referencia.
void SmoothLeaderPose(float xIn, float yIn, float angIn,
                      float *xOut, float *yOut, float *angOut) {
  if (leaderSmCount > 0) {
    float ax = 0, ay = 0;
    for (int i = 0; i < leaderSmCount; i++) { ax += leaderSmX[i]; ay += leaderSmY[i]; }
    ax /= leaderSmCount;  ay /= leaderSmCount;
    if (CalculateDistance(xIn, yIn, ax, ay) > LEADER_SMOOTH_RESET) {
      leaderSmCount = 0;  leaderSmIdx = 0;   // el líder se movió: ventana nueva
    }
  }
  float rad = angIn * DEG_TO_RAD;
  leaderSmX[leaderSmIdx] = xIn;
  leaderSmY[leaderSmIdx] = yIn;
  leaderSmS[leaderSmIdx] = sin(rad);
  leaderSmC[leaderSmIdx] = cos(rad);
  leaderSmIdx = (leaderSmIdx + 1) % LEADER_SMOOTH_N;
  if (leaderSmCount < LEADER_SMOOTH_N) leaderSmCount++;

  float sx = 0, sy = 0, ss = 0, sc = 0;
  for (int i = 0; i < leaderSmCount; i++) {
    sx += leaderSmX[i];  sy += leaderSmY[i];
    ss += leaderSmS[i];  sc += leaderSmC[i];
  }
  *xOut   = sx / leaderSmCount;
  *yOut   = sy / leaderSmCount;
  *angOut = atan2(ss, sc) * RAD_TO_DEG;
}
DisperseState disperse;  // dispersión de enjambre (DISPERSE + NEIGHBOR_POSITIONS)
EKFState ekf;  // observador pasivo por ahora — la nav sigue usando robotPose
SearchState search;

// SEARCH_OBJECT: aproximación y clasificación de color
const int SEARCH_CREEP_PWM = 70;         // PWM de aproximación lenta
const uint8_t SEARCH_PROX_NEAR = 180;    // readProximity() ≥ esto = al alcance
const unsigned long SEARCH_APPROACH_TIMEOUT = 6000;  // ms
ObstacleState obstacles;
MovementMetrics movement;
LedController ledCtrl;

// IMU — control de frecuencia de lectura
unsigned long lastImuRead = 0;
const unsigned long imuReadInterval = 20;  // ms — 50Hz, por debajo del ODR del DMP (~112Hz)

// Variables de control de movimiento
unsigned long currentMillis = millis();
unsigned long previousMillisRW = 0;
int millisDifference;
int pastLeftEncoder = 0;
int pastRightEncoder = 0;

// Variables de sensores
unsigned long currentMicros = micros();
unsigned long previousMicros = micros();
bool isLateralCycleActive = false;
bool isCentralCycleActive = false;
// Máscaras de sensores IR — true = ignorar ese sensor (SENSOR_MASK|L/R/C|1)
bool maskLeftIR = false;
bool maskRightIR = false;
bool maskCentralIR = false;
// Umbral de proximidad del APDS9960 central (0..255, mayor = más cerca).
// Configurable en vivo con SENSOR_THRESHOLD|C|<n>[|SAVE] y persistido en NVS.
int centralIRThreshold = 2;
int cycleCounter = 0;
int microsDifference;
volatile unsigned long leftObsStartTime = 0;
volatile unsigned long rightObsStartTime = 0;
unsigned long centralObsStartTime = 0;
int centralDistance;

// Estado del robot
String robotID = "-1";
std::map<String, IPAddress> robots;
RobotState state = WAIT;
float instructionValue = 100;
bool movementReady = true;

// Navegación reactiva unificada (GT + congregación)
ReactiveNav nav;
// EKF_NAV|1 → la nav se controla con la pose del EKF en vez del ArUco crudo.
// Arranca apagado para poder A/B-testear contra el comportamiento conocido.
bool ekfNavEnabled = false;

// IMU-assisted TURN: giro cerrado en yaw — el arco restante se re-apunta con
// el IMU en cada ciclo, y al final se corrige si quedó residuo
bool  imuTurnActive   = false;
bool  imuTurnIsCorrection = false;  // el residuo se cierra SIN brake-lead (lead=0);
                                    // si no, en arcos chicos el coast reservado se
                                    // come la corrección entera (cmd -5.3° → real 0.8°)
int   imuTurnCorrCount = 0;         // correcciones hechas en este giro (tope: imuTurnMaxCorrections)
float imuTurnStartYaw = 0.0f;
float imuTurnPrevYaw  = 0.0f;    // última lectura para unwrap incremental
float imuTurnAccumDeg = 0.0f;    // giro acumulado medido por IMU (sin wrap, soporta >180°)
float imuTurnTargetDeg = 0.0f;   // ángulo objetivo con signo (+ = CCW, - = CW)
unsigned long imuTurnSettleUntil = 0;  // !=0: motores cortados, midiendo coast
const float imuTurnBrakeLead = 3.0f;   // cortar motores N° antes: la inercia
                                       // (coast, 2-12° medido vs ArUco) completa el giro
const unsigned long imuTurnSettleMs = 400;  // ventana para que el coast termine
                                            // antes de la verificación final
const float imuTurnTolerance = 3.0f;   // residuo bajo el cual el giro se da por bueno;
                                       // con corrección iterativa ahora sí aterriza acá
const int imuTurnMaxCorrections = 4;   // tope de correcciones por giro — evita
                                       // perseguir el ruido del gyro indefinidamente
const int instructionCompletedDelay = 400;
std::array<float, 2> fsmInstruction;
std::deque<std::array<float, 2>> instructionList;

// ¿Ya hay un REQUEST_POSITION esperando en la cola? El guard
// congregation.waitingForResponse solo se levanta cuando la FSM EJECUTA la
// instrucción, no cuando se encola; entre ambos momentos puede haber cientos de
// ms (el WAIT de arranque de CONGREGATION). Sin este chequeo se encolaban dos
// pedidos, llegaban dos POSITION_RESPONSE, corrían dos ReactiveNavStep y la cola
// quedaba con un ciclo duplicado: de ahí en más el robot ejecutaba giros
// calculados para una pose vieja (medido 2026-07-27: giros un ciclo atrasados).
bool RequestPositionQueued() {
  for (const auto &ins : instructionList) {
    if ((int)ins[0] == REQUEST_POSITION) return true;
  }
  return false;
}
pose robotPose(0, 0, 0);

// Evasión
bool isEvading = false;
unsigned long evasionStartTime = 0;
const unsigned long evasionCooldown = 2000;
bool resumeScheduled = false;
bool obstacleDetected = false;

// Controladores PID
pidController leftControl(kfPID, pidSpeed, samplingTimeS, minPWMValue,
                          maxPWMValue);
pidController rightControl(kfPID, pidSpeed, samplingTimeS, minPWMValue,
                           maxPWMValue);

// Hardware
Adafruit_APDS9960 frontSensor;
CRGB leds[NUM_LEDS];
WiFiUDP udp;
ICM_20948_I2C imu;
Preferences preferences;

// ============================================================================
// DECLARACIONES FORWARD DE FUNCIONES
// ============================================================================

// Setup y configuración
void SetupFrontSensor();
void WiFiStatus();
void updateMillimetersPerPulse();
void InitializePPR();
void SavePPR(float newPPR);
void InitializePID();
void SavePID(float kp, float ki, float kd);

// Comunicación
void ReadUdpPackets();
void SendMessage(IPAddress host, const char *message);
void SendPose();
void MessageDebugf(const char *format, ...);

// Sensores y control
void ReadSensors();
void ResetPID();
void ConfigureHBridge(int leftWheelPWM, int rightWheelPWM);

// Movimiento
bool MoveDistanceByWheel(float leftDistance, float rightDistance);
float DesiredSpeed(float distance, float wheelDistance);
bool IsStationary(float currentLeftSpeed, float currentRightSpeed,
                  float leftWheelDistance, float rightWheelDistance);
void SelectMovementRW();

// Auxiliares
std::array<String, 6> SeparateCommand(const String &command, char delimiter);
void MaybeDisperseHop();
int  MeetSlotIndex(float tx, float ty, float ring, int n);
bool IsRobotObstacle(float x2, float y2, float angle, int sensors, String id);
void ReadSerialCommands();
// Nota: CalculateDistance, NormalizeAngle e InRange están definidas inline en utils.h
// Se redeclaran aquí para garantizar visibilidad desde este translation unit
inline float CalculateDistance(float x1, float y1, float x2, float y2);
inline float CalculateAngleToTarget(float x1, float y1, float x2, float y2);
inline float NormalizeAngle(float angle);

// LED
void setLedColor(uint8_t red, uint8_t green, uint8_t blue);
void setLedBrightness(uint8_t brightness);
void setLedBlink(uint8_t red, uint8_t green, uint8_t blue,
                 unsigned long intervalMs);

// IMU
void setupIMU();
void SaveIMUBias(biasStore* store);
void LeerYaw();
void EkfTick();
bool MatchColor(const char *target, uint16_t r, uint16_t g, uint16_t b,
                uint16_t c);
void SearchEvadeAndResume();

// ============================================================================
// INTERRUPCIONES (ISR)
// ============================================================================

void i2cScan() {
  Serial.println("\n=== I2C SCAN ===");
  int found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("  Dispositivo en 0x%02X", addr);
      if (addr == 0x68) Serial.print("  ← IMU (AD0=GND)");
      if (addr == 0x69) Serial.print("  ← IMU (AD0=VCC)");
      if (addr == 0x39) Serial.print("  ← APDS9960");
      Serial.println();
      found++;
    }
  }
  Serial.printf("  Total: %d dispositivo(s)\n", found);
  Serial.println("================\n");
}

void IRAM_ATTR LeftWheelPulses() {
  int MSB = digitalRead(leftEncoderC2);
  int LSB = digitalRead(leftEncoderC1);
  int encoder = (MSB << 1) | LSB;
  int sum = (pastLeftEncoder << 2) | encoder;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    movement.leftPulseCount++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    movement.leftPulseCount--;
  }
  pastLeftEncoder = encoder;
}

void IRAM_ATTR RightWheelPulses() {
  int MSB = digitalRead(rightEncoderC1);
  int LSB = digitalRead(rightEncoderC2);
  int encoder = (MSB << 1) | LSB;
  int sum = (pastRightEncoder << 2) | encoder;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    movement.rightPulseCount++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    movement.rightPulseCount--;
  }
  pastRightEncoder = encoder;
}

void IRAM_ATTR DetectLeftObstacle() {
  if (lateralSensorsEnabled && digitalRead(leftInfraredSensor) == LOW) {
    leftObsStartTime = micros();
  }
}

void IRAM_ATTR DetectRightObstacle() {
  if (lateralSensorsEnabled && digitalRead(rightInfraredSensor) == LOW) {
    rightObsStartTime = micros();
  }
}

void LowBattery() {
  if (digitalRead(batteryStatus) == LOW) {
    lowBatteryTime = millis();
  }
}

// ============================================================================
// FUNCIONES DE SETUP Y CONFIGURACIÓN
// ============================================================================

void InitializePPR() {
  preferences.begin("attabot-config", false);

  float storedPPR = preferences.getFloat("ppr", 0);

  if (storedPPR == 0) {
    preferences.putFloat("ppr", pulsesPerRev);
    DebugSerialPrintf("PPR inicial guardado: %.2f\n", pulsesPerRev);
  } else {
    pulsesPerRev = storedPPR;
    DebugSerialPrintf("PPR cargado desde memoria: %.2f\n", pulsesPerRev);
  }

  updateMillimetersPerPulse();

  yawScale = preferences.getFloat("yaw_scale", 1.0f);
  DebugSerialPrintf("Yaw scale: %.4f\n", yawScale);

  centralIRThreshold = preferences.getInt("ir_cen_thr", 2);
  DebugSerialPrintf("Umbral IR central: %d\n", centralIRThreshold);

  preferences.end();

  uint64_t chipid = ESP.getEfuseMac();
  DebugSerialPrintf("Robot Chip ID: %04X%08X\n", (uint16_t)(chipid >> 32),
                    (uint32_t)chipid);
}

void SavePPR(float newPPR) {
  preferences.begin("attabot-config", false);
  preferences.putFloat("ppr", newPPR);
  preferences.end();
  DebugSerialPrintf("PPR guardado permanentemente: %.2f\n", newPPR);
}

void InitializePID() {
  preferences.begin("attabot-config", false);
  int   savedRes = preferences.getInt  ("pid_res", -1);
  float kp       = preferences.getFloat("pid_kp",  -1.0f);
  float ki       = preferences.getFloat("pid_ki",  -1.0f);
  float kd       = preferences.getFloat("pid_kd",  -1.0f);
  preferences.end();

  if (kp > 0.0f && savedRes == pwm_resolution) {
    leftControl.pidConst.kp  = kp;
    leftControl.pidConst.ki  = ki;
    leftControl.pidConst.kd  = kd;
    rightControl.pidConst.kp = kp;
    rightControl.pidConst.ki = ki;
    rightControl.pidConst.kd = kd;
    DebugSerialPrintf("PID cargado desde flash: Kp=%.2f Ki=%.2f Kd=%.3f\n", kp, ki, kd);
  } else {
    if (kp > 0.0f && savedRes != pwm_resolution) {
      DebugSerialPrintf("PID en flash descartado: guardado con %d-bit, actual %d-bit\n",
                        savedRes, pwm_resolution);
    }
    DebugSerialPrintf("PID usando defaults: Kp=%.2f Ki=%.2f Kd=%.3f\n",
                      leftControl.pidConst.kp, leftControl.pidConst.ki, leftControl.pidConst.kd);
  }
}

void SavePID(float kp, float ki, float kd) {
  preferences.begin("attabot-config", false);
  preferences.putInt  ("pid_res", pwm_resolution);
  preferences.putFloat("pid_kp",  kp);
  preferences.putFloat("pid_ki",  ki);
  preferences.putFloat("pid_kd",  kd);
  preferences.end();
  DebugSerialPrintf("PID guardado (%d-bit): Kp=%.2f Ki=%.2f Kd=%.3f\n",
                    pwm_resolution, kp, ki, kd);
}

void SaveIMUBias(biasStore* store) {
  preferences.begin("attabot-config", false);
  preferences.putInt("bias_gx", store->biasGyroX);
  preferences.putInt("bias_gy", store->biasGyroY);
  preferences.putInt("bias_gz", store->biasGyroZ);
  preferences.putInt("bias_ax", store->biasAccelX);
  preferences.putInt("bias_ay", store->biasAccelY);
  preferences.putInt("bias_az", store->biasAccelZ);
  preferences.putInt("bias_cx", store->biasCPassX);
  preferences.putInt("bias_cy", store->biasCPassY);
  preferences.putInt("bias_cz", store->biasCPassZ);
  preferences.end();
  DebugSerialPrintln("Bias IMU guardados en Preferences");
}

void updateMillimetersPerPulse() {
  millimetersPerPulse = wheelCircumference / pulsesPerRev;
}

void setup() {
#ifdef DebugSerial
  Serial.begin(115200);
  delay(500); // Dar tiempo al Serial Monitor para conectar
  Serial.println("\n\n=== INICIO DE SETUP ===");
#endif

  // Delay aleatorio para evitar colisiones DHCP cuando múltiples ESP32 arrancan
  // juntos Usa la MAC address como semilla para que cada robot tenga un delay
  // único
  randomSeed(ESP.getEfuseMac());
  unsigned long startupDelay = random(100, 2000); // Entre 100ms y 2 segundos
  DebugSerialPrintf("Esperando %lu ms antes de iniciar WiFi...\n",
                    startupDelay);
  delay(startupDelay);

  DebugSerialPrintln("[1] Inicializando PPR desde flash...");
  InitializePPR();
  DebugSerialPrintln("[1] PPR OK");

  DebugSerialPrintln("[1b] Inicializando PID desde flash...");
  InitializePID();
  DebugSerialPrintln("[1b] PID OK");

  // Reloj del LEDC clavado en APB (80MHz), antes de cualquier canal (con
  // canales ya tomados la llamada falla a propósito). Por defecto el driver va
  // en LEDC_AUTO_CLK y lo elige el PRIMER periférico que se attachea; los
  // motores a 1kHz/14-bit necesitan 16.384MHz de fuente, que solo sale de APB,
  // y los 4 timers low-speed comparten un único mux de reloj. Cuando el servo
  // frontal (50Hz/10-bit) se lo llevaba al reloj lento, los ledcAttach() de
  // los motores fallaban y el robot no giraba ni avanzaba (2026-07-29). El
  // servo ya no está, pero esto deja el reloj explícito en vez de heredado.
  DebugSerialPrintln("[2] Fijando reloj del LEDC en APB...");
  if (!ledcSetClockSource(LEDC_USE_APB_CLK))
    DebugSerialPrintln("[2] AVISO: no se pudo fijar el reloj del LEDC en APB");

  DebugSerialPrintln("[3] Inicializando motores PWM...");
  bool pwmOk = ledcAttach(leftMotorForward,  pwm_freq, pwm_resolution)
             & ledcAttach(leftMotorBackward,  pwm_freq, pwm_resolution)
             & ledcAttach(rightMotorForward,  pwm_freq, pwm_resolution)
             & ledcAttach(rightMotorBackward, pwm_freq, pwm_resolution);
  if (!pwmOk) DebugSerialPrintf("[3] ERROR: ledcAttach falló — freq=%d res=%d "
                                "incompatibles, o algo más se quedó con el "
                                "reloj del LEDC (ver nota en [2])\n",
                                 pwm_freq, pwm_resolution);
  // Frecuencia REAL del timer, no la pedida: si el LEDC tuvo que ajustar (o si
  // otro periférico le movió el reloj), acá se ve. Debe leer 1000Hz.
  // ledcReadFreq() mide el CANAL y devuelve 0 si el duty es 0 (core 3.3.11,
  // esp32-hal-ledc.c:411), y en el arranque las ruedas están quietas: hay que
  // darle un duty mínimo para que la lectura sea legible. 1/16384 son 61ns de
  // pulso, muy por debajo del 20% que el puente H necesita para mover el motor.
  ledcWrite(leftMotorForward, 1);
  uint32_t realPwmFreq = ledcReadFreq(leftMotorForward);
  ledcWrite(leftMotorForward, 0);
  DebugSerialPrintf("[3] Motores PWM: %dHz %d-bit (max=%d) %s — real=%uHz\n",
                    pwm_freq, pwm_resolution, maxPWMValue, pwmOk ? "OK" : "FALLO",
                    realPwmFreq);

  DebugSerialPrintln("[4] Inicializando I2C...");
  Wire.begin();
  Wire.setClock(400000);
  i2cScan();
  DebugSerialPrintln("[4] I2C OK");

  // IMPORTANTE: setHostname DEBE estar ANTES de WiFi.begin()
  DebugSerialPrintln("[5] Inicializando WiFi...");
  WiFi.mode(WIFI_STA);
  String hostname = "AttaBot-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  WiFi.setHostname(hostname.c_str());

  DebugSerialPrintf("Iniciando WiFi con hostname: %s\n", hostname.c_str());
  WiFi.begin(ssid, password);
  DebugSerialPrintln("[5] WiFi iniciado");

  DebugSerialPrintln("[6] Inicializando LEDs...");
  FastLED.addLeds<WS2812, ledPin, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(maxBrightness);
  FastLED.setMaxRefreshRate(120);
  ledCtrl.setOff();
  DebugSerialPrintln("[6] LEDs OK");

  DebugSerialPrintln("[7] Inicializando OTA y UDP...");
  ArduinoOTA.setHostname(hostname.c_str());
  ArduinoOTA.begin();

  udp.begin(localPort);
  DebugSerialPrintf("El servidor UDP se inició en el puerto: %u\n", localPort);
  DebugSerialPrintln("[7] OTA y UDP OK");

  DebugSerialPrintln("[8] Configurando encoders...");
  pinMode(leftEncoderC1, INPUT_PULLUP);
  pinMode(leftEncoderC2, INPUT); // GPIO 35 es input-only, sin pull-up
                                 // (compatible con ESP32 Core 3.x)
  pinMode(rightEncoderC1, INPUT_PULLUP);
  pinMode(rightEncoderC2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEncoderC1), LeftWheelPulses,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoderC2), LeftWheelPulses,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderC1), RightWheelPulses,
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderC2), RightWheelPulses,
                  CHANGE);
  DebugSerialPrintln("[8] Encoders OK");

  DebugSerialPrintln("[9] Configurando sensores infrarrojos...");
  pinMode(enableLeftInfraredSensor, OUTPUT);
  pinMode(enableRightInfraredSensor, OUTPUT);

  digitalWrite(enableLeftInfraredSensor, LOW);
  digitalWrite(enableRightInfraredSensor, LOW);

  pinMode(batteryStatus, INPUT);
  attachInterrupt(digitalPinToInterrupt(batteryStatus), LowBattery, FALLING);
  pinMode(leftInfraredSensor, INPUT);
  pinMode(rightInfraredSensor, INPUT);
  attachInterrupt(digitalPinToInterrupt(leftInfraredSensor), DetectLeftObstacle,
                  FALLING);
  attachInterrupt(digitalPinToInterrupt(rightInfraredSensor),
                  DetectRightObstacle, FALLING);
  DebugSerialPrintln("[9] Sensores OK");

  // IMPORTANTE: La IMU debe inicializarse ANTES que el APDS9960.
  // Invertir este orden rompe silenciosamente la init del ICM-20948 en el bus I2C.
  DebugSerialPrintln("[10] Inicializando IMU ICM-20948...");
  setupIMU();
  if (imuAvailable) {
    DebugSerialPrintln("[10] IMU OK");
  } else {
    DebugSerialPrintln("[10] IMU no disponible — continuando sin IMU");
  }

  DebugSerialPrintln("[11] Inicializando sensor frontal APDS9960...");
  SetupFrontSensor();
  delay(200);

  DebugSerialPrintln("\n=== SETUP COMPLETO ===\n");
}

// ============================================================================
// LOOP PRINCIPAL
// ============================================================================

void loop() {
  ledCtrl.update();
  WiFiStatus();
  if (WiFi.status() != WL_CONNECTED)
    return;
  ReadUdpPackets();
  ReadSensors();
  SetupFrontSensor();

  // Lectura IMU no bloqueante — se ejecuta solo si la IMU está disponible
  // y han pasado al menos imuReadInterval ms desde la última lectura.
  if (imuAvailable && (millis() - lastImuRead >= imuReadInterval)) {
    lastImuRead = millis();
    LeerYaw();
    EkfTick();
  }

#ifdef DebugSerial
  ReadSerialCommands();
#endif

  switch (state) {
  case WAIT: {
    ArduinoOTA.handle();

    if ((millis() - movement.previousMillis) >= instructionValue) {
      movement.previousMillis = millis();
      ResetPID();
      if (movementReady) {
        state = READ_INSTRUCTION;
      } else {
        direction = '-';
        state = REVERSE;
      }
    }

    break;
  }

  case MOVE: {
    movementReady = MoveDistanceByWheel(instructionValue, instructionValue);

    if (movementReady) {
      MessageDebugf("DEBUG: -1, ID: %s, Movimiento completado",
                    robotID.c_str());
      intContext.Clear();
      isEvading = false;
      state = STOP;

    } else if (obstacles.HasAnyObstacle() && !isEvading) {
      obstacles.UpdateBitmap();

      intContext.wasInterrupted = true;
      intContext.previousState = MOVE;
      intContext.leftPulsesBeforeStop = movement.pastLeftPulseCount;
      intContext.rightPulsesBeforeStop = movement.pastRightPulseCount;

      float avgTraveled =
          (movement.pastLeftPulseCount + movement.pastRightPulseCount) / 2.0 *
          millimetersPerPulse;
      intContext.remainingValue = instructionValue - avgTraveled;

      isEvading = true;
      evasionStartTime = millis();

      MessageDebugf("DEBUG: -1, ID: %s, MOVE interrumpido: restante=%.1fmm",
                    robotID.c_str(), intContext.remainingValue);

      state = STOP;
    }

    break;
  }

  case TURN: {
    // Capturar yaw inicial la primera vez que se entra al estado
    if (imuAvailable && !imuTurnActive) {
      imuTurnActive      = true;
      imuTurnStartYaw    = yaw;
      imuTurnPrevYaw     = yaw;
      imuTurnAccumDeg    = 0.0f;
      imuTurnSettleUntil = 0;
      imuTurnTargetDeg = (instructionValue / centerToWheelDistance) * RAD_TO_DEG;
    }

    if (imuAvailable && imuTurnActive) {
      // Giro cerrado en yaw: acumular el giro real con unwrap incremental y
      // re-apuntar el arco restante en cada ciclo. El objetivo lo define el
      // IMU, no la geometría supuesta — rueda loca, stiction y wheel_dist
      // dejan de producir déficit.
      float dYaw = yaw - imuTurnPrevYaw;
      if (dYaw >  180.0f) dYaw -= 360.0f;
      if (dYaw < -180.0f) dYaw += 360.0f;
      imuTurnAccumDeg += dYaw * yawScale;  // grados físicos, no crudos del gyro
      imuTurnPrevYaw   = yaw;

      if (imuTurnSettleUntil != 0) {
        // Motores ya cortados: seguir midiendo hasta que el coast termine,
        // así la verificación final incluye la inercia (la cámara mostraba
        // 2-12° de giro extra después del corte que el IMU no contaba)
        if (millis() < imuTurnSettleUntil) {
          break;
        }
        imuTurnSettleUntil = 0;

        float delta = imuTurnAccumDeg;           // unwrapped: válido >180°
        float error = imuTurnTargetDeg - delta;  // positivo = giró de menos
        MessageDebugf("DEBUG: -1, ID: %s, TURN IMU: objetivo=%.1f° real=%.1f° error=%.1f° corr#%d (yaw %.1f→%.1f)",
                      robotID.c_str(), imuTurnTargetDeg, delta, error,
                      imuTurnCorrCount, imuTurnStartYaw, yaw);

        if (abs(error) > imuTurnTolerance &&
            imuTurnCorrCount < imuTurnMaxCorrections) {
          // Closed-loop iterativo: encolar otra corrección y volver a medir
          // tras su settle. Ya no es de un solo tiro — itera hasta |error|<tol
          // o agotar imuTurnMaxCorrections. La corrección corre SIN brake-lead
          // (ver rama de manejo) para que el arco chico no se cancele solo.
          float corrArc = DegToArc(error);
          fsmInstruction[0] = TURN;
          fsmInstruction[1] = corrArc;
          instructionList.push_front(fsmInstruction);
          imuTurnIsCorrection = true;
          imuTurnCorrCount++;
          MessageDebugf("DEBUG: -1, ID: %s, TURN corrección #%d: %.1f° (arc=%.1fmm)",
                        robotID.c_str(), imuTurnCorrCount, error, corrArc);
        } else {
          imuTurnIsCorrection = false;
          imuTurnCorrCount    = 0;
        }
        imuTurnActive = false;
        movementReady = true;
      } else {
        float leftTraveled  = movement.leftPulseCount * millimetersPerPulse;
        float rightTraveled = movement.rightPulseCount * millimetersPerPulse;

        // Failsafe: si el yaw no avanza (IMU muda/congelada), cerrar por
        // encoders para no girar infinito
        if (fabs(leftTraveled) > fabs(instructionValue) * 1.5f + 20.0f) {
          MessageDebugf("DEBUG: -1, ID: %s, TURN failsafe: yaw estancado "
                        "(%.1f° de %.1f°), cierro por encoders",
                        robotID.c_str(), imuTurnAccumDeg, imuTurnTargetDeg);
          imuTurnActive       = false;
          imuTurnIsCorrection = false;
          imuTurnCorrCount    = 0;
          movementReady       = true;
        } else {
          // Frenar imuTurnBrakeLead antes del objetivo: la inercia completa el
          // giro. PERO en una corrección el arco es chico y no hay momento que
          // costear → lead=0; con lead la corrección se cancelaría a sí misma.
          float lead = imuTurnIsCorrection
                           ? 0.0f
                           : ((imuTurnTargetDeg >= 0) ? imuTurnBrakeLead
                                                      : -imuTurnBrakeLead);
          float remainingArc =
              DegToArc(imuTurnTargetDeg - lead - imuTurnAccumDeg);
          bool reached = MoveDistanceByWheel(leftTraveled + remainingArc,
                                             rightTraveled - remainingArc);
          if (reached) {
            ConfigureHBridge(0, 0);
            imuTurnSettleUntil = millis() + imuTurnSettleMs;
          }
          movementReady = false;  // el giro termina tras el settle
        }
      }
    } else {
      movementReady = MoveDistanceByWheel(instructionValue, -instructionValue);
    }

    if (movementReady) {
      MessageDebugf("DEBUG: -1, ID: %s, Giro completado", robotID.c_str());
      intContext.Clear();
      isEvading = false;
      state = STOP;

    } else if (obstacles.HasAnyObstacle() && !isEvading) {
      obstacles.UpdateBitmap();

      intContext.wasInterrupted = true;
      intContext.previousState = TURN;
      intContext.leftPulsesBeforeStop = movement.pastLeftPulseCount;

      float traveled = movement.pastLeftPulseCount * millimetersPerPulse;
      intContext.remainingValue = instructionValue - traveled;

      isEvading = true;
      evasionStartTime = millis();

      float angleRemaining =
          (intContext.remainingValue / centerToWheelDistance) * RAD_TO_DEG;
      MessageDebugf("DEBUG: -1, ID: %s, TURN interrumpido: restante=%.1f°",
                    robotID.c_str(), angleRemaining);

      if (obstacles.centralObstacle) {
        movementReady = false;
      }

      imuTurnActive = false;  // cancelar seguimiento IMU si el giro fue interrumpido
      imuTurnIsCorrection = false;
      imuTurnCorrCount = 0;
      state = STOP;
    }

    break;
  }

  case RANDOM_WALK: {
    if (previousMillisRW == 0) {
      previousMillisRW = millis();
    }

    currentMillis = millis();
    millisDifference = currentMillis - previousMillisRW;
    if (millisDifference < instructionValue) {
      previousMillisRW = currentMillis;
      fsmInstruction[0] = RANDOM_WALK;
      fsmInstruction[1] = instructionValue - millisDifference;
      instructionList.push_front(fsmInstruction);
      SelectMovementRW();
    } else {
      previousMillisRW = 0;
      MessageDebugf("DEBUG: -1, ID: %s, Random Walk terminado",
                    robotID.c_str());
    }

    state = READ_INSTRUCTION;
    break;
  }

  case REVERSE: {
    movementReady = MoveDistanceByWheel(reverseDistance, reverseDistance);

    if (movementReady) {
      obstacleDetected = true;
      MessageDebugf("DEBUG: -1, ID: %s, Retroceso completado", robotID.c_str());
      state = STOP;
    } else if (obstacles.HasAnyObstacle() && !isEvading) {
      MessageDebugf("DEBUG: -1, ID: %s, Obstáculo durante retroceso!",
                    robotID.c_str());
      state = STOP;
    }

    break;
  }

  case STOP: {
    ConfigureHBridge(0, 0);

    if (movementReady == true) {
      if (!intContext.wasInterrupted) {
        isEvading = false;
        resumeScheduled = false;
      }

      state = WAIT;
      instructionValue = instructionCompletedDelay;

    } else {
      SendPose();
      state = IDENTIFY_OBSTACLE;
    }

    break;
  }

  case READ_INSTRUCTION: {
    if (!isEvading && !intContext.wasInterrupted) {

      obstacles.Clear();
    }

    if (!instructionList.empty()) {
      fsmInstruction = instructionList.front();
      instructionList.pop_front();
      instructionValue = fsmInstruction[1];
      state = static_cast<RobotState>(fsmInstruction[0]);
      direction = instructionValue > 0 ? '+' : '-';
      // Sonda: qué valor sale realmente de la cola (2026-07-29, TURN con
      // objetivo=0°). Solo con debug activo; MOVE=1 TURN=2 WAIT=3.
      if (state == TURN || state == MOVE) {
        MessageDebugf("DEBUG: -1, ID: %s, READ: estado=%d valor=%.2f (quedan %d)",
                      robotID.c_str(), (int)state, instructionValue,
                      (int)instructionList.size());
      }

    } else {
      state = WAIT;
      instructionValue = instructionCompletedDelay;
    }

    break;
  }

  case MESSAGE_BASE: {
    const char *message = "";
    if (instructionValue == 1) {
      message = "READY";
    }

    SendMessage(robots["Base"], message);
    state = WAIT;
    instructionValue = instructionCompletedDelay;

    break;
  }

  case IDENTIFY_OBSTACLE: {
    currentMillis = millis();
    if (obstacles.robotDetected) {
      state = WAIT;
      instructionValue = instructionCompletedDelay / 2;
      MessageDebugf("DEBUG: -1, ID: %s, Obstáculo encontrado es robot id: %s",
                    robotID.c_str(), obstacles.fromRobotID.c_str());
    } else if ((currentMillis - movement.previousMillis) >= obstacleWaitTime) {
      movement.previousMillis = currentMillis;
      state = ACTIVE_EVASION;
      instructionValue = 0;
      MessageDebugf("DEBUG: -1, ID: %s, Obstáculo encontrado no es un robot",
                    robotID.c_str());
    }

    break;
  }

  case REQUEST_POSITION: {
    if (robots.find("Base") == robots.end() ||
        robots["Base"] == IPAddress(0, 0, 0, 0)) {
      MessageDebugf("DEBUG: -1, ID: %s, No hay IP de base — abortando nav",
                    robotID.c_str());
      congregation.CompleteRequest();
      state = WAIT;
      instructionValue = 500;
      break;
    }

    if (!congregation.waitingForResponse) {
      SendMessage(robots["Base"], "REQUEST_POSITION");
      MessageDebugf("DEBUG: -1, ID: %s, Solicitud enviada: REQUEST_POSITION",
                    robotID.c_str());
      congregation.StartRequest();
    }

    if (congregation.HasTimedOut()) {
      MessageDebugf("DEBUG: -1, ID: %s, Timeout en REQUEST_POSITION",
                    robotID.c_str());
      congregation.CompleteRequest();
      state = WAIT;
      instructionValue = 500;
      break;
    }

    if (congregation.positionReceived) {
      MessageDebugf("DEBUG: -1, ID: %s, Posición recibida",
                    robotID.c_str());
      congregation.CompleteRequest();
      congregation.positionReceived = false;

      // nav activo: obstáculo ya en obstacles struct, ReactiveNavStep lo procesará
      state = READ_INSTRUCTION;
      instructionValue = 0;
    }

    break;
  }

  case ACTIVE_EVASION: {
    if (!obstacles.HasAnyObstacle()) {
      // Sin obstáculo real — puede haber desaparecido entre detección y aquí
      MessageDebugf(
          "DEBUG: -1, ID: %s, ACTIVE_EVASION sin obstáculo. Abortando.",
          robotID.c_str());
      obstacles.Clear();
      state = READ_INSTRUCTION;
      break;
    }

    // Si solo disparó el sensor frontal APDS9960 (sin IR laterales),
    // normalizarlo como obstáculo central para el pattern matching.
    if (obstacles.obstacleSensors == 0 && obstacles.centralObstacle) {
      obstacles.obstacleSensors = 0b010;
    }

    // SEARCH activo: un obstáculo central es un CANDIDATO — aproximarse y
    // leerle el color en vez de evadir (los laterales evaden normal)
    if (search.active && obstacles.centralObstacle) {
      instructionList.clear();
      obstacleDetected = false;
      isEvading = false;
      resumeScheduled = false;
      intContext.Clear();
      obstacles.Clear();
      search.approachStart = millis();
      ConfigureHBridge(SEARCH_CREEP_PWM, SEARCH_CREEP_PWM);
      MessageDebugf("DEBUG: -1, ID: %s, SEARCH: candidato central — aproximando",
                    robotID.c_str());
      state = SEARCH_APPROACH;
      break;
    }

    unsigned long timeSinceDetection = millis() - evasionStartTime;
    if (timeSinceDetection > 1500) {
      MessageDebugf("DEBUG: -1, ID: %s, Datos de obstáculo obsoletos (%lums). "
                    "Re-escaneando.",
                    robotID.c_str(), timeSinceDetection);
      state = STOP;
      break;
    }

    evasionTracker.RecordEvasion();

    int avoidanceDistance = 0;
    int avoidanceAngle = 0;
    bool needsRetreat = evasionTracker.ShouldRetreat();

    if (needsRetreat) {
      MessageDebugf(
          "DEBUG: -1, ID: %s, Ejecutando retroceso forzado (evasiones: %d)",
          robotID.c_str(), evasionTracker.consecutiveEvasions);

      std::deque<std::array<float, 2>> retreatSequence;

      fsmInstruction[0] = REVERSE;
      fsmInstruction[1] = reverseDistance * 3;
      retreatSequence.push_back(fsmInstruction);

      // Lado del rodeo hacia el INTERIOR de la arena (no ciego): proyecta el
      // tramo por cada lado y toma el que queda más adentro (lejos de paredes).
      // Corrige el thrash en esquina; espeja escape_turn() de la sim.
      float escAng   = escapeTurnDeg;
      float bestScore = -1.0e9f;
      for (int s = -1; s <= 1; s += 2) {
        float th = DegToRad(robotPose.angle + s * escapeTurnDeg);
        float ex = robotPose.x + escapeMoveMm * cos(th);
        float ey = robotPose.y + escapeMoveMm * sin(th);
        float score = min(min(ex, arenaWidthMm - ex),
                          min(ey, arenaHeightMm - ey));
        if (score > bestScore) {
          bestScore = score;
          escAng    = s * escapeTurnDeg;
        }
      }

      MessageDebugf("DEBUG: -1, ID: %s, Escape de deadlock: rodeo %.0f° hacia interior",
                    robotID.c_str(), escAng);

      fsmInstruction[0] = TURN;
      fsmInstruction[1] = DegToArc(escAng);
      retreatSequence.push_back(fsmInstruction);

      fsmInstruction[0] = MOVE;
      fsmInstruction[1] = escapeMoveMm;
      retreatSequence.push_back(fsmInstruction);

      for (auto it = retreatSequence.rbegin(); it != retreatSequence.rend();
           ++it) {
        instructionList.push_front(*it);
      }

      evasionTracker.Reset();

    } else {
      if (obstacles.obstacleSensors == 0b100) {
        avoidanceAngle = 45;
        avoidanceDistance = 120;
      } else if (obstacles.obstacleSensors == 0b001) {
        avoidanceAngle = -45;
        avoidanceDistance = 120;
      } else if (obstacles.obstacleSensors == 0b010) {
        avoidanceAngle = (random(2) == 0) ? 60 : -60;
        avoidanceDistance = 150;
      } else if (obstacles.obstacleSensors == 0b110) {
        avoidanceAngle = 90;
        avoidanceDistance = 150;
      } else if (obstacles.obstacleSensors == 0b011) {
        avoidanceAngle = -90;
        avoidanceDistance = 150;
      } else if (obstacles.obstacleSensors == 0b111) {
        avoidanceAngle = (random(2) == 0) ? 135 : -135;
        avoidanceDistance = 100;
      }

      if (!obstacles.HasAnyObstacle()) {
        MessageDebugf("DEBUG: -1, ID: %s, Obstáculo desapareció durante "
                      "cálculo de evasión",
                      robotID.c_str());
        obstacles.Clear();
        state = READ_INSTRUCTION;
        break;
      }

      std::deque<std::array<float, 2>> evasionSequence;

      if (obstacles.obstacleSensors & 0b010 ||
          obstacles.obstacleSensors == 0b111) {
        fsmInstruction[0] = REVERSE;
        fsmInstruction[1] = reverseDistance * 1.5;
        evasionSequence.push_back(fsmInstruction);
      }

      if (avoidanceAngle != 0) {
        fsmInstruction[0] = TURN;
        fsmInstruction[1] = DegToArc(avoidanceAngle);
        evasionSequence.push_back(fsmInstruction);
      }

      if (avoidanceDistance > 0) {
        fsmInstruction[0] = MOVE;
        fsmInstruction[1] = avoidanceDistance;
        evasionSequence.push_back(fsmInstruction);
      }

      for (auto it = evasionSequence.rbegin(); it != evasionSequence.rend();
           ++it) {
        instructionList.push_front(*it);
      }
    }

    if (intContext.wasInterrupted && !resumeScheduled) {
      fsmInstruction[0] = RESUME_AFTER_EVASION;
      fsmInstruction[1] = 0;
      instructionList.push_back(fsmInstruction);
      resumeScheduled = true;
    }

    obstacleDetected = false;

    state = READ_INSTRUCTION;

    MessageDebugf("DEBUG: -1, ID: %s, Evasión: patrón=%s, giro=%d°, "
                  "avance=%dmm, forzado=%d",
                  robotID.c_str(), obstacles.GetObstaclePattern().c_str(),
                  avoidanceAngle, avoidanceDistance, needsRetreat);

    break;
  }

  case RESUME_AFTER_EVASION: {
    if (!intContext.wasInterrupted) {
      resumeScheduled = false;
      obstacles.Clear();
      state = READ_INSTRUCTION;
      break;
    }

    MessageDebugf(
        "DEBUG: -1, ID: %s, Resumiendo: estado=%d, valor=%.1f",
        robotID.c_str(), intContext.previousState, intContext.remainingValue);

    if (nav.isActive) {
      // Navegación activa: re-solicitar posición para recalcular ruta en vez
      // de terminar a ciegas el segmento interrumpido (nav.isActive faltaba
      // tras la migración a ReactiveNav — causaba desvíos largos al evadir)
      MessageDebugf(
          "DEBUG: -1, ID: %s, GT activo: solicitando posición post-evasión",
          robotID.c_str());
      fsmInstruction[0] = REQUEST_POSITION;
      fsmInstruction[1] = 0;
      instructionList.push_front(fsmInstruction);

    } else {
      switch (intContext.previousState) {
      case MOVE: {
        if (intContext.remainingValue > 20) {
          fsmInstruction[0] = MOVE;
          fsmInstruction[1] = intContext.remainingValue;
          instructionList.push_front(fsmInstruction);

          MessageDebugf("DEBUG: -1, ID: %s, Reanudando MOVE: %.1fmm restantes",
                        robotID.c_str(), intContext.remainingValue);
        }
        break;
      }

      case TURN: {
        float angleRemaining = abs(
            (intContext.remainingValue / centerToWheelDistance) * RAD_TO_DEG);
        if (angleRemaining > 5) {
          fsmInstruction[0] = TURN;
          fsmInstruction[1] = intContext.remainingValue;
          instructionList.push_front(fsmInstruction);

          MessageDebugf("DEBUG: -1, ID: %s, Reanudando TURN: %.1f° restantes",
                        robotID.c_str(), angleRemaining);
        }
        break;
      }
      }
    }

    intContext.Clear();
    resumeScheduled = false;
    isEvading = false;
    obstacles.Clear();

    evasionTracker.Reset();

    state = READ_INSTRUCTION;
    break;
  }

  case SEARCH_APPROACH: {
    // Aproximación lenta al candidato hasta el alcance del APDS9960 (~pocos
    // cm), donde la lectura de color es confiable. Validado en sim (Webots).
    if (!search.active || !frontSensorInitialized) {
      ConfigureHBridge(0, 0);
      state = READ_INSTRUCTION;
      break;
    }

    if (millis() - search.approachStart > SEARCH_APPROACH_TIMEOUT) {
      ConfigureHBridge(0, 0);
      MessageDebugf("DEBUG: -1, ID: %s, SEARCH: aproximación agotada — evadiendo",
                    robotID.c_str());
      SearchEvadeAndResume();
      break;
    }

    if (frontSensor.readProximity() < SEARCH_PROX_NEAR) {
      break;   // seguir avanzando lento (motores ya configurados)
    }

    // Al alcance: detenerse y esperar una lectura de color válida (~100ms)
    ConfigureHBridge(0, 0);
    if (!frontSensor.colorDataReady()) {
      break;
    }
    uint16_t r, g, b, c;
    frontSensor.getColorData(&r, &g, &b, &c);
    bool match = MatchColor(search.targetColor, r, g, b, c);
    MessageDebugf("DEBUG: -1, ID: %s, SEARCH: RGBC=%u,%u,%u,%u → %s",
                  robotID.c_str(), r, g, b, c,
                  match ? search.targetColor : "no coincide");

    if (match) {
      // El objeto está ~100mm frente al robot
      float ox = robotPose.x + 100.0f * cos(DegToRad(robotPose.angle));
      float oy = robotPose.y + 100.0f * sin(DegToRad(robotPose.angle));
      char buf[80];
      snprintf(buf, sizeof(buf), "OBJECT_FOUND|%s|%.0f|%.0f|%s",
               robotID.c_str(), ox, oy, search.targetColor);
      SendMessage(robots["Base"], buf);
      MessageDebugf("DEBUG: -1, ID: %s, OBJETO %s ENCONTRADO en (%.0f,%.0f)",
                    robotID.c_str(), search.targetColor, ox, oy);
      search.Reset();
      frontSensor.enableColor(false);
      instructionList.clear();
      ledCtrl.setSolid(0, 255, 0, maxBrightness);
      state = STOP;
    } else {
      SearchEvadeAndResume();
    }
    break;
  }
  }
}

// ============================================================================
// NAVEGACIÓN REACTIVA UNIFICADA — GT y Congregación
// ============================================================================

// Ejecuta un paso de navegación hacia (nav.goalX, nav.goalY).
// Calcula el ángulo hacia el objetivo y aplica bias reactivo si hay obstáculo
// en los sensores IR. Encola TURN+WAIT+MOVE+WAIT+REQUEST_POSITION.
// Llamar desde el handler de POSITION_RESPONSE cuando nav.isActive.
// Recalcula el slot de congregación y el goal de navegación a partir de la
// pose del líder. La usan DOS caminos: LEADER_POSITION (líder robot, difunde su
// pose) y CONGREGATION|VIRTUAL (punto fijo, sin nadie que difunda). Extraerla
// evita que las dos formas de congregar se desincronicen al tocar una sola.
void UpdateCongregationGoal(float leaderX, float leaderY, float leaderAngle) {

      // Calcular el slot y el waypoint de aproximación (staging → parking).
      const float STAGING_MARGIN = 150.0f;
      int    n     = max(1, congregation.totalFollowers);
      String shape = congregation.formationShape;
      float  parkX, parkY;

      if (shape == "linea" || shape == "cuna") {
        // Slot perpendicular al heading del líder (fila), con eje opcional de la
        // Base (formationAxis). cuna: además desplazado k·s hacia atrás (V detrás
        // del líder). Staging POR DETRÁS de la fila (opuesto al heading) para que
        // cada robot entre por su propio carril y no cruce los slots vecinos.
        float leaderAngle = leaderAngle;   // ° heading del líder
        float s   = congregation.parkingDist;
        float rad = leaderAngle * PI / 180.0f;
        float hx  = cos(rad), hy = sin(rad);          // heading unitario
        float pa  = rad + PI / 2.0f + congregation.formationAxis * PI / 180.0f;
        float px  = cos(pa), py = sin(pa);            // eje de la fila
        int   k    = congregation.followerIndex / 2 + 1;
        int   side = (congregation.followerIndex % 2 == 0) ? 1 : -1;
        float ox   = side * k * s * px;
        float oy   = side * k * s * py;
        if (shape == "cuna") { ox -= k * s * hx; oy -= k * s * hy; }
        congregation.slotX = leaderX + ox;
        congregation.slotY = leaderY + oy;
        if (congregation.stagingDone) {
          parkX = congregation.slotX;
          parkY = congregation.slotY;
        } else {
          parkX = congregation.slotX - STAGING_MARGIN * hx;
          parkY = congregation.slotY - STAGING_MARGIN * hy;
        }
      } else {
        // circulo / congregación clásica: slot en círculo alrededor del líder,
        // aproximación radial (waypoint STAGING_MARGIN más lejos por el mismo
        // rayo → la recta al goal nunca cruza el círculo de parking ni al líder).
        // El slot de n==1 sale del bearing líder→ROBOT, así que exige una pose
        // propia fresca. LEADER_POSITION suele llegar ANTES del primer
        // POSITION_RESPONSE: latchear ahí anclaba el slot a una lectura vieja y
        // el robot salía al lado contrario, sin recalcular nunca (2026-07-27).
        // Sin pose fresca: pedirla y esperar al próximo LEADER_POSITION (4Hz).
        if (!congregation.slotAngleSet && n == 1 && !congregation.poseFresh) {
          if (!congregation.waitingForResponse && !nav.pendingInit &&
              !RequestPositionQueued()) {
            fsmInstruction[0] = REQUEST_POSITION;
            fsmInstruction[1] = 0;
            instructionList.push_back(fsmInstruction);
          }
          return;
        }

        if (!congregation.slotAngleSet) {
          // Anillo wall-safe DETERMINISTA (ver SafeRingSlotAngle): todos los
          // seguidores calculan el mismo anillo con datos compartidos y cada
          // uno toma su índice — la Base solo comparte posiciones.
          float bearing = atan2(robotPose.y - leaderY, robotPose.x - leaderX);
          float effR;
          congregation.slotAngle = SafeRingSlotAngle(
              leaderX, leaderY, congregation.followerIndex, n,
              congregation.parkingDist, bearing, n == 1, &effR);
          congregation.slotRadius = effR;
          congregation.slotAngleSet = true;
          // Saltarse el staging solo si el slot quedó SOBRE el rayo
          // líder→robot con el radio nominal: ahí la recta al slot no cruza
          // al líder. Si el anillo se corrió o creció, aproximación radial.
          if (n == 1 && fabs(congregation.slotAngle - bearing) < 0.01f &&
              effR <= congregation.parkingDist + 1.0f) {
            congregation.stagingDone = true;
          }
        }
        float angle = congregation.slotAngle;
        congregation.slotX = leaderX + congregation.slotRadius * cos(angle);
        congregation.slotY = leaderY + congregation.slotRadius * sin(angle);
        float goalDist = congregation.stagingDone
                             ? congregation.slotRadius
                             : congregation.slotRadius + STAGING_MARGIN;
        parkX = leaderX + goalDist * cos(angle);
        parkY = leaderY + goalDist * sin(angle);
      }

      if (nav.isActive) {
        // Banda muerta: LEADER_POSITION llega ~3Hz con la pose CRUDA del líder,
        // así que sin esto el goal se corría unos pocos mm en cada mensaje y el
        // navegador re-apuntaba contra el ruido en vez de contra el movimiento
        // real del líder. Solo se reubica si el cambio es significativo.
        if (CalculateDistance(nav.goalX, nav.goalY, parkX, parkY) >
            nav.goalDeadband) {
          nav.goalX = parkX;
          nav.goalY = parkY;
        }
      } else if (!nav.pendingInit) {
        nav.goalX       = parkX;
        nav.goalY       = parkY;
        nav.pendingInit = true;
        if (!congregation.waitingForResponse && !RequestPositionQueued()) {
          fsmInstruction[0] = REQUEST_POSITION;
          fsmInstruction[1] = 0;
          instructionList.push_back(fsmInstruction);
        }
        MessageDebugf("DEBUG: -1, ID: %s, %s: slot %d/%d → %s (%.1f,%.1f)",
                      robotID.c_str(),
                      shape.length() ? shape.c_str() : "CONGREGATION",
                      congregation.followerIndex, n,
                      congregation.stagingDone ? "parking" : "staging",
                      parkX, parkY);
      }
}

void ReactiveNavStep() {
  // Fuente de pose para el control. Con EKF_NAV|1 se navega con el estado
  // fusionado (encoders+gyro+ArUco) en vez del ArUco crudo: el rumbo del EKF no
  // trae el σ≈3.8° de la cámara, que es lo que disparaba los giros espurios.
  bool  useEkf  = ekfNavEnabled && ekf.initialized;
  float x       = useEkf ? ekf.x : robotPose.x;
  float y       = useEkf ? ekf.y : robotPose.y;
  float heading = useEkf ? ekf.AngleDeg() : robotPose.angle;

  if (nav.HasReached(x, y)) {
    if (congregation.IsActive() && !congregation.isLeader &&
        !congregation.stagingDone) {
      // Etapa 1 (waypoint de aproximación) alcanzada: entrada radial al slot
      congregation.stagingDone = true;
      nav.goalX = congregation.slotX;
      nav.goalY = congregation.slotY;
      MessageDebugf("DEBUG: -1, ID: %s, NAV: staging listo, entrando al slot "
                    "(%.1f,%.1f)",
                    robotID.c_str(), nav.goalX, nav.goalY);
    } else {
      MessageDebugf("DEBUG: -1, ID: %s, NAV: llegó a (%.1f,%.1f)",
                    robotID.c_str(), x, y);
      nav.Reset();
      instructionList.clear();
      return;
    }
  }

  if (nav.HasTimedOut()) {
    MessageDebugf("DEBUG: -1, ID: %s, NAV: timeout", robotID.c_str());
    nav.Reset();
    return;
  }

  float dx   = nav.goalX - x;
  float dy   = nav.goalY - y;
  float dist = sqrt(dx * dx + dy * dy);

  float goalAngle = atan2(dy, dx) * RAD_TO_DEG;

  // ── Capa reactiva de obstáculos ──────────────────────────────────────────
  bool frontBlocked = obstacles.centralObstacle || obstacles.IsFrontalObstacle();
  bool rightBlocked = obstacles.rightObstacle;
  bool leftBlocked  = obstacles.leftObstacle;
  obstacles.Clear();
  isEvading = false;

  float bias    = 0.0f;
  float seg     = constrain(dist * 0.9f, 10.0f, nav.segmentDistance);
  bool avoiding = false;

  if (frontBlocked) {
    // Elegir lado de evasión hacia el objetivo para "doblar" correctamente
    float relGoal = NormalizeAngle(goalAngle - heading);
    bias    = (relGoal >= 0.0f) ? -nav.avoidFrontAngle : nav.avoidFrontAngle;
    seg     = nav.avoidSegment;
    avoiding = true;
  } else if (rightBlocked) {
    bias    = nav.avoidSideAngle;   // bias izquierda
    seg     = nav.avoidSegment;
    avoiding = true;
  } else if (leftBlocked) {
    bias    = -nav.avoidSideAngle;  // bias derecha
    seg     = nav.avoidSegment;
    avoiding = true;
  }

  float finalAngle = NormalizeAngle(goalAngle + bias);
  float angleDiff  = NormalizeAngle(finalAngle - heading);

  MessageDebugf("DEBUG: -1, ID: %s, NAV: dist=%.1f goal=%.1f° head=%.1f°(%s) "
                "diff=%.1f° bias=%.1f° seg=%.1f%s",
                robotID.c_str(), dist, goalAngle, heading, useEkf ? "EKF" : "cam",
                angleDiff, bias, seg, avoiding ? " [AVOID]" : "");

  if (abs(angleDiff) > nav.realignThreshold) {
    fsmInstruction[0] = TURN;
    fsmInstruction[1] = DegToArc(angleDiff);
    instructionList.push_back(fsmInstruction);
    fsmInstruction[0] = WAIT;
    fsmInstruction[1] = 300;
    instructionList.push_back(fsmInstruction);
  }

  if (seg > 10.0f) {
    fsmInstruction[0] = MOVE;
    fsmInstruction[1] = seg;
    instructionList.push_back(fsmInstruction);
    fsmInstruction[0] = WAIT;
    fsmInstruction[1] = 300;
    instructionList.push_back(fsmInstruction);
  }

  fsmInstruction[0] = REQUEST_POSITION;
  fsmInstruction[1] = 0;
  instructionList.push_back(fsmInstruction);
}

// ============================================================================
// FUNCIONES DE SENSORES Y HARDWARE
// ============================================================================

void SetupFrontSensor() {
  if (frontSensorInitialized)
    return;

  unsigned long now = millis();

  // CORRECCIÓN CLAVE: Permite la ejecución si es el primer intento
  // (lastFrontSensorAttempt == 0), o si han pasado 5 segundos desde el último
  // intento fallido.
  if (lastFrontSensorAttempt != 0 &&
      (now - lastFrontSensorAttempt < frontSensorRetryInterval)) {
    return;
  }

  lastFrontSensorAttempt = now;

  DebugSerialPrintln("Intentando inicializar APDS9960...");

  if (frontSensor.begin()) {
    frontSensorInitialized = true;
    ledCtrl.setOff();
    DebugSerialPrintln(" Sensor APDS-9960 inicializado correctamente");
  } else {
    DebugSerialPrintln(
        " Falló la inicialización del sensor APDS-9960. Reintentando...");
    ledCtrl.setBlink(255, 128, 0, maxBrightness, 500);
  }
}

void WiFiStatus() {
  static uint8_t lastStatus = 255; // Inicializar con valor inválido
  uint8_t currentStatus = WiFi.status();

  // Debug: mostrar cambios de estado
  if (currentStatus != lastStatus) {
    const char *statusStr[] = {
        "WL_IDLE_STATUS",     // 0
        "WL_NO_SSID_AVAIL",   // 1
        "WL_SCAN_COMPLETED",  // 2
        "WL_CONNECTED",       // 3
        "WL_CONNECT_FAILED",  // 4
        "WL_CONNECTION_LOST", // 5
        "WL_DISCONNECTED"     // 6
    };
    if (currentStatus <= 6) {
      DebugSerialPrintf("WiFi Status cambió: %s (%d)\n",
                        statusStr[currentStatus], currentStatus);
    }
    lastStatus = currentStatus;
  }

  if (WiFi.status() != WL_CONNECTED) {
    static bool wifiConnecting = false;
    static unsigned long lastWifiAttempt = 0;
    static int retryCount = 0;

    if (!wifiConnecting) {
      ConfigureHBridge(0, 0);
      DebugSerialPrintln("=== Iniciando conexión WiFi ===");
      DebugSerialPrintf("SSID: %s\n", ssid);
      DebugSerialPrintf("MAC: %s\n", WiFi.macAddress().c_str());
      DebugSerialPrintf("Hostname: %s\n", WiFi.getHostname());
      ledCtrl.setBlink(0, 255, 255, maxBrightness, 250);
      wifiConnecting = true;
      lastWifiAttempt = millis();
      retryCount = 0;
    }

    // Timeout de conexión: reintentar después de 10 segundos
    if (millis() - lastWifiAttempt > 10000) {
      retryCount++;
      DebugSerialPrintf("⚠ Timeout WiFi (intento #%d). Estado: %d\n",
                        retryCount, WiFi.status());

      // Después de 3 intentos, hacer un reset más agresivo
      if (retryCount >= 3) {
        DebugSerialPrintln(
            "🔴 Múltiples fallos. Reiniciando WiFi completamente...");
        WiFi.mode(WIFI_OFF);
        delay(500);
        WiFi.mode(WIFI_STA);
        String hostname = "AttaBot-" + String((uint32_t)ESP.getEfuseMac(), HEX);
        WiFi.setHostname(hostname.c_str());
        retryCount = 0;
      }

      WiFi.disconnect();
      delay(100);
      WiFi.begin(ssid, password);
      lastWifiAttempt = millis();
    }

    return;
  } else {
    static bool firstConnect = true;
    if (firstConnect) {
      DebugSerialPrintln("=== ✓ WiFi CONECTADO ===");
      DebugSerialPrintf("IP: %s\n", WiFi.localIP().toString().c_str());
      DebugSerialPrintf("Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
      DebugSerialPrintf("Subnet: %s\n", WiFi.subnetMask().toString().c_str());
      DebugSerialPrintf("DNS: %s\n", WiFi.dnsIP().toString().c_str());
      DebugSerialPrintf("MAC: %s\n", WiFi.macAddress().c_str());
      DebugSerialPrintf("Hostname: %s\n", WiFi.getHostname());
      DebugSerialPrintf("RSSI: %d dBm\n", WiFi.RSSI());
      DebugSerialPrintln("=======================");
      firstConnect = false;
    }
    ledCtrl.setOff();
  }
}

void ReadSensors() {
  if (((millis() - movement.previousMillis) <= samplingTime - 2) ||
      isLateralCycleActive || isCentralCycleActive || (debugUdp == 3)) {
    currentMicros = micros();
    if (currentMicros - previousMicros >= observationTime) {
      previousMicros = currentMicros;
      cycleCounter = (cycleCounter + 1) % numberOfCycles;

      isLateralCycleActive = (lateralCycle == cycleCounter);

      if (isLateralCycleActive != lateralSensorsEnabled) {
        lateralSensorsEnabled = isLateralCycleActive;
        digitalWrite(enableLeftInfraredSensor, lateralSensorsEnabled);
        digitalWrite(enableRightInfraredSensor, lateralSensorsEnabled);

        if (lateralSensorsEnabled) {
          noInterrupts();
          leftObsStartTime = micros();
          rightObsStartTime = micros();
          interrupts();
        }
      }

      isCentralCycleActive = (centralCycle == cycleCounter);

      // GUARDIA 1: Solo intenta habilitar la proximidad si el sensor ya está
      // inicializado.
      if (frontSensorInitialized) {
        frontSensor.enableProximity(isCentralCycleActive);
      }
    }

    if (isLateralCycleActive) {
      noInterrupts();
      unsigned long leftTime = leftObsStartTime;
      unsigned long rightTime = rightObsStartTime;
      interrupts();

      unsigned long now = micros();
      obstacles.leftObstacle = !maskLeftIR &&
                               (digitalRead(leftInfraredSensor) == LOW) &&
                               ((now - leftTime) >= minObstacleTime);
      obstacles.rightObstacle = !maskRightIR &&
                                (digitalRead(rightInfraredSensor) == LOW) &&
                                ((now - rightTime) >= minObstacleTime);
    }

    if (isCentralCycleActive) {
      // GUARDIA 2: Solo intenta leer la proximidad si el sensor ya está
      // inicializado.
      if (frontSensorInitialized) {
        centralDistance = frontSensor.readProximity();
        if (centralDistance > centralIRThreshold && !maskCentralIR) {
          obstacles.centralObstacle =
              (micros() - centralObsStartTime) >= minObstacleTime / 2;
        } else {
          centralObsStartTime = micros();
          obstacles.centralObstacle = false;
        }
      } else {
        // Si no está inicializado, asumimos que no hay obstáculo
        obstacles.centralObstacle = false;
      }
    }
  }

  if (debugUdp == 3) {
    if (obstacles.HasAnyObstacle()) {
      ledCtrl.setSolid(255, 128, 0, maxBrightness);
    } else {
      ledCtrl.setOff();
    }
  } else {
    if ((digitalRead(batteryStatus) == LOW) &&
        ((millis() - lowBatteryTime) >= minLowBatteryTime)) {
      ledCtrl.setSolid(255, 255, 0, 255);
    }
  }

  if (isEvading && (millis() - evasionStartTime > evasionCooldown)) {
    isEvading = false;
    MessageDebugf("DEBUG: -1, ID: %s, Cooldown de evasión completado",
                  robotID.c_str());
  }
}

// ============================================================================
// FUNCIONES DE CONTROL DE MOTORES
// ============================================================================

void ResetPID() {
  leftControl.Reset();
  rightControl.Reset();
  debugCounter = 0;
  movement.Reset();
}

// EKF pasivo: propaga con gyro (Δθ) y encoders (Δd) en cada lectura de IMU.
// No afecta la navegación todavía — es el observador a validar contra ArUco.
void EkfTick() {
  static bool yawInit = false;
  static float prevYaw = 0;
  static float prevAvgPulses = 0;

  if (!yawInit) {
    prevYaw = yaw;
    yawInit = true;
    return;
  }

  float dYaw = yaw - prevYaw;
  if (dYaw >  180.0f) dYaw -= 360.0f;
  if (dYaw < -180.0f) dYaw += 360.0f;
  prevYaw = yaw;

  // Δ distancia solo cuando las ruedas avanzan de verdad (en TURN los
  // contadores suben pero el desplazamiento neto es ~0)
  float avg = (movement.pastLeftPulseCount + movement.pastRightPulseCount) / 2.0f;
  float d = 0.0f;
  if (state == MOVE || state == REVERSE) {
    float dAvg = avg - prevAvgPulses;
    if (dAvg < 0) dAvg = avg;   // ResetPID reinició los contadores
    d = dAvg * millimetersPerPulse * (state == REVERSE ? -1.0f : 1.0f);
  }
  prevAvgPulses = avg;

  ekf.Predict(d, dYaw * yawScale);
}

// Clasifica una lectura RGBC del APDS9960 contra un color objetivo.
// Umbrales de primera pasada — calibrar en lab con COLOR_READ.
bool MatchColor(const char *target, uint16_t r, uint16_t g, uint16_t b,
                uint16_t c) {
  if (c < 10) return false;   // muy oscuro / sin señal útil
  if (strcmp(target, "rojo") == 0)  return r > g * 3 / 2 && r > b * 3 / 2;
  if (strcmp(target, "verde") == 0) return g > r * 3 / 2 && g > b * 3 / 2;
  if (strcmp(target, "azul") == 0)  return b > r * 3 / 2 && b > g * 3 / 2;
  return false;
}

// Candidato descartado (o inalcanzable): evadir con el patrón estándar de
// retroceso y re-armar la patrulla RANDOM_WALK de la búsqueda.
void SearchEvadeAndResume() {
  instructionList.clear();
  fsmInstruction[0] = REVERSE;
  fsmInstruction[1] = reverseDistance * 2;
  instructionList.push_back(fsmInstruction);
  fsmInstruction[0] = TURN;
  fsmInstruction[1] = DegToArc(random(2) ? 60 : -60);
  instructionList.push_back(fsmInstruction);
  fsmInstruction[0] = MOVE;
  fsmInstruction[1] = 150;
  instructionList.push_back(fsmInstruction);
  fsmInstruction[0] = RANDOM_WALK;
  fsmInstruction[1] = 600000;
  instructionList.push_back(fsmInstruction);
  obstacles.Clear();
  obstacleDetected = false;
  isEvading = false;
  state = READ_INSTRUCTION;
}

void ConfigureHBridge(int leftWheelPWM, int rightWheelPWM) {
  if (leftWheelPWM >= 0) {
    ledcWrite(leftMotorBackward, 0);
    ledcWrite(leftMotorForward, leftWheelPWM);
  } else {
    ledcWrite(leftMotorForward, 0);
    ledcWrite(leftMotorBackward, abs(leftWheelPWM));
  }

  if (rightWheelPWM >= 0) {
    ledcWrite(rightMotorBackward, 0);
    ledcWrite(rightMotorForward, rightWheelPWM);
  } else {
    ledcWrite(rightMotorForward, 0);
    ledcWrite(rightMotorBackward, abs(rightWheelPWM));
  }
}

float DesiredSpeed(float distance, float wheelDistance) {
  float remainingDistance = distance - wheelDistance;
  float desiredSpeed = maxSpeed;
  if (abs(remainingDistance) < speedReductionThreshold) {
    desiredSpeed = map(abs(remainingDistance), 0, speedReductionThreshold,
                       minSpeed, maxSpeed);
  }

  return (remainingDistance < 0) ? -desiredSpeed : desiredSpeed;
}

bool IsStationary(float currentLeftSpeed, float currentRightSpeed,
                  float leftWheelDistance, float rightWheelDistance) {
  bool speedsAtZero =
      (static_cast<int>(abs(currentLeftSpeed) + abs(currentRightSpeed)) == 0);
  bool wheelsHaveMoved =
      (static_cast<int>(abs(leftWheelDistance) + abs(rightWheelDistance)) != 0);
  if (!speedsAtZero) {
    movement.steadyStatePreviousMillis = millis();
  } else if ((millis() - movement.steadyStatePreviousMillis >=
              SteadyStateTime) &&
             wheelsHaveMoved) {
    return true;
  }

  return false;
}

bool MoveDistanceByWheel(float leftDistance, float rightDistance) {
  currentMillis = millis();
  millisDifference = currentMillis - movement.previousMillis;
  if (millisDifference < samplingTime) {
    return false;
  }

  movement.previousMillis = currentMillis;

  movement.currentLeftSpeed =
      ((movement.leftPulseCount - movement.pastLeftPulseCount) *
       millimetersPerPulse) /
      samplingTimeS;
  movement.pastLeftPulseCount = movement.leftPulseCount;
  movement.currentRightSpeed =
      ((movement.rightPulseCount - movement.pastRightPulseCount) *
       millimetersPerPulse) /
      samplingTimeS;
  movement.pastRightPulseCount = movement.rightPulseCount;

  float leftWheelDistance = movement.pastLeftPulseCount * millimetersPerPulse;
  float desiredLeftSpeed = DesiredSpeed(leftDistance, leftWheelDistance);
  int leftWheelPWM =
      leftControl.Calculate(desiredLeftSpeed, movement.currentLeftSpeed);

  float rightWheelDistance = movement.pastRightPulseCount * millimetersPerPulse;
  float desiredRightSpeed = DesiredSpeed(rightDistance, rightWheelDistance);
  int rightWheelPWM =
      rightControl.Calculate(desiredRightSpeed, movement.currentRightSpeed);

  ConfigureHBridge(leftWheelPWM, rightWheelPWM);
  bool IsMoveFinished =
      ((abs(leftWheelDistance) + distanceOffset) >= abs(leftDistance)) &&
      ((abs(rightWheelDistance) + distanceOffset) >= abs(rightDistance));
  IsMoveFinished =
      IsMoveFinished ||
      IsStationary(movement.currentLeftSpeed, movement.currentRightSpeed,
                   leftWheelDistance, rightWheelDistance);

  if (debugUdp >= 2) {
    MessageDebugf(debugMessage, debugCounter, robotID.c_str(), direction,
                  movement.leftPulseCount, movement.rightPulseCount,
                  movement.currentLeftSpeed, movement.currentRightSpeed,
                  leftWheelPWM, rightWheelPWM, leftControl.error,
                  rightControl.error, leftControl.sumError,
                  rightControl.sumError, leftWheelDistance, rightWheelDistance,
                  millisDifference);
  }

  return IsMoveFinished;
}

// ============================================================================
// FUNCIONES DE COMUNICACIÓN
// ============================================================================

void SendMessage(IPAddress host, const char *message) {
  udp.beginPacket(host, localPort);
  udp.write(reinterpret_cast<const uint8_t *>(message), strlen(message));
  udp.endPacket();
}

void MessageDebugf(const char *format, ...) {
  char buffer[200];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  DebugSerialPrintln(buffer);
  if (debugUdp != 0) {
    SendMessage(robots["Base"], buffer);
  }

  debugCounter++;
}

void SendPose() {
  obstacles.robotDetected = false;
  const char *message = "CHECK_OBSTACLE|%d|%.1f|%.1f|%.1f";
  char buffer[40];
  snprintf(buffer, sizeof(buffer), message, obstacles.obstacleSensors,
           robotPose.x, robotPose.y, robotPose.angle);
  SendMessage(robots["Broadcast"], buffer);
  movement.previousMillis = millis();
}

std::array<String, 6> SeparateCommand(const String &command, char delimiter) {
  std::array<String, 6> results;
  int startIndex = 0;
  int endIndex;
  int count = 0;

  while (count < results.size()) {
    endIndex = command.indexOf(delimiter, startIndex);
    if (endIndex == -1) {
      results[count] = command.substring(startIndex);
      break;
    } else {
      results[count] = command.substring(startIndex, endIndex);
      startIndex = endIndex + 1;
    }
    count++;
  }

  return results;
}

// ============================================================================
// DISPERSIÓN DE ENJAMBRE — port 1:1 de maybe_disperse_hop() del controller de
// sim validado en Webots. Se llama al recibir NEIGHBOR_POSITIONS (1 Hz).
// Constantes de arena = FOV útil del lab (2.4 x 1.55 m) con inset de 350mm para
// que los saltos no apunten a la pared. Ajustar si cambia el montaje de cámara.
// ============================================================================
// Límites de los saltos de dispersión: la arena en curso (NAV_CONFIG|ARENA, que
// la Base manda al registrar) menos un inset para que ningún salto apunte a la
// pared. Antes eran cuatro #define fijos al montaje de 2400×1550 del lab, así
// que en cualquier otro escenario los robots se dispersaban contra un borde
// imaginario.
#define DISP_INSET 350.0f
#define DISP_ARENA_XMIN DISP_INSET
#define DISP_ARENA_XMAX (arenaWidthMm - DISP_INSET)
#define DISP_ARENA_YMIN DISP_INSET
#define DISP_ARENA_YMAX (arenaHeightMm - DISP_INSET)

// ============================================================================
// MEET — reparto de los slots del anillo por CERCANÍA, resuelto localmente.
//
// Es el mismo greedy que hacía la Base (par (distancia, robot, slot) más chico
// primero, el que llega se queda), pero corrido por cada robot. Sin él el slot
// saldría del orden de id y dos robots se cruzarían el camino para ocuparlo.
//
// Que dé lo mismo en todos NO es casualidad: las poses salen del ÚLTIMO
// NEIGHBOR_POSITIONS, un mensaje idéntico para todos, y el orden por id fija el
// desempate. Mismo insumo + mismo criterio = misma repartición, sin negociar y
// sin que la Base decida nada. Por eso la pose propia se toma de ese barrido y
// no de robotPose, que cada robot refresca en un instante distinto.
// ============================================================================
int MeetSlotIndex(float tx, float ty, float ring, int n) {
  const int MAXN = DisperseState::MAX_NEIGHBORS + 1;
  int   id[MAXN];
  float px[MAXN], py[MAXN];
  int   count = 0;

  for (int i = 0; i < disperse.nCount && count < MAXN; i++) {
    id[count] = disperse.nId[i].toInt();
    px[count] = disperse.nX[i];
    py[count] = disperse.nY[i];
    count++;
  }
  int myId = robotID.toInt();
  if (count < MAXN) {
    id[count] = myId;
    // Único caso en que el insumo difiere: la cámara no vio a este robot en ese
    // barrido. Ahí no queda otra que la pose propia.
    px[count] = disperse.selfSeen ? disperse.selfX : robotPose.x;
    py[count] = disperse.selfSeen ? disperse.selfY : robotPose.y;
    count++;
  }
  if (count > n) count = n;
  if (count <= 1) return 0;

  for (int a = 0; a < count - 1; a++) {         // orden por id
    for (int b = a + 1; b < count; b++) {
      if (id[b] < id[a]) {
        int   ti = id[a]; id[a] = id[b]; id[b] = ti;
        float t  = px[a]; px[a] = px[b]; px[b] = t;
        t = py[a];        py[a] = py[b]; py[b] = t;
      }
    }
  }

  float sx[MAXN], sy[MAXN];                     // posición de cada slot
  for (int s = 0; s < count; s++) {
    float ang = 2.0f * PI * s / count;
    sx[s] = tx + ring * cosf(ang);
    sy[s] = ty + ring * sinf(ang);
  }

  bool robotDone[MAXN] = {false};
  bool slotDone[MAXN]  = {false};
  int  asg[MAXN];                               // robot → slot
  for (int k = 0; k < count; k++) asg[k] = k;
  for (int k = 0; k < count; k++) {             // greedy: el par más corto gana
    float best = 1.0e12f;
    int   br = -1, bs = -1;
    for (int r = 0; r < count; r++) {
      if (robotDone[r]) continue;
      for (int s = 0; s < count; s++) {
        if (slotDone[s]) continue;
        float d = CalculateDistance(px[r], py[r], sx[s], sy[s]);
        if (d < best) { best = d; br = r; bs = s; }
      }
    }
    if (br < 0) break;
    robotDone[br] = true;
    slotDone[bs]  = true;
    asg[br] = bs;
  }

  // 2-opt: intercambiar los slots de dos robots mientras eso acorte la suma de
  // recorridos. Es lo que garantiza que NO haya cruces — si dos caminos se
  // cruzaran, intercambiarlos acortaría la suma por desigualdad triangular, así
  // que el bucle no puede terminar con un cruce. El greedy solo no lo asegura.
  // El epsilon evita que dos slots empatados se intercambien para siempre.
  bool improved = true;
  for (int pass = 0; pass < 2 * MAXN && improved; pass++) {
    improved = false;
    for (int a = 0; a < count - 1 && !improved; a++) {
      for (int b = a + 1; b < count && !improved; b++) {
        float now = CalculateDistance(px[a], py[a], sx[asg[a]], sy[asg[a]]) +
                    CalculateDistance(px[b], py[b], sx[asg[b]], sy[asg[b]]);
        float swp = CalculateDistance(px[a], py[a], sx[asg[b]], sy[asg[b]]) +
                    CalculateDistance(px[b], py[b], sx[asg[a]], sy[asg[a]]);
        if (swp < now - 0.01f) {
          int t = asg[a];  asg[a] = asg[b];  asg[b] = t;
          improved = true;
        }
      }
    }
  }

  for (int r = 0; r < count; r++) {
    if (id[r] == myId) return asg[r];
  }
  return 0;
}


void MaybeDisperseHop() {
  // Guardas: dispersión activa, robot OCIOSO (sin nav ni instrucciones), con
  // vecinos. Un robot ocupado espera al siguiente tick para reevaluar.
  if (!disperse.IsActive() || nav.isActive || nav.pendingInit ||
      !instructionList.empty() || disperse.nCount == 0) {
    return;
  }
  float x = robotPose.x, y = robotPose.y;

  // Vecino más cercano
  float dmin = 1e12f;
  for (int i = 0; i < disperse.nCount; i++) {
    float d = CalculateDistance(x, y, disperse.nX[i], disperse.nY[i]);
    if (d < dmin) dmin = d;
  }
  float dminS = disperse.SmoothDmin(dmin);

  // Histéresis de 80mm (~2σ del jitter ArUco): un robot satisfecho no se
  // des-satisface por ruido de medición.
  float settleAt = disperse.target - (disperse.settled ? 80.0f : 0.0f);
  if (dminS >= settleAt) {
    if (!disperse.settled) {
      disperse.settled = true;
      MessageDebugf("DEBUG: -1, ID: %s, dispersión lograda — vecino más cercano a %.0fmm",
                    robotID.c_str(), dminS);
    }
    return;
  }
  disperse.settled = false;

  // Turno secuencial: solo salta el de MENOR id entre los que están demasiado
  // cerca; el resto espera quieto (evita la tormenta de evasiones IR mutuas).
  // Margen de 100mm: un vecino en la banda de jitter no bloquea. Si igual
  // quedamos bloqueados ~10 rondas, saltar de todos modos (anti-deadlock).
  int  myId = robotID.toInt();
  bool blockedByLower = false;
  for (int i = 0; i < disperse.nCount; i++) {
    float d = CalculateDistance(x, y, disperse.nX[i], disperse.nY[i]);
    if (d < disperse.target - 100.0f && disperse.nId[i].toInt() < myId) {
      blockedByLower = true;
    }
  }
  if (blockedByLower) {
    disperse.blocked++;
    if (disperse.blocked < 10) return;
  }
  disperse.blocked = 0;

  // Suma de repulsión 1/d²
  float vx = 0, vy = 0;
  for (int i = 0; i < disperse.nCount; i++) {
    float d = CalculateDistance(x, y, disperse.nX[i], disperse.nY[i]);
    if (d < 1.0f) d = 1.0f;
    vx += (x - disperse.nX[i]) / (d * d);
    vy += (y - disperse.nY[i]) / (d * d);
  }
  float norm = sqrt(vx * vx + vy * vy);
  if (norm < 1e-9f) {                    // sobre el vecino: dirección aleatoria
    float ang = random(0, 62832) / 10000.0f;   // ~[0, 2π)
    vx = cos(ang); vy = sin(ang); norm = 1.0f;
  }

  // Candidatos: repulsión directa y sus dos rotaciones ±90° (escape de esquina).
  // Score = separación del vecino más cercano + bono de holgura a la pared: un
  // objetivo en la esquina maximiza la separación pero deja al robot raspando
  // dos muros (348 evasiones IR en sim → 2 con el bono).
  float cand[3][2] = { { vx, vy }, { -vy, vx }, { vy, -vx } };
  float bestX = 0, bestY = 0, bestScore = -1e12f;
  bool  haveBest = false;
  for (int c = 0; c < 3; c++) {
    float gx = constrain(x + cand[c][0] / norm * 450.0f,
                         DISP_ARENA_XMIN, DISP_ARENA_XMAX);
    float gy = constrain(y + cand[c][1] / norm * 450.0f,
                         DISP_ARENA_YMIN, DISP_ARENA_YMAX);
    if (CalculateDistance(x, y, gx, gy) < 100.0f) continue;
    float nd = 1e12f;
    for (int i = 0; i < disperse.nCount; i++) {
      float d = CalculateDistance(gx, gy, disperse.nX[i], disperse.nY[i]);
      if (d < nd) nd = d;
    }
    float wcx = min(gx - DISP_ARENA_XMIN, DISP_ARENA_XMAX - gx);
    float wcy = min(gy - DISP_ARENA_YMIN, DISP_ARENA_YMAX - gy);
    float wallClear = min(wcx, wcy);
    float score = nd + 0.5f * min(wallClear, 300.0f);
    if (score > bestScore) {
      bestScore = score;  bestX = gx;  bestY = gy;  haveBest = true;
    }
  }
  if (!haveBest) return;

  // Salto: iniciar navegación reactiva al mejor candidato (como GT/CONGREGATION)
  nav.goalX       = bestX;
  nav.goalY       = bestY;
  nav.pendingInit = true;
  fsmInstruction[0] = REQUEST_POSITION;
  fsmInstruction[1] = 0;
  instructionList.push_back(fsmInstruction);
  MessageDebugf("DEBUG: -1, ID: %s, dispersión: hop → (%.0f,%.0f) (vecino a %.0fmm)",
                robotID.c_str(), bestX, bestY, dmin);
}

bool IsRobotObstacle(float x2, float y2, float angle, int sensors, String id) {
  float deltaX = x2 - robotPose.x;
  float deltaY = y2 - robotPose.y;

  float distanceBetweenRobots = sqrt(deltaX * deltaX + deltaY * deltaY);
  if (distanceBetweenRobots > robotDistanceMargin) {
    return false;
  }

  float angleBetweenRobots = atan2f(deltaY, deltaX) * RAD_TO_DEG + 180;
  float angleDifference = angleBetweenRobots - angle;
https://meet.google.com/fdh-njby-vde?hs=224
  if (angleDifference > 180) {
    angleDifference -= 360;
  } else if (angleDifference < -180) {
    angleDifference += 360;
  }

  MessageDebugf("DEBUG: -1, ID: %s, From ID: %s, Distancia: %.1f, Angulo: "
                "%.1f, DifAngulo: %.1f, sensors: %d",
                robotID.c_str(), id.c_str(), distanceBetweenRobots,
                angleBetweenRobots, angleDifference, sensors);

  if (abs(angleDifference) <= maxRobotAngleMargin) {
    if (sensors == 0b100 && angleDifference <= 0) {
      return true;
    } else if (sensors == 0b001 && angleDifference >= 0) {
      return true;
    } else if (sensors != 0b100 && sensors != 0b001) {
      return true;
    }
  }

  return false;
}

// ============================================================================
// FUNCIÓN DE LECTURA DE PAQUETES UDP (REFACTORIZADA)
// ============================================================================

void ReadUdpPackets() {
  int packetBytes = udp.parsePacket();
  if (!packetBytes) {
    return;
  }

  int len = udp.read(receivedPacket, sizeof(receivedPacket) - 1);
  if (len > 0) {
    receivedPacket[len] = 0;
  }
  DebugSerialPrintf("Recibidos %d bytes de %s: %s\n", packetBytes,
                    udp.remoteIP().toString().c_str(), receivedPacket);

  String command(receivedPacket);
  std::array<String, 6> arguments = SeparateCommand(command, '|');
  command = arguments[0];

  // CONFIG
  if (command == "CONFIG") {
    if (arguments[1] == "START") {
      robots["Base"] = udp.remoteIP();
      IPAddress ipAddress;
      ipAddress.fromString(arguments[2]);
      robots["Broadcast"] = ipAddress;
      // Reset completo: limpia cualquier navegación activa de sesiones anteriores.
      // Incluye congregación/dispersión: si no, un robot que quedó de LÍDER en la
      // prueba anterior sigue difundiendo LEADER_POSITION a 4Hz en la nueva sesión
      // (rol persiste en RAM y CONFIG|START no lo limpiaba) → inunda y estorba.
      instructionList.clear();
      isEvading = false;
      obstacles.Clear();
      congregation.Reset();
      disperse.Reset();
      nav.Reset();
      ekf.Reset();   // sin esto la deriva del EKF sobrevive entre sesiones
      state = STOP;
      SendMessage(robots["Base"], "CONFIG|RECEIVED");
      debugUdp = 0;
      countMessages = 0;

    } else if (arguments[1] == "SAVE") {
      robots[arguments[2]] = udp.remoteIP();

    } else if (arguments[1] == "ROBOTS") {
      for (const auto &pair : robots) {
        DebugSerialPrintf("Nombre: %s, IP: %s\n", pair.first.c_str(),
                          pair.second.toString().c_str());
      }

    } else if (arguments[1] == "DEBUG") {
      debugUdp = arguments[2].toInt();
      SendMessage(robots["Base"], debugUdp != 0 ? "Modo debug activado"
                                                : "Modo debug desactivado");

    } else {
      robotID = arguments[1];
      char buffer[50];
      snprintf(buffer, sizeof(buffer), "CONFIG|SAVE|%s", robotID.c_str());
      SendMessage(robots["Broadcast"], buffer);
    }
  }

  // INSTRUCCIONES DE MOVIMIENTO
  else if (command == "MOVE" || command == "TURN" || command == "WAIT" ||
           command == "RANDOMW" || command == "MESSAGE_BASE") {
    short value = arguments[1].toInt();

    if (command == "MOVE") {
      fsmInstruction[0] = MOVE;
      fsmInstruction[1] = value;
    } else if (command == "TURN") {
      fsmInstruction[0] = TURN;
      fsmInstruction[1] = DegToArc(value);
    } else if (command == "WAIT") {
      fsmInstruction[0] = WAIT;
      fsmInstruction[1] = value * 1000;
    } else if (command == "RANDOMW") {
      fsmInstruction[0] = RANDOM_WALK;
      fsmInstruction[1] = value * 1000;
    } else if (command == "MESSAGE_BASE") {
      fsmInstruction[0] = MESSAGE_BASE;
      fsmInstruction[1] = arguments[1].toInt();
    }
    instructionList.push_back(fsmInstruction);
  }

  // RESET
  else if (command == "RESET") {
    ESP.restart();
  }

  // PID
  else if (command == "PID") {
    float kp = arguments[1].toFloat();
    float ki = arguments[2].toFloat();
    float kd = arguments[3].toFloat();

    leftControl.pidConst.kp  = kp;
    leftControl.pidConst.ki  = ki;
    leftControl.pidConst.kd  = kd;
    rightControl.pidConst.kp = kp;
    rightControl.pidConst.ki = ki;
    rightControl.pidConst.kd = kd;

    char pidBuf[100];
    if (arguments[4] == "SAVE") {
      SavePID(kp, ki, kd);
      snprintf(pidBuf, sizeof(pidBuf),
               "PID modificado y GUARDADO: Kp=%.2f Ki=%.2f Kd=%.3f", kp, ki, kd);
    } else {
      snprintf(pidBuf, sizeof(pidBuf),
               "PID modificado temporalmente: Kp=%.2f Ki=%.2f Kd=%.3f", kp, ki, kd);
    }
    SendMessage(robots["Base"], pidBuf);
  }

  // KFPID
  else if (command == "KFPID") {
    float R = arguments[1].toFloat();
    float H = arguments[2].toFloat();
    float Q = arguments[3].toFloat();

    leftControl.kf.R = R;
    leftControl.kf.H = H;
    leftControl.kf.Q = Q;
    rightControl.kf.R = R;
    rightControl.kf.H = H;
    rightControl.kf.Q = Q;
    SendMessage(robots["Base"], "Filtro de kalman modificado");
  }

  // POSE
  else if (command == "POSE") {
    float newX = arguments[1].toFloat();
    float newY = arguments[2].toFloat();
    float newAngle = arguments[3].toFloat();

    if (robotPose.x != 0 || robotPose.y != 0) {
      float deltaX = newX - robotPose.x;
      float deltaY = newY - robotPose.y;
      float distanceJump = sqrt(deltaX * deltaX + deltaY * deltaY);

      float angleDiff = NormalizeAngle(newAngle - robotPose.angle);

      if (distanceJump > max_pose_jump || abs(angleDiff) > max_angle_jump) {
        // Saltos consistentes = la pose interna quedó obsoleta (robot movido a
        // mano, cambio de origen). Tras varios rechazos seguidos, re-sincronizar
        // con la cámara en vez de quedar atrapado ignorando para siempre.
        poseJumpRejections++;
        if (poseJumpRejections < max_pose_jump_rejections) {
          char buffer[150];
          snprintf(buffer, sizeof(buffer),
                   "WARNING: Salto brusco detectado. ΔPos=%.1fmm, ΔAng=%.1f°. "
                   "Ignorando actualización (%d/%d).",
                   distanceJump, angleDiff, poseJumpRejections,
                   max_pose_jump_rejections);
          MessageDebugf("DEBUG: -1, ID: %s, %s", robotID.c_str(), buffer);
          return;
        }
        MessageDebugf("DEBUG: -1, ID: %s, WARNING: %d saltos consecutivos — "
                      "re-sincronizando pose con la cámara.",
                      robotID.c_str(), poseJumpRejections);
      }
    }

    poseJumpRejections = 0;
    robotPose.x = newX;
    robotPose.y = newY;
    robotPose.angle = newAngle;

    // Decentralizado: si este robot es el líder de congregación, difunde su
    // propia pose a los peers por WiFi (la Base es solo sensor de localización).
    // Un líder quieto solo hacía REQUEST_POSITION una vez → los seguidores se
    // quedaban sin actualizaciones para progresar staging→parking ni seguirlo.
    // POSE llega cada frame (~15-20Hz); se throttlea a ~4Hz para no saturar.
    if (congregation.isLeader && congregation.leaderID != "-1" &&
        (millis() - lastLeaderCast) >= 250) {
      lastLeaderCast = millis();
      float lx, ly, lang;
      SmoothLeaderPose(robotPose.x, robotPose.y, robotPose.angle, &lx, &ly, &lang);
      char buffer[64];
      snprintf(buffer, sizeof(buffer), "LEADER_POSITION|%s|%.1f|%.1f|%.1f",
               robotID.c_str(), lx, ly, lang);
      if (robots.find("Broadcast") != robots.end()) {
        SendMessage(robots["Broadcast"], buffer);
        delayMicroseconds(500);
        SendMessage(robots["Broadcast"], buffer);
      }
    }
  }

  // SETPPR
  else if (command == "SETPPR") {
    float newPPR = arguments[1].toFloat();
    bool permanent = (arguments[2] == "SAVE");

    if (newPPR > 100 && newPPR < 5000) {
      pulsesPerRev = newPPR;
      updateMillimetersPerPulse();

      char buffer[100];
      if (permanent) {
        SavePPR(newPPR);
        snprintf(buffer, sizeof(buffer),
                 "PPR modificado y GUARDADO: %.2f (Robot ID: %s)", newPPR,
                 robotID.c_str());
      } else {
        snprintf(buffer, sizeof(buffer),
                 "PPR modificado temporalmente: %.2f (Robot ID: %s)", newPPR,
                 robotID.c_str());
      }
      SendMessage(robots["Base"], buffer);

    } else {
      SendMessage(robots["Base"], "Error: PPR debe estar entre 100-5000");
    }
  }

  // GETPPR
  else if (command == "GETPPR") {
    char buffer[100];
    snprintf(buffer, sizeof(buffer),
             "Robot %s - PPR actual: %.2f, Chip ID: %04X%08X", robotID.c_str(),
             pulsesPerRev, (uint16_t)(ESP.getEfuseMac() >> 32),
             (uint32_t)ESP.getEfuseMac());
    SendMessage(robots["Base"], buffer);
  }

  // CHECK_OBSTACLE
  else if (command == "CHECK_OBSTACLE") {
    int sensors = arguments[1].toInt();
    float x = arguments[2].toFloat();
    float y = arguments[3].toFloat();
    float angle = arguments[4].toFloat();

    String id = "-1";
    for (const auto &pair : robots) {
      if (pair.second.toString() == udp.remoteIP().toString()) {
        id = pair.first;
        break;
      }
    }

    if (IsRobotObstacle(x, y, angle, sensors, id)) {
      char buffer[50];
      snprintf(buffer, sizeof(buffer), "OBSTACLE_DETECTED|%s", robotID.c_str());
      delayMicroseconds(800);
      SendMessage(robots[id], buffer);
      SendMessage(robots[id], buffer);
    }
  }

  // OBSTACLE_DETECTED
  else if (command == "OBSTACLE_DETECTED") {
    obstacles.fromRobotID = arguments[1];
    obstacles.robotDetected = true;
  }

  // COUNT_MESSAGE
  else if (command == "COUNT_MESSAGE") {
    countMessages++;
  }

  // SEND_COUNT_MESSAGE
  else if (command == "SEND_COUNT_MESSAGE") {
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "Robot ID: %s, Total messages: %d",
             robotID.c_str(), countMessages);
    SendMessage(robots["Base"], buffer);
  }

  // MEET|x|y — congregarse alrededor de un PUNTO, sin robot líder. Es el
  // experimento de topología: los robots cruzan la barrera y se agrupan en el
  // otro extremo aunque allí no haya nadie.
  //
  // No es un GT: con GT los N robots apuntarían al MISMO punto y terminarían
  // amontonados y evadiéndose entre sí. Acá cada uno toma su slot del anillo
  // (mismo SafeRingSlotAngle de siempre), así que llegan repartidos.
  //
  // Va por BROADCAST literal: el índice de slot y el tamaño del enjambre NO los
  // reparte la Base, los saca cada robot de la lista de vecinos que ya recibe
  // por NEIGHBOR_POSITIONS (1 Hz). Todos ven la misma lista, así que todos
  // calculan la misma repartición sin negociar y ninguno pisa el slot de otro.
  else if (command == "MEET") {
    congregation.leaderID = "VIRTUAL";
    congregation.isLeader = false;
    congregation.hasGlobalTarget = true;
    congregation.globalTargetX = arguments[1].toFloat();
    congregation.globalTargetY = arguments[2].toFloat();

    // n sale de los vecinos que este robot conoce; el anillo, de la misma
    // fórmula que usaba la Base (250mm de arco por robot) para que con muchos
    // robots no queden los slots pegados.
    int n = disperse.nCount + 1;
    if (n > DisperseState::MAX_NEIGHBORS + 1) n = DisperseState::MAX_NEIGHBORS + 1;
    float ring = max(300.0f, n * 250.0f / (2.0f * PI));
    congregation.parkingDist    = ring;
    congregation.totalFollowers = n;
    congregation.followerIndex  = MeetSlotIndex(congregation.globalTargetX,
                                                congregation.globalTargetY,
                                                ring, n);
    congregation.positionReceived = false;
    congregation.stagingDone = false;
    congregation.slotAngleSet = false;
    congregation.poseFresh = false;
    nav.Reset();
    ekf.Reset();
    instructionList.clear();

    MessageDebugf("DEBUG: -1, ID: %s, MEET en (%.0f,%.0f), slot %d/%d",
                  robotID.c_str(), congregation.globalTargetX,
                  congregation.globalTargetY, congregation.followerIndex,
                  congregation.totalFollowers);

    fsmInstruction[0] = WAIT;
    fsmInstruction[1] = robotID.toInt() * 200;   // arranque escalonado
    instructionList.push_back(fsmInstruction);
    fsmInstruction[0] = REQUEST_POSITION;
    fsmInstruction[1] = 0;
    instructionList.push_back(fsmInstruction);
  }

  else if (command == "CONGREGATION") {
    congregation.leaderID = arguments[1];
    congregation.isLeader = (congregation.leaderID == robotID);
    congregation.positionReceived = false;
    congregation.hasGlobalTarget = false;
    congregation.stagingDone = false;
    congregation.slotAngleSet = false;
    congregation.poseFresh = false;   // el slot espera una pose de cámara nueva
    congregation.followerIndex  = arguments[2].toInt();
    congregation.totalFollowers = (arguments[3] != "") ? arguments[3].toInt() : 1;

    nav.Reset();
    instructionList.clear();

    MessageDebugf("DEBUG: -1, ID: %s, Congregación iniciada. Líder: %s, slot: %d/%d",
                  robotID.c_str(), congregation.leaderID.c_str(),
                  congregation.followerIndex, congregation.totalFollowers);

    int delay = robotID.toInt() * 200;
    fsmInstruction[0] = WAIT;
    fsmInstruction[1] = delay;
    instructionList.push_back(fsmInstruction);

    fsmInstruction[0] = REQUEST_POSITION;
    fsmInstruction[1] = 0;
    instructionList.push_back(fsmInstruction);
  }

  // FORMATION — congregación con forma: linea/cuna/circulo (+ eje opcional)
  // FORMATION|<forma>|<líderID>|<idx>|<n>|[axis°]. Mismo flujo que CONGREGATION;
  // solo cambia la fórmula del slot (ver LEADER_POSITION). circulo == congregación.
  else if (command == "FORMATION") {
    congregation.formationShape = arguments[1];
    congregation.leaderID = arguments[2];
    congregation.isLeader = (congregation.leaderID == robotID);
    congregation.positionReceived = false;
    congregation.hasGlobalTarget = false;
    congregation.stagingDone = false;
    congregation.slotAngleSet = false;
    congregation.followerIndex  = arguments[3].toInt();
    congregation.totalFollowers = (arguments[4] != "") ? arguments[4].toInt() : 1;
    congregation.formationAxis  = (arguments[5] != "") ? arguments[5].toFloat() : 0.0f;
    disperse.Reset();  // no dispersar y formar a la vez

    nav.Reset();
    instructionList.clear();

    MessageDebugf("DEBUG: -1, ID: %s, Formación %s. Líder: %s, slot: %d/%d, axis %.0f",
                  robotID.c_str(), congregation.formationShape.c_str(),
                  congregation.leaderID.c_str(), congregation.followerIndex,
                  congregation.totalFollowers, congregation.formationAxis);

    int delay = robotID.toInt() * 200;
    fsmInstruction[0] = WAIT;
    fsmInstruction[1] = delay;
    instructionList.push_back(fsmInstruction);

    fsmInstruction[0] = REQUEST_POSITION;
    fsmInstruction[1] = 0;
    instructionList.push_back(fsmInstruction);
  }

  // GT / GOTO / POSITIONGT / BUG2 — Navegación reactiva al objetivo
  // ("BUG2" se acepta por compatibilidad; el algoritmo es ReactiveNav)
  // Uso: GT|x|y          — navega al objetivo
  //      GT|x|y|seg      — ídem con segmento personalizado (50–400mm)
  else if (command == "GT" || command == "GOTO" ||
           command == "POSITIONGT" || command == "BUG2") {
    float targetX = arguments[1].toFloat();
    float targetY = arguments[2].toFloat();

    // El destino tiene que caer dentro de la arena en curso (NAV_CONFIG|ARENA),
    // no dentro de una constante: con el tope fijo un montaje más grande queda
    // recortado y uno más chico acepta destinos que la cámara no ve.
    if (targetX < 0 || targetX > arenaWidthMm ||
        targetY < 0 || targetY > arenaHeightMm) {
      char buffer[100];
      snprintf(buffer, sizeof(buffer),
               "ERROR: GT objetivo fuera de la arena. X=%.1f (0..%.0f), "
               "Y=%.1f (0..%.0f)",
               targetX, arenaWidthMm, targetY, arenaHeightMm);
      SendMessage(robots["Base"], buffer);
      return;
    }

    // Argumento 3 opcional: distancia de segmento personalizada
    if (arguments[3] != "") {
      float arg3 = arguments[3].toFloat();
      if (arg3 >= 50 && arg3 <= 400) nav.segmentDistance = arg3;
    }

    nav.goalX       = targetX;
    nav.goalY       = targetY;
    nav.pendingInit = true;
    nav.isActive    = false;
    // Re-anclar el EKF al empezar: entre comandos sigue integrando odometría a
    // ciegas y puede arrancar la navegación con metros de deriva acumulada.
    ekf.Reset();
    instructionList.clear();

    fsmInstruction[0] = REQUEST_POSITION;
    fsmInstruction[1] = 0;
    instructionList.push_back(fsmInstruction);

    MessageDebugf("DEBUG: -1, ID: %s, GT: goal=(%.1f,%.1f) seg=%.0fmm",
                  robotID.c_str(), targetX, targetY, nav.segmentDistance);
    char ack[60];
    snprintf(ack, sizeof(ack), "GT iniciado: goal=(%.0f,%.0f)", targetX, targetY);
    SendMessage(robots["Base"], ack);
  }

  // POSITION_RESPONSE
  else if (command == "POSITION_RESPONSE") {
    // Ignorar respuestas no solicitadas (paquetes residuales de sesiones anteriores)
    if (!congregation.waitingForResponse && !nav.pendingInit) {
      MessageDebugf("DEBUG: -1, ID: %s, POSITION_RESPONSE ignorado (no esperado)",
                    robotID.c_str());
      return;
    }
    robotPose.x = arguments[1].toFloat();
    robotPose.y = arguments[2].toFloat();
    robotPose.angle = arguments[3].toFloat();
    congregation.poseFresh = true;   // habilita anclar el slot (ver LEADER_POSITION)

    // Congregación VIRTUAL: no hay líder que difunda su pose, así que el
    // recálculo del slot se dispara acá, con cada pose propia nueva. El punto
    // es fijo, de modo que el goal converge en cuanto se ancla el slot.
    if (congregation.hasGlobalTarget && !congregation.isLeader) {
      UpdateCongregationGoal(congregation.globalTargetX,
                             congregation.globalTargetY, 0.0f);
    }

    MessageDebugf(
        "DEBUG: -1, ID: %s, Posición recibida: x=%.1f, y=%.1f, ángulo=%.1f",
        robotID.c_str(), robotPose.x, robotPose.y, robotPose.angle);

    // EKF: corrección ArUco. La innovación mide qué tan lejos venía la
    // predicción (encoders+gyro) de la cámara — la métrica de validación.
    bool ekfWasInit = ekf.initialized;
    float ekfInnov = ekf.UpdateAruco(robotPose.x, robotPose.y, robotPose.angle);
    if (ekfWasInit) {
      MessageDebugf("DEBUG: -1, ID: %s, EKF innov=%.0fmm est=(%.0f,%.0f,%.0f°)",
                    robotID.c_str(), ekfInnov, ekf.x, ekf.y, ekf.AngleDeg());
    }

    if (congregation.isLeader && congregation.leaderID != "-1") {
      float lx, ly, lang;
      SmoothLeaderPose(robotPose.x, robotPose.y, robotPose.angle, &lx, &ly, &lang);
      char buffer[64];
      snprintf(buffer, sizeof(buffer), "LEADER_POSITION|%s|%.1f|%.1f|%.1f",
               robotID.c_str(), lx, ly, lang);

      if (robots.find("Broadcast") != robots.end()) {
        SendMessage(robots["Broadcast"], buffer);
        delayMicroseconds(500);
        SendMessage(robots["Broadcast"], buffer);
      }
    }

    if (nav.isActive) {
      ReactiveNavStep();
    } else if (nav.pendingInit) {
      nav.pendingInit = false;
      nav.Start(nav.goalX, nav.goalY);
      ReactiveNavStep();
    }

    congregation.positionReceived = true;
  }

  // LEADER_POSITION
  else if (command == "LEADER_POSITION") {
    String receivedLeaderID = arguments[1];

    if (!congregation.isLeader && receivedLeaderID == congregation.leaderID) {
      UpdateCongregationGoal(arguments[2].toFloat(), arguments[3].toFloat(),
                             arguments[4].toFloat());
    }
  }

  // CANCEL_CONGREGATION — también termina la dispersión (mismo "alto enjambre")
  else if (command == "CANCEL_CONGREGATION") {
    congregation.Reset();
    disperse.Reset();
    nav.Reset();
    instructionList.clear();
    state = STOP;
    MessageDebugf("DEBUG: -1, ID: %s, Congregación cancelada", robotID.c_str());
  }

  // DISPERSE — dispersión de enjambre: repeler vecinos hasta separación >= mm
  // DISPERSE|<mm>. Los vecinos llegan por NEIGHBOR_POSITIONS (1 Hz de la Base).
  else if (command == "DISPERSE") {
    congregation.Reset();               // no formar y dispersar a la vez
    disperse.Reset();
    disperse.target = (arguments[1] != "") ? arguments[1].toFloat() : 600.0f;
    nav.Reset();
    instructionList.clear();
    MessageDebugf("DEBUG: -1, ID: %s, dispersión: separación objetivo %.0fmm",
                  robotID.c_str(), disperse.target);
    // Refrescar la pose antes del primer NEIGHBOR (así el settle/hop se decide
    // sobre robotPose fresco, no sobre la última posición de otra conducta).
    fsmInstruction[0] = REQUEST_POSITION;
    fsmInstruction[1] = 0;
    instructionList.push_back(fsmInstruction);
  }

  // NEIGHBOR_POSITIONS — posiciones de los demás robots (para dispersión)
  // NEIGHBOR_POSITIONS|id,x,y;id,x,y;...  (la Base excluye o no al propio robot;
  // aquí se filtra por id). Al llegar, se evalúa un salto de dispersión.
  else if (command == "NEIGHBOR_POSITIONS") {
    disperse.nCount = 0;
    String list = arguments[1];
    int start = 0;
    while (start < (int)list.length() && disperse.nCount < DisperseState::MAX_NEIGHBORS) {
      int semi = list.indexOf(';', start);
      String item = (semi == -1) ? list.substring(start) : list.substring(start, semi);
      int c1 = item.indexOf(',');
      int c2 = item.indexOf(',', c1 + 1);
      if (c1 > 0 && c2 > c1) {
        String nid = item.substring(0, c1);
        float  nx  = item.substring(c1 + 1, c2).toFloat();
        float  ny  = item.substring(c2 + 1).toFloat();
        if (nid != robotID) {
          int i = disperse.nCount++;
          disperse.nId[i] = nid;
          disperse.nX[i]  = nx;
          disperse.nY[i]  = ny;
        } else {
          disperse.selfX    = nx;   // insumo compartido para el reparto de MEET
          disperse.selfY    = ny;
          disperse.selfSeen = true;
        }
      }
      if (semi == -1) break;
      start = semi + 1;
    }
    MaybeDisperseHop();
  }

  // CLEAR_EVASION
  else if (command == "CLEAR_EVASION") {
    intContext.Clear();
    isEvading = false;
    resumeScheduled = false;
    obstacles.Clear();
    instructionList.clear();
    state = STOP;
    MessageDebugf("DEBUG: -1, ID: %s, Sistema de evasión reseteado",
                  robotID.c_str());
  }

  // NAV_CONFIG — configura parámetros de navegación
  // Uso: NAV_CONFIG|SEGMENT_DIST|250   NAV_CONFIG|ARRIVAL_THRESHOLD|20
  else if (command == "NAV_CONFIG") {
    if (arguments[1] == "SEGMENT_DIST") {
      float newDist = arguments[2].toFloat();
      if (newDist >= 50 && newDist <= 400) {
        nav.segmentDistance = newDist;
        char buf[60];
        snprintf(buf, sizeof(buf), "NAV_CONFIG: segmento=%.0fmm", newDist);
        SendMessage(robots["Base"], buf);
      }
    } else if (arguments[1] == "ARRIVAL_THRESHOLD") {
      float newThr = arguments[2].toFloat();
      if (newThr >= 5 && newThr <= 200) {
        nav.arrivalThreshold = newThr;
        char buf[60];
        snprintf(buf, sizeof(buf), "NAV_CONFIG: llegada=%.0fmm", newThr);
        SendMessage(robots["Base"], buf);
      }
    } else if (arguments[1] == "PARKING_DIST") {
      float newDist = arguments[2].toFloat();
      if (newDist >= 150 && newDist <= 600) {
        congregation.parkingDist = newDist;
        char buf[60];
        snprintf(buf, sizeof(buf), "NAV_CONFIG: parking=%.0fmm", newDist);
        SendMessage(robots["Base"], buf);
      }
    } else if (arguments[1] == "WHEEL_DIST") {
      float newDist = arguments[2].toFloat();
      if (newDist >= 20.0 && newDist <= 100.0) {
        centerToWheelDistance = newDist;
        char buf[60];
        snprintf(buf, sizeof(buf), "NAV_CONFIG: wheel_dist=%.1fmm", newDist);
        SendMessage(robots["Base"], buf);
      }
    } else if (arguments[1] == "REALIGN") {
      float newThr = arguments[2].toFloat();
      if (newThr >= 1 && newThr <= 90) {
        nav.realignThreshold = newThr;
        char buf[60];
        snprintf(buf, sizeof(buf), "NAV_CONFIG: realign=%.1f°", nav.realignThreshold);
        SendMessage(robots["Base"], buf);
      }
    } else if (arguments[1] == "GOAL_DEADBAND") {
      float newBand = arguments[2].toFloat();
      if (newBand >= 0 && newBand <= 300) {
        nav.goalDeadband = newBand;
        char buf[60];
        snprintf(buf, sizeof(buf), "NAV_CONFIG: goal_deadband=%.0fmm", nav.goalDeadband);
        SendMessage(robots["Base"], buf);
      }
    } else if (arguments[1] == "ARENA") {
      // Límites del escenario en curso (la base los manda al registrar).
      float w = arguments[2].toFloat();
      float h = arguments[3].toFloat();
      if (w > 100 && h > 100) {
        arenaWidthMm  = w;
        arenaHeightMm = h;
        char buf[70];
        snprintf(buf, sizeof(buf), "NAV_CONFIG: arena=%.0fx%.0fmm", arenaWidthMm,
                 arenaHeightMm);
        SendMessage(robots["Base"], buf);
      }
    } else if (arguments[1] == "YAW_SCALE") {
      float newScale = arguments[2].toFloat();
      if (newScale >= 0.9 && newScale <= 1.1) {
        yawScale = newScale;
        bool save = (arguments[3] == "SAVE");
        if (save) {
          preferences.begin("attabot-config", false);
          preferences.putFloat("yaw_scale", yawScale);
          preferences.end();
        }
        char buf[70];
        snprintf(buf, sizeof(buf), "NAV_CONFIG: yaw_scale=%.4f%s", yawScale,
                 save ? " (guardado)" : "");
        SendMessage(robots["Base"], buf);
      }
    }
  }

  // SENSOR_MASK — ignora un sensor IR (diagnóstico de falsos positivos)
  // Uso: SENSOR_MASK|L|1 (ignorar izquierdo)  SENSOR_MASK|L|0 (reactivar)
  else if (command == "SENSOR_MASK") {
    bool masked = (arguments[2].toInt() != 0);
    if      (arguments[1] == "L") maskLeftIR    = masked;
    else if (arguments[1] == "R") maskRightIR   = masked;
    else if (arguments[1] == "C") maskCentralIR = masked;
    char buf[80];
    snprintf(buf, sizeof(buf), "SENSOR_MASK: L=%d R=%d C=%d (1=ignorado)",
             (int)maskLeftIR, (int)maskRightIR, (int)maskCentralIR);
    SendMessage(robots["Base"], buf);
  }

  // SENSOR_THRESHOLD — umbral de proximidad del APDS9960 central (0..255)
  // Uso: SENSOR_THRESHOLD|C|25        (solo en RAM, para tantear)
  //      SENSOR_THRESHOLD|C|25|SAVE   (persiste en NVS, sobrevive apagón)
  else if (command == "SENSOR_THRESHOLD") {
    if (arguments[1] == "C") {
      int newThr = arguments[2].toInt();
      if (newThr >= 0 && newThr <= 255) {
        centralIRThreshold = newThr;
        bool save = (arguments[3] == "SAVE");
        if (save) {
          preferences.begin("attabot-config", false);
          preferences.putInt("ir_cen_thr", centralIRThreshold);
          preferences.end();
        }
        char buf[80];
        snprintf(buf, sizeof(buf), "SENSOR_THRESHOLD: C=%d%s", centralIRThreshold,
                 save ? " (guardado)" : "");
        SendMessage(robots["Base"], buf);
      }
    }
  }

  // SELFTEST — diagnóstico de hardware rueda por rueda.
  // Uso: SELFTEST            (PWM 40% por defecto)
  //      SELFTEST|55         (fuerza otro % si el robot no arranca)
  //
  // Mueve CADA motor por separado y reporta pulsos de AMBOS encoders más el
  // giro que midió la IMU. Con eso se separan las tres fallas que a simple
  // vista se parecen:
  //   motor muerto   → sus pulsos ≈0 Y la IMU no gira
  //   encoder muerto → sus pulsos ≈0 PERO la IMU sí gira (el motor empuja)
  //   motor flojo    → pulsos muy por debajo del otro lado
  // El robot GIRA sobre su eje en cada paso: necesita ~30cm libres alrededor.
  else if (command == "SELFTEST") {
    int pct = (arguments[1] != "") ? arguments[1].toInt() : 40;
    pct = constrain(pct, 15, 90);
    int pwm = (int)(maxPWMValue * pct / 100.0f);
    const unsigned long STEP_MS = 900;

    instructionList.clear();
    nav.Reset();
    state = STOP;
    ConfigureHBridge(0, 0);

    char buf[190];
    snprintf(buf, sizeof(buf), "SELFTEST: inicio (PWM %d%%) — bateria=%s IMU=%d",
             pct, digitalRead(batteryStatus) == LOW ? "BAJA" : "ok",
             (int)imuAvailable);
    SendMessage(robots["Base"], buf);

    for (int fase = 0; fase < 3; fase++) {
      noInterrupts();
      movement.leftPulseCount = 0;
      movement.rightPulseCount = 0;
      interrupts();
      float yaw0 = yaw;

      int lp = (fase == 1) ? 0 : pwm;      // fase 0: izq · 1: der · 2: ambos
      int rp = (fase == 0) ? 0 : pwm;
      ConfigureHBridge(lp, rp);
      unsigned long t0 = millis();
      while (millis() - t0 < STEP_MS) {
        LeerYaw();                          // mantener el yaw fresco
        delay(10);
      }
      ConfigureHBridge(0, 0);
      delay(400);                           // dejar frenar antes de medir
      LeerYaw();

      noInterrupts();
      int lc = movement.leftPulseCount;
      int rc = movement.rightPulseCount;
      interrupts();
      float dyaw = yaw - yaw0;
      if (dyaw > 180.0f) dyaw -= 360.0f;
      if (dyaw < -180.0f) dyaw += 360.0f;

      const char *nombre = (fase == 0) ? "IZQ sola"
                         : (fase == 1) ? "DER sola" : "AMBAS";
      snprintf(buf, sizeof(buf),
               "SELFTEST %s: pulsos_izq=%d pulsos_der=%d dyaw=%.1f",
               nombre, lc, rc, dyaw);
      SendMessage(robots["Base"], buf);
      delay(500);
    }
    SendMessage(robots["Base"], "SELFTEST: fin");
  }

  // EKF_NAV — conmuta la fuente de pose del control (0 = ArUco crudo, 1 = EKF)
  else if (command == "EKF_NAV") {
    ekfNavEnabled = (arguments[1].toInt() != 0);
    char buf[80];
    snprintf(buf, sizeof(buf), "EKF_NAV: nav con pose %s%s",
             ekfNavEnabled ? "EKF" : "ArUco",
             (ekfNavEnabled && !ekf.initialized) ? " (EKF aún sin inicializar)" : "");
    SendMessage(robots["Base"], buf);
  }

  // GET_STATUS
  else if (command == "GET_STATUS") {
    char buffer[300];
    snprintf(
        buffer, sizeof(buffer),
        "STATUS|ID:%s|State:%d|NAV:%d|Evading:%d|Obs:%d|"
        "Sensors:L%d-C%d-R%d|Prox:%d|Thr:%d|Mask:L%d-C%d-R%d|"
        "Pos:(%.1f,%.1f,%.1f)|"
        "Goal:(%.1f,%.1f)|Yaw:%.1f|IMU:%d|EKF:(%.0f,%.0f,%.0f)|"
        "Cal:(ppr=%.2f,yaw_s=%.4f)|EkfNav:%d",
        robotID.c_str(), state, (int)nav.isActive, (int)isEvading,
        (int)obstacles.HasAnyObstacle(), (int)obstacles.leftObstacle,
        (int)obstacles.centralObstacle, (int)obstacles.rightObstacle,
        centralDistance, centralIRThreshold,
        (int)maskLeftIR, (int)maskCentralIR, (int)maskRightIR,
        robotPose.x, robotPose.y, robotPose.angle,
        nav.goalX, nav.goalY,
        yaw, (int)imuAvailable,
        ekf.x, ekf.y, ekf.AngleDeg(),
        pulsesPerRev, yawScale, (int)ekfNavEnabled);
    SendMessage(robots["Base"], buffer);
  }

  else if (command == "RESET_EVASION") {
    evasionTracker.Reset();
    intContext.Clear();
    isEvading = false;
    resumeScheduled = false;
    obstacles.Clear();
    MessageDebugf(
        "DEBUG: -1, ID: %s, ✅ Sistema de evasión reseteado manualmente",
        robotID.c_str());
  }

  // SEARCH_OBJECT — búsqueda semántica por color (rojo/verde/azul)
  // Patrulla con RANDOM_WALK, se aproxima a candidatos y les lee el color.
  // Uso: SEARCH_OBJECT|rojo — ABORT_NAV la cancela. Prototipo validado en sim.
  else if (command == "SEARCH_OBJECT") {
    if (!frontSensorInitialized) {
      SendMessage(robots["Base"], "SEARCH: APDS9960 no disponible");
      return;
    }
    strncpy(search.targetColor, arguments[1].c_str(),
            sizeof(search.targetColor) - 1);
    search.targetColor[sizeof(search.targetColor) - 1] = '\0';
    search.active = true;
    frontSensor.enableColor(true);
    instructionList.clear();
    fsmInstruction[0] = RANDOM_WALK;
    fsmInstruction[1] = 600000;   // patrulla de 10 min (se re-arma al evadir)
    instructionList.push_back(fsmInstruction);
    state = READ_INSTRUCTION;
    MessageDebugf("DEBUG: -1, ID: %s, SEARCH: buscando objeto %s",
                  robotID.c_str(), search.targetColor);
  }

  // COLOR_READ — lectura puntual RGBC para calibrar umbrales de color en lab
  else if (command == "COLOR_READ") {
    if (!frontSensorInitialized) {
      SendMessage(robots["Base"], "COLOR_READ: APDS9960 no disponible");
      return;
    }
    frontSensor.enableColor(true);
    unsigned long t0 = millis();
    while (!frontSensor.colorDataReady() && millis() - t0 < 300) delay(5);
    uint16_t r, g, b, c;
    frontSensor.getColorData(&r, &g, &b, &c);
    if (!search.active) frontSensor.enableColor(false);
    char buf[80];
    snprintf(buf, sizeof(buf), "COLOR: R=%u G=%u B=%u C=%u", r, g, b, c);
    SendMessage(robots["Base"], buf);
  }

  else if (command == "ABORT_NAV") {
    nav.Reset();
    if (search.active && frontSensorInitialized) frontSensor.enableColor(false);
    search.Reset();
    instructionList.clear();
    imuTurnActive       = false;  // si se abortó a mitad de un giro, no dejar el
    imuTurnIsCorrection = false;  // tracking IMU activo: el próximo TURN debe
    imuTurnCorrCount    = 0;      // reinicializarse limpio (start yaw/accum/target)
    state = STOP;
    SendMessage(robots["Base"], "GT abortado");
    MessageDebugf("DEBUG: -1, ID: %s, Navegación abortada manualmente",
                  robotID.c_str());
  }

  // GET_YAW — retorna el yaw actual de la IMU para validación en Fase 2
  // Uso desde la base: BASE.GET_YAW → responde YAW|<valor>|<imuAvailable>
  else if (command == "GET_YAW") {
    char buffer[60];
    snprintf(buffer, sizeof(buffer), "YAW|%.2f|%d|%.3f",
             yaw, (int)imuAvailable, imuGravity);
    SendMessage(robots["Base"], buffer);
  }
}

// ============================================================================
// FUNCIONES AUXILIARES
// ============================================================================

void SelectMovementRW() {
  int probabilityTurnPos = 15;
  int probabilityMove = 70 * (obstacleDetected ? 0 : 1);
  int probabilityTurnNeg = 15;
  int totalProbabilities =
      probabilityTurnPos + probabilityMove + probabilityTurnNeg;
  std::array<int, 3> cumulativeProbabilities = {
      probabilityTurnPos, probabilityTurnPos + probabilityMove,
      totalProbabilities};
  obstacleDetected = false;

  int randomSelection = random(totalProbabilities);
  int directionRW;
  if (randomSelection < cumulativeProbabilities[0]) {
    directionRW = TURN_POS;
  } else if (randomSelection < cumulativeProbabilities[1]) {
    directionRW = MOVE_FORWARD;
  } else {
    directionRW = TURN_NEG;
  }

  int angle = possibleAngles[random(possibleAngles.size() * 10) %
                             possibleAngles.size()];
  int distance = possibleAdvances[random(possibleAdvances.size() * 10) %
                                  possibleAdvances.size()];
  switch (directionRW) {
  case TURN_POS: {
    fsmInstruction[0] = TURN;
    fsmInstruction[1] = DegToArc(angle);
    break;
  }

  case MOVE_FORWARD: {
    fsmInstruction[0] = MOVE;
    fsmInstruction[1] = distance;
    break;
  }

  case TURN_NEG: {
    fsmInstruction[0] = TURN;
    fsmInstruction[1] = DegToArc(-angle);
    break;
  }
  }

  instructionList.push_front(fsmInstruction);
}

#ifdef DebugSerial
void ReadSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();

    int separatorIndex = command.indexOf('|');
    String cmd = command.substring(0, separatorIndex);
    String valueStr = command.substring(separatorIndex + 1);
    int value = valueStr.toInt();

    if (cmd == "MOVE") {
      fsmInstruction[0] = MOVE;
      fsmInstruction[1] = value;
      instructionList.push_back(fsmInstruction);
      Serial.printf("Comando MOVE %d mm agregado\n", value);

    } else if (cmd == "TURN") {
      fsmInstruction[0] = TURN;
      fsmInstruction[1] = DegToArc(value);
      instructionList.push_back(fsmInstruction);
      Serial.printf("Comando TURN %d grados agregado — arco=%.2fmm\n", value,
                    fsmInstruction[1]);

    } else if (cmd == "STOP") {
      instructionList.clear();
      ConfigureHBridge(0, 0);
      state = STOP;
      Serial.println("Robot detenido");

    } else {
      Serial.println("Comandos: MOVE|valor, TURN|valor, STOP");
    }
  }
}
#endif

// ============================================================================
// FUNCIONES DE LED
// ============================================================================

void LedController::update() {
  unsigned long now = millis();
  switch (currentState) {
  case OFF:
    if (brightness != 0) {
      brightness = 0;
      FastLED.setBrightness(0);
      FastLED.show();
    }
    break;

  case SOLID:
    if (now - lastUpdate > 50) {
      leds[0] = CRGB(red, green, blue);
      FastLED.setBrightness(brightness);
      FastLED.show();
      lastUpdate = now;
    }
    break;

  case BLINKING:
    if (now - lastUpdate >= interval) {
      blinkState = !blinkState;
      if (blinkState) {
        leds[0] = CRGB(red, green, blue);
        FastLED.setBrightness(brightness);
      } else {
        FastLED.setBrightness(0);
      }
      FastLED.show();
      lastUpdate = now;
    }
    break;
  }
}

void setLedColor(uint8_t red, uint8_t green, uint8_t blue) {
  ledCtrl.setSolid(red, green, blue, maxBrightness);
}

void setLedBrightness(uint8_t brightness) {
  ledCtrl.brightness = brightness;
  if (brightness == 0)
    ledCtrl.setOff();
  else
    ledCtrl.currentState = LedController::SOLID;
}

void setLedBlink(uint8_t red, uint8_t green, uint8_t blue,
                 unsigned long intervalMs) {
  ledCtrl.setBlink(red, green, blue, maxBrightness, intervalMs);
}

// ============================================================================
// FUNCIONES IMU
// ============================================================================

void setupIMU() {
  DebugSerialPrintln("Inicializando IMU ICM-20948...");

  bool imuDetected = false;
  int attempts = 0;
  const int maxAttempts = 3;

  while (!imuDetected && attempts < maxAttempts) {
    attempts++;
    DebugSerialPrintf("Intento %d/%d de conexión con IMU...\n", attempts, maxAttempts);

    imu.begin(Wire, AD0_VAL);

    if (imu.status == ICM_20948_Stat_Ok) {
      imuDetected = true;
      DebugSerialPrintln("IMU detectada correctamente");
    } else {
      DebugSerialPrintf("Error al conectar con IMU. Status: %d\n", imu.status);
      delay(500);
    }
  }

  if (!imuDetected) {
    DebugSerialPrintln("ERROR CRÍTICO: No se pudo detectar la IMU");
    DebugSerialPrintln("Verifica:");
    DebugSerialPrintln("  1. Conexión física del cable Qwiic");
    DebugSerialPrintln("  2. AD0_VAL debe ser 1 (0x69) o 0 (0x68)");
    DebugSerialPrintln("  3. Que no haya conflictos con otros dispositivos I2C");
    ledCtrl.setBlink(255, 0, 0, maxBrightness, 500);
    return;  // imuAvailable permanece false
  }

  DebugSerialPrintln("Inicializando DMP...");
  bool success = true;

  success &= (imu.initializeDMP() == ICM_20948_Stat_Ok);
  if (!success) {
    DebugSerialPrintln("ERROR: Falló initializeDMP()");
    DebugSerialPrintln("Verifica que ICM_20948_USE_DMP esté definido en ICM_20948_C.h");
    ledCtrl.setBlink(255, 128, 0, maxBrightness, 300);
    return;
  }

  success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER)   == ICM_20948_Stat_Ok);

  if (!success) {
    DebugSerialPrintln("ERROR: Falló habilitando sensores DMP");
    return;
  }

  success &= (imu.setDMPODRrate(DMP_ODR_Reg_Quat9, 1) == ICM_20948_Stat_Ok);
  success &= (imu.setDMPODRrate(DMP_ODR_Reg_Accel, 1) == ICM_20948_Stat_Ok);
  success &= (imu.enableFIFO()  == ICM_20948_Stat_Ok);
  success &= (imu.enableDMP()   == ICM_20948_Stat_Ok);
  success &= (imu.resetDMP()    == ICM_20948_Stat_Ok);
  success &= (imu.resetFIFO()   == ICM_20948_Stat_Ok);

  if (!success) {
    DebugSerialPrintln("ERROR: Falló configurando FIFO/DMP");
    ledCtrl.setBlink(0, 255, 0, maxBrightness, 300);
    return;
  }

  // --- Restaurar calibración desde Preferences ---
  biasStore store;
  preferences.begin("attabot-config", true);  // read-only
  store.biasGyroX  = preferences.getInt("bias_gx", 0);
  store.biasGyroY  = preferences.getInt("bias_gy", 0);
  store.biasGyroZ  = preferences.getInt("bias_gz", 0);
  store.biasAccelX = preferences.getInt("bias_ax", 0);
  store.biasAccelY = preferences.getInt("bias_ay", 0);
  store.biasAccelZ = preferences.getInt("bias_az", 0);
  store.biasCPassX = preferences.getInt("bias_cx", 0);
  store.biasCPassY = preferences.getInt("bias_cy", 0);
  store.biasCPassZ = preferences.getInt("bias_cz", 0);
  preferences.end();

  if (store.IsValid()) {
    DebugSerialPrintln("Calibración válida encontrada en Preferences");
    bool calOk = true;
    calOk &= (imu.setBiasGyroX(store.biasGyroX)   == ICM_20948_Stat_Ok);
    calOk &= (imu.setBiasGyroY(store.biasGyroY)   == ICM_20948_Stat_Ok);
    calOk &= (imu.setBiasGyroZ(store.biasGyroZ)   == ICM_20948_Stat_Ok);
    calOk &= (imu.setBiasAccelX(store.biasAccelX) == ICM_20948_Stat_Ok);
    calOk &= (imu.setBiasAccelY(store.biasAccelY) == ICM_20948_Stat_Ok);
    calOk &= (imu.setBiasAccelZ(store.biasAccelZ) == ICM_20948_Stat_Ok);
    calOk &= (imu.setBiasCPassX(store.biasCPassX) == ICM_20948_Stat_Ok);
    calOk &= (imu.setBiasCPassY(store.biasCPassY) == ICM_20948_Stat_Ok);
    calOk &= (imu.setBiasCPassZ(store.biasCPassZ) == ICM_20948_Stat_Ok);

    if (calOk) {
      DebugSerialPrintln("Calibración restaurada correctamente");
    } else {
      DebugSerialPrintln("ADVERTENCIA: Falló al aplicar calibración");
    }
  } else {
    DebugSerialPrintln("ADVERTENCIA: No hay calibración válida en Preferences");
    DebugSerialPrintln("La IMU funcionará con valores por defecto");
  }

  imuAvailable = true;
  DebugSerialPrintln("IMU inicializada exitosamente");
  ledCtrl.setSolid(0, 255, 0, maxBrightness);
  delay(1000);
  ledCtrl.setOff();
}

void LeerYaw() {
  if (!imuAvailable) return;

  // El DMP produce a ~112Hz y este loop lee a ≤50Hz: hay que drenar TODOS
  // los paquetes pendientes y quedarse con el más reciente. No usar
  // resetFIFO() con el DMP activo — deja paquetes parciales que corrompen
  // las lecturas siguientes (yaw congelado durante giros).
  icm_20948_DMP_data_t data;
  bool   gotQuat = false;
  double q1 = 0, q2 = 0, q3 = 0;

  for (int i = 0; i < 20; i++) {
    imu.readDMPdataFromFIFO(&data);

    if ((imu.status != ICM_20948_Stat_Ok) &&
        (imu.status != ICM_20948_Stat_FIFOMoreDataAvail)) {
      if (imu.status != ICM_20948_Stat_FIFONoDataAvail) {
        DebugSerialPrintf("Error leyendo FIFO: %d\n", imu.status);
      }
      break;
    }

    if ((data.header & DMP_header_bitmap_Quat9) > 0) {
      q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;
      q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
      q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;
      gotQuat = true;
    }

    if ((data.header & DMP_header_bitmap_Accel) > 0) {
      float accX = (float)data.Raw_Accel.Data.X / conversionFactor;
      float accY = (float)data.Raw_Accel.Data.Y / conversionFactor;
      float accZ = (float)data.Raw_Accel.Data.Z / conversionFactor;
      imuGravity = sqrt(accX * accX + accY * accY + accZ * accZ);
    }

    if (imu.status != ICM_20948_Stat_FIFOMoreDataAvail) break;
  }

  if (gotQuat) {
    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
    double t3 = +2.0 * (q0 * q3 + q1 * q2);
    double t4 = +1.0 - 2.0 * (q2 * q2 + q3 * q3);
    yaw = fmod(-atan2(t3, t4) * RAD_TO_DEG + 450.0, 360.0);
    DebugSerialPrintf("Yaw actual: %.2f°\n", yaw);
  }
}