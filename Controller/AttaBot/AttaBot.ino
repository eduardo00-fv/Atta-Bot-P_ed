#include "utils.h"
#include <Adafruit_APDS9960.h> // v1.3.0
#include <ArduinoOTA.h>
#include <ESP32Servo.h> // v3.0.9
#include <FastLED.h>    // v3.10.2
#include <ICM_20948.h>  // v1.2.12
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
#define frontServoPin 26
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

// Límites del área de trabajo (en milímetros)
const float max_workspace_x = 2000;
const float max_workspace_y = 2000;

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

// Servo
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
Servo frontServo;
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
std::array<String, 5> SeparateCommand(const String &command, char delimiter);
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

  DebugSerialPrintln("[2] Inicializando servo...");
  frontServo.setPeriodHertz(50);
  frontServo.attach(frontServoPin, 1000, 2000);
  frontServo.write(90);
  DebugSerialPrintln("[2] Servo OK");

  DebugSerialPrintln("[3] Inicializando motores PWM...");
  bool pwmOk = ledcAttach(leftMotorForward,  pwm_freq, pwm_resolution)
             & ledcAttach(leftMotorBackward,  pwm_freq, pwm_resolution)
             & ledcAttach(rightMotorForward,  pwm_freq, pwm_resolution)
             & ledcAttach(rightMotorBackward, pwm_freq, pwm_resolution);
  if (!pwmOk) DebugSerialPrintf("[3] ERROR: ledcAttach falló — freq=%d res=%d incompatibles\n",
                                 pwm_freq, pwm_resolution);
  DebugSerialPrintf("[3] Motores PWM: %dHz %d-bit (max=%d) %s\n",
                    pwm_freq, pwm_resolution, maxPWMValue, pwmOk ? "OK" : "FALLO");

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
          float corrArc = radians(error) * centerToWheelDistance;
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
              radians(imuTurnTargetDeg - lead - imuTurnAccumDeg) *
              centerToWheelDistance;
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

      fsmInstruction[0] = TURN;
      fsmInstruction[1] =
          radians(random(2) ? 180 : -180) * centerToWheelDistance;
      retreatSequence.push_back(fsmInstruction);

      fsmInstruction[0] = MOVE;
      fsmInstruction[1] = 300;
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
        fsmInstruction[1] = radians(avoidanceAngle) * centerToWheelDistance;
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
      float ox = robotPose.x + 100.0f * cos(radians(robotPose.angle));
      float oy = robotPose.y + 100.0f * sin(radians(robotPose.angle));
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
void ReactiveNavStep() {
  float x = robotPose.x;
  float y = robotPose.y;

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
    float relGoal = NormalizeAngle(goalAngle - robotPose.angle);
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
  float angleDiff  = NormalizeAngle(finalAngle - robotPose.angle);

  MessageDebugf("DEBUG: -1, ID: %s, NAV: dist=%.1f goal=%.1f° bias=%.1f° seg=%.1f%s",
                robotID.c_str(), dist, goalAngle, bias, seg,
                avoiding ? " [AVOID]" : "");

  if (abs(angleDiff) > 5.0f) {
    fsmInstruction[0] = TURN;
    fsmInstruction[1] = radians(angleDiff) * centerToWheelDistance;
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
        if (centralDistance > 2 && !maskCentralIR) {
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
  fsmInstruction[1] = radians(random(2) ? 60 : -60) * centerToWheelDistance;
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

std::array<String, 5> SeparateCommand(const String &command, char delimiter) {
  std::array<String, 5> results;
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
  std::array<String, 5> arguments = SeparateCommand(command, '|');
  command = arguments[0];

  // CONFIG
  if (command == "CONFIG") {
    if (arguments[1] == "START") {
      robots["Base"] = udp.remoteIP();
      IPAddress ipAddress;
      ipAddress.fromString(arguments[2]);
      robots["Broadcast"] = ipAddress;
      // Reset completo: limpia cualquier navegación activa de sesiones anteriores
      instructionList.clear();
      isEvading = false;
      obstacles.Clear();
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
      fsmInstruction[1] = radians(value) * centerToWheelDistance;
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

  // SERVO
  else if (command == "SERVO") {
    int servoAngle = constrain(arguments[1].toInt(), 5, 175);
    DebugSerialPrintf("Servo: %d°\n", servoAngle);
    frontServo.write(servoAngle);
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

  // CONGREGATION
  else if (command == "CONGREGATION") {
    congregation.leaderID = arguments[1];
    congregation.isLeader = (congregation.leaderID == robotID);
    congregation.positionReceived = false;
    congregation.hasGlobalTarget = false;
    congregation.stagingDone = false;
    congregation.slotAngleSet = false;
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

  // GT / GOTO / POSITIONGT / BUG2 — Navegación reactiva al objetivo
  // ("BUG2" se acepta por compatibilidad; el algoritmo es ReactiveNav)
  // Uso: GT|x|y          — navega al objetivo
  //      GT|x|y|seg      — ídem con segmento personalizado (50–400mm)
  else if (command == "GT" || command == "GOTO" ||
           command == "POSITIONGT" || command == "BUG2") {
    float targetX = arguments[1].toFloat();
    float targetY = arguments[2].toFloat();

    if (abs(targetX) > max_workspace_x || abs(targetY) > max_workspace_y) {
      char buffer[100];
      snprintf(buffer, sizeof(buffer),
               "ERROR: GT objetivo fuera de rango. X=%.1f (max=%.1f), "
               "Y=%.1f (max=%.1f)",
               targetX, max_workspace_x, targetY, max_workspace_y);
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
      char buffer[64];
      snprintf(buffer, sizeof(buffer), "LEADER_POSITION|%s|%.1f|%.1f|%.1f",
               robotID.c_str(), robotPose.x, robotPose.y, robotPose.angle);

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
      float leaderX = arguments[2].toFloat();
      float leaderY = arguments[3].toFloat();

      // Calcular punto de estacionamiento: slot en círculo alrededor del líder.
      // Aproximación en dos etapas: primero un waypoint en el mismo rayo del
      // slot pero STAGING_MARGIN más lejos del líder, y de ahí entrada radial
      // — la recta al goal nunca cruza el círculo de parking (ni al líder).
      const float STAGING_MARGIN = 150.0f;
      int   n     = max(1, congregation.totalFollowers);
      if (!congregation.slotAngleSet) {
        // n==1: slot del lado por donde viene el follower — evita slots contra
        // la pared cuando el líder está cerca del borde (visto 2026-07-03).
        // n>1: distribución fija por índice (única entre followers, pero ciega
        // a paredes; asignación por la Base pendiente al escalar el enjambre).
        congregation.slotAngle =
            (n == 1) ? atan2(robotPose.y - leaderY, robotPose.x - leaderX)
                     : (2.0f * PI * congregation.followerIndex) / n;
        congregation.slotAngleSet = true;
      }
      float angle = congregation.slotAngle;
      congregation.slotX = leaderX + congregation.parkingDist * cos(angle);
      congregation.slotY = leaderY + congregation.parkingDist * sin(angle);
      float goalDist = congregation.stagingDone
                           ? congregation.parkingDist
                           : congregation.parkingDist + STAGING_MARGIN;
      float parkX = leaderX + goalDist * cos(angle);
      float parkY = leaderY + goalDist * sin(angle);

      if (nav.isActive) {
        nav.goalX = parkX;
        nav.goalY = parkY;
      } else if (!nav.pendingInit) {
        nav.goalX       = parkX;
        nav.goalY       = parkY;
        nav.pendingInit = true;
        if (!congregation.waitingForResponse) {
          fsmInstruction[0] = REQUEST_POSITION;
          fsmInstruction[1] = 0;
          instructionList.push_back(fsmInstruction);
        }
        MessageDebugf("DEBUG: -1, ID: %s, CONGREGATION: slot %d/%d → %s (%.1f,%.1f)",
                      robotID.c_str(), congregation.followerIndex, n,
                      congregation.stagingDone ? "parking" : "staging",
                      parkX, parkY);
      }
    }
  }

  // CANCEL_CONGREGATION
  else if (command == "CANCEL_CONGREGATION") {
    congregation.Reset();
    nav.Reset();
    instructionList.clear();
    state = STOP;
    MessageDebugf("DEBUG: -1, ID: %s, Congregación cancelada", robotID.c_str());
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

  // GET_STATUS
  else if (command == "GET_STATUS") {
    char buffer[250];
    snprintf(
        buffer, sizeof(buffer),
        "STATUS|ID:%s|State:%d|NAV:%d|Evading:%d|Obs:%d|"
        "Sensors:L%d-C%d-R%d|Pos:(%.1f,%.1f,%.1f)|"
        "Goal:(%.1f,%.1f)|Yaw:%.1f|IMU:%d|EKF:(%.0f,%.0f,%.0f)",
        robotID.c_str(), state, (int)nav.isActive, (int)isEvading,
        (int)obstacles.HasAnyObstacle(), (int)obstacles.leftObstacle,
        (int)obstacles.centralObstacle, (int)obstacles.rightObstacle,
        robotPose.x, robotPose.y, robotPose.angle,
        nav.goalX, nav.goalY,
        yaw, (int)imuAvailable,
        ekf.x, ekf.y, ekf.AngleDeg());
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
    fsmInstruction[1] = radians(angle) * centerToWheelDistance;
    break;
  }

  case MOVE_FORWARD: {
    fsmInstruction[0] = MOVE;
    fsmInstruction[1] = distance;
    break;
  }

  case TURN_NEG: {
    fsmInstruction[0] = TURN;
    fsmInstruction[1] = -radians(angle) * centerToWheelDistance;
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
      fsmInstruction[1] = radians(value) * centerToWheelDistance;
      instructionList.push_back(fsmInstruction);
      Serial.printf("Comando TURN %d grados agregado\n", value);

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