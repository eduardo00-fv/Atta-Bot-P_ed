/*
 * HW_Test.ino — Diagnóstico de hardware del AttaBot por SERIAL.
 *
 * Sketch INDEPENDIENTE: sin WiFi, sin base y sin cámara. Sirve justamente
 * cuando el robot NO se conecta o "dejó de andar" y no se puede diagnosticar
 * por UDP. Flashealo, abrí el Monitor Serie a 115200 y escribí los comandos.
 * Cuando termines, re-flasheá el firmware normal (Controller/AttaBot/AttaBot.ino).
 *
 * MISMOS pines, misma librería y misma decodificación de encoders que el
 * firmware, para que lo que mida acá valga allá.
 *
 * ── Lo que resuelve ────────────────────────────────────────────────────────
 * Mueve CADA motor por separado y cruza tres fuentes: pulsos del encoder
 * propio, pulsos del ajeno y giro medido por la IMU. Ese cruce separa fallas
 * que a simple vista son idénticas ("el robot no anda"):
 *
 *   pulsos ≈0  Y  IMU no gira   → MOTOR muerto (no empuja)
 *   pulsos ≈0  PERO IMU gira    → ENCODER muerto (empuja pero no cuenta)
 *   pulsos < 75% del otro lado  → motor FLOJO (se irá de lado al avanzar)
 *   cuenta el encoder ajeno     → cableado cruzado
 *
 * ── Comandos (Monitor Serie, 115200, "Nueva línea") ────────────────────────
 *   t        test COMPLETO: motores + encoders + IMU + IR + batería
 *   m        solo motores (izq sola → der sola → ambas)
 *   e        encoders en vivo: girá las ruedas A MANO y mirá los contadores
 *   i        IR en vivo (IZQ/DER digitales + APDS central crudo)
 *   g        IMU en vivo (yaw)
 *   b        batería
 *   p <n>    cambia el PWM de prueba en % (default 40; subilo si no arranca)
 *   s        STOP de emergencia
 *   ?        ayuda
 *
 * OJO: en los tests de motor el robot GIRA SOBRE SU EJE. Dejale ~30cm libres
 * o sostenelo en el aire (en el aire los pulsos son válidos, el giro de la
 * IMU no).
 */
#include <ICM_20948.h>          // v1.2.12 — misma que el firmware
#include <Adafruit_APDS9960.h>  // v1.3.0
#include <Wire.h>

// ── Pines (idénticos a AttaBot.ino) ─────────────────────────────────────────
#define leftMotorForward  12
#define leftMotorBackward 14
#define rightMotorForward 13
#define rightMotorBackward 15

#define leftEncoderC1  32
#define leftEncoderC2  35
#define rightEncoderC1 23
#define rightEncoderC2 25

#define enableLeftInfraredSensor  5
#define leftInfraredSensor       33
#define enableRightInfraredSensor 18
#define rightInfraredSensor      27

#define batteryStatus 19
#define AD0_VAL 1

#define pwm_freq 1000
#define pwm_resolution 14
const int maxPWMValue = (1 << pwm_resolution) - 1;

// Geometría (para traducir pulsos a mm, igual que el firmware)
const float wheelCircumference = PI * 44.5;
float pulsesPerRev = 574;       // nominal; cada robot tiene el suyo calibrado

ICM_20948_I2C imu;
Adafruit_APDS9960 apds;
bool imuOk = false, apdsOk = false;
int  testPct = 40;

volatile int leftPulseCount = 0, rightPulseCount = 0;
volatile int pastLeftEncoder = 0, pastRightEncoder = 0;

// ── Encoders: decodificación en cuadratura idéntica al firmware ─────────────
void IRAM_ATTR LeftWheelPulses() {
  int MSB = digitalRead(leftEncoderC2);
  int LSB = digitalRead(leftEncoderC1);
  int encoder = (MSB << 1) | LSB;
  int sum = (pastLeftEncoder << 2) | encoder;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    leftPulseCount++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    leftPulseCount--;
  }
  pastLeftEncoder = encoder;
}

void IRAM_ATTR RightWheelPulses() {
  int MSB = digitalRead(rightEncoderC1);
  int LSB = digitalRead(rightEncoderC2);
  int encoder = (MSB << 1) | LSB;
  int sum = (pastRightEncoder << 2) | encoder;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    rightPulseCount++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    rightPulseCount--;
  }
  pastRightEncoder = encoder;
}

void motors(int leftPWM, int rightPWM) {
  if (leftPWM >= 0) {
    ledcWrite(leftMotorBackward, 0);
    ledcWrite(leftMotorForward, leftPWM);
  } else {
    ledcWrite(leftMotorForward, 0);
    ledcWrite(leftMotorBackward, -leftPWM);
  }
  if (rightPWM >= 0) {
    ledcWrite(rightMotorBackward, 0);
    ledcWrite(rightMotorForward, rightPWM);
  } else {
    ledcWrite(rightMotorForward, 0);
    ledcWrite(rightMotorBackward, -rightPWM);
  }
}

void stopMotors() { motors(0, 0); }

// Port 1:1 de LeerYaw() del firmware: DRENA todos los paquetes pendientes y se
// queda con el más reciente (el DMP produce a ~112Hz). No usar resetFIFO() con
// el DMP activo: deja paquetes parciales y el yaw se congela durante los giros.
float lastYaw = 0.0f;
float readYaw() {
  if (!imuOk) return lastYaw;
  icm_20948_DMP_data_t data;
  bool gotQuat = false;
  double q1 = 0, q2 = 0, q3 = 0;
  for (int i = 0; i < 20; i++) {
    imu.readDMPdataFromFIFO(&data);
    if ((imu.status != ICM_20948_Stat_Ok) &&
        (imu.status != ICM_20948_Stat_FIFOMoreDataAvail)) break;
    if ((data.header & DMP_header_bitmap_Quat9) > 0) {
      q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;
      q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
      q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;
      gotQuat = true;
    }
    if (imu.status != ICM_20948_Stat_FIFOMoreDataAvail) break;
  }
  if (gotQuat) {
    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
    double t3 = +2.0 * (q0 * q3 + q1 * q2);
    double t4 = +1.0 - 2.0 * (q2 * q2 + q3 * q3);
    lastYaw = (float)fmod(-atan2(t3, t4) * RAD_TO_DEG + 450.0, 360.0);
  }
  return lastYaw;
}

// Un paso del test de motores. Devuelve por referencia lo medido.
void motorStep(const char *nombre, int lPWM, int rPWM,
               int *lc, int *rc, float *dyaw) {
  noInterrupts();
  leftPulseCount = 0;
  rightPulseCount = 0;
  interrupts();
  float y0 = readYaw();

  motors(lPWM, rPWM);
  unsigned long t0 = millis();
  while (millis() - t0 < 900) {
    readYaw();                    // mantener el FIFO drenado
    delay(5);
  }
  stopMotors();
  delay(400);                     // dejar frenar antes de medir

  noInterrupts();
  *lc = leftPulseCount;
  *rc = rightPulseCount;
  interrupts();
  float y1 = readYaw();
  float d = y1 - y0;
  if (d > 180.0f) d -= 360.0f;
  if (d < -180.0f) d += 360.0f;
  *dyaw = d;

  Serial.printf("  %-9s pulsos_izq=%5d  pulsos_der=%5d  dyaw=%+6.1f°\n",
                nombre, *lc, *rc, *dyaw);
}

void veredictoLado(const char *nombre, int propios, int ajenos, float dyaw) {
  if (abs(propios) < 20) {
    if (fabs(dyaw) > 8.0f) {
      Serial.printf("  %-10s ENCODER MUERTO — el motor empuja (IMU %+.0f°) "
                    "pero no cuenta pulsos\n", nombre, dyaw);
    } else {
      Serial.printf("  %-10s MOTOR MUERTO — sin pulsos y sin giro (%+.0f°)\n",
                    nombre, dyaw);
    }
  } else if (abs(ajenos) > abs(propios) * 0.4f) {
    Serial.printf("  %-10s CABLEADO CRUZADO — el otro encoder también contó "
                  "(%d vs %d)\n", nombre, propios, ajenos);
  } else {
    Serial.printf("  %-10s ok (%d pulsos, IMU %+.0f°)\n", nombre, propios, dyaw);
  }
}

void testMotores() {
  int pwm = (int)(maxPWMValue * testPct / 100.0f);
  Serial.printf("\n=== MOTORES (PWM %d%%) — el robot GIRA, dejá espacio ===\n", testPct);
  for (int k = 3; k > 0; k--) { Serial.printf("  %d...\n", k); delay(1000); }

  int li, ld, ri, rd, bi, bd;
  float ly, ry, by;
  motorStep("IZQ sola", pwm, 0, &li, &ld, &ly);
  delay(500);
  motorStep("DER sola", 0, pwm, &ri, &rd, &ry);
  delay(500);
  motorStep("AMBAS",    pwm, pwm, &bi, &bd, &by);

  Serial.println("\n--- Veredicto ---");
  veredictoLado("IZQUIERDO", li, ld, ly);
  veredictoLado("DERECHO",   rd, ri, ry);

  if (abs(bi) > 20 && abs(bd) > 20) {
    float ratio = (float)min(abs(bi), abs(bd)) / max(abs(bi), abs(bd));
    if (ratio < 0.75f) {
      Serial.printf("  AMBAS      DESBALANCE %.0f%% — el motor %s rinde menos "
                    "(%d vs %d); se irá de lado al avanzar\n",
                    ratio * 100, abs(bi) < abs(bd) ? "izquierdo" : "derecho", bi, bd);
    } else if (fabs(by) > 12.0f) {
      Serial.printf("  AMBAS      avanza TORCIDO (IMU %+.0f° en línea recta)\n", by);
    } else {
      Serial.printf("  AMBAS      ok (%d vs %d, desvío %+.0f°)\n", bi, bd, by);
    }
    float mmPorPulso = wheelCircumference / pulsesPerRev;
    Serial.printf("  (≈ %.0fmm por rueda con PPR nominal %.0f)\n",
                  (abs(bi) + abs(bd)) / 2.0f * mmPorPulso, pulsesPerRev);
  } else {
    Serial.printf("  AMBAS      NO AVANZÓ (%d vs %d)\n", bi, bd);
  }
}

void testIR() {
  Serial.println("\n=== IR (5s) — pasá la mano por cada sensor ===");
  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    bool izq = digitalRead(leftInfraredSensor) == LOW;
    bool der = digitalRead(rightInfraredSensor) == LOW;
    int prox = apdsOk ? apds.readProximity() : -1;
    Serial.printf("\r  IZQ %s   DER %s   CEN prox=%3d  ",
                  izq ? "DETECTA" : " libre ", der ? "DETECTA" : " libre ", prox);
    delay(150);
  }
  Serial.println();
}

void testIMU() {
  Serial.println("\n=== IMU (5s) — girá el robot a mano ===");
  if (!imuOk) { Serial.println("  ✗ IMU NO disponible (revisá I2C)"); return; }
  float y0 = readYaw(), mn = 999, mx = -999;
  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    float y = readYaw();
    mn = min(mn, y); mx = max(mx, y);
    Serial.printf("\r  yaw=%+7.1f°  ", y);
    delay(150);
  }
  Serial.printf("\n  rango observado: %.1f°  %s\n", mx - mn,
                (mx - mn) < 1.0 ? "⚠ el yaw NO se movió — IMU congelada" : "ok");
  (void)y0;
}

void testBateria() {
  bool baja = digitalRead(batteryStatus) == LOW;
  Serial.printf("\n=== BATERÍA: %s ===\n", baja ? "⚠ BAJA (afecta a los motores)" : "ok");
}

void ayuda() {
  Serial.println(F("\n=== HW_Test AttaBot (serial 115200) ==="));
  Serial.println(F("  t  test COMPLETO      m  solo motores     e  encoders en vivo"));
  Serial.println(F("  i  IR en vivo         g  IMU en vivo      b  batería"));
  Serial.println(F("  p <n>  PWM de prueba %   s  STOP           ?  ayuda"));
  Serial.printf("  PWM actual: %d%%\n", testPct);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  delay(400);

  ledcAttach(leftMotorForward,  pwm_freq, pwm_resolution);
  ledcAttach(leftMotorBackward, pwm_freq, pwm_resolution);
  ledcAttach(rightMotorForward, pwm_freq, pwm_resolution);
  ledcAttach(rightMotorBackward, pwm_freq, pwm_resolution);
  stopMotors();

  pinMode(leftEncoderC1, INPUT_PULLUP);
  pinMode(leftEncoderC2, INPUT);      // GPIO35 es input-only, sin pull-up
  pinMode(rightEncoderC1, INPUT_PULLUP);
  pinMode(rightEncoderC2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncoderC1),  LeftWheelPulses,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoderC2),  LeftWheelPulses,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderC1), RightWheelPulses, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderC2), RightWheelPulses, CHANGE);

  pinMode(enableLeftInfraredSensor, OUTPUT);
  pinMode(enableRightInfraredSensor, OUTPUT);
  digitalWrite(enableLeftInfraredSensor, LOW);   // LOW = habilitado (como el firmware)
  digitalWrite(enableRightInfraredSensor, LOW);
  pinMode(leftInfraredSensor, INPUT);
  pinMode(rightInfraredSensor, INPUT);
  pinMode(batteryStatus, INPUT);

  Wire.begin();
  // La IMU DEBE inicializarse ANTES del APDS (mismo orden que el firmware)
  imu.begin(Wire, AD0_VAL);
  imuOk = (imu.status == ICM_20948_Stat_Ok);
  if (imuOk) {
    imuOk &= (imu.initializeDMP() == ICM_20948_Stat_Ok);
    imuOk &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
    imuOk &= (imu.setDMPODRrate(DMP_ODR_Reg_Quat9, 1) == ICM_20948_Stat_Ok);
    imuOk &= (imu.enableFIFO() == ICM_20948_Stat_Ok);
    imuOk &= (imu.enableDMP() == ICM_20948_Stat_Ok);
    imuOk &= (imu.resetDMP() == ICM_20948_Stat_Ok);
    imuOk &= (imu.resetFIFO() == ICM_20948_Stat_Ok);
  }
  apdsOk = apds.begin();
  if (apdsOk) apds.enableProximity(true);

  Serial.printf("\nIMU %s   APDS9960 %s   batería %s\n",
                imuOk ? "OK" : "NO DETECTADA",
                apdsOk ? "OK" : "NO DETECTADO",
                digitalRead(batteryStatus) == LOW ? "BAJA" : "ok");
  ayuda();
}

void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;
  char c = line.charAt(0);

  if (c == 't') {
    testBateria();
    if (!imuOk) Serial.println("⚠ sin IMU no se puede distinguir motor muerto de encoder muerto");
    testMotores();
    testIR();
    testIMU();
    Serial.println("\n=== FIN ===");
  } else if (c == 'm') {
    testMotores();
  } else if (c == 'e') {
    Serial.println("\n=== ENCODERS (8s) — girá las ruedas A MANO ===");
    noInterrupts(); leftPulseCount = 0; rightPulseCount = 0; interrupts();
    unsigned long t0 = millis();
    while (millis() - t0 < 8000) {
      Serial.printf("\r  izq=%6d   der=%6d  ", leftPulseCount, rightPulseCount);
      delay(150);
    }
    Serial.println("\n  (si una rueda no cuenta al girarla a mano: encoder o cableado)");
  } else if (c == 'i') {
    testIR();
  } else if (c == 'g') {
    testIMU();
  } else if (c == 'b') {
    testBateria();
  } else if (c == 'p') {
    int sp = line.indexOf(' ');
    if (sp > 0) {
      testPct = constrain(line.substring(sp + 1).toInt(), 15, 90);
      Serial.printf(">> PWM de prueba = %d%%\n", testPct);
    }
  } else if (c == 's') {
    stopMotors();
    Serial.println(">> STOP");
  } else {
    ayuda();
  }
}
