/*
 * IR_Calibration.ino — Calibración de los sensores IR del AttaBot por SERIAL.
 *
 * Sketch INDEPENDIENTE para desarmar y calibrar los sensores en casa: sin base,
 * sin cámara y sin WiFi. Flashealo, abrí el Monitor Serie a 115200 y ajustá los
 * pots mirando las lecturas en vivo. Cuando termines, re-flasheá el firmware
 * normal (Controller/AttaBot/AttaBot.ino).
 *
 * Sensores (MISMOS pines y librería que el firmware, para que lo calibrado valga):
 *   IZQ = pin 33  (HW-488 digital, 2 pots)  — OUT activo-BAJO: LOW = detecta
 *   DER = pin 27  (HW-488 digital, 2 pots)  — idem
 *   CEN = APDS9960 (I2C 0x39)               — proximidad CRUDA 0..255 (mayor = más cerca)
 *
 * HW-488 (IZQ/DER) — 2 pots, se tunean a ojo con el feedback:
 *   1. Frente LIBRE → debe quedar «libre». Si dispara solo, bajá sensibilidad.
 *   2. Objeto a la distancia deseada (regla) → girá el pot de DISTANCIA hasta que
 *      JUSTO pase a «detecta».
 *   3. Quitá el objeto → vuelve a «libre» sin titilar. Si titila, afiná el 2º pot.
 *   (El renglón de ESTABILIDAD cada 2s cuenta los cambios: 0 = estable.)
 *
 * CEN (APDS, SIN pots) — su umbral es software. Mirá el valor crudo a distintas
 *   distancias y elegí un umbral. En el firmware real la regla es
 *   'centralDistance > umbral' (hoy 2, demasiado sensible). Comando serial:
 *      t <n>   fija el umbral de prueba y ves cómo queda «detecta/libre».
 *
 * Comandos serial:   t <n> = umbral central de prueba    ·    ? = ayuda
 */
#include <Wire.h>
#include <Adafruit_APDS9960.h>   // v1.3.0 (misma que el firmware)

#define IR_LEFT   33
#define IR_RIGHT  27

Adafruit_APDS9960 apds;
bool apdsOK = false;
int  centralThreshold = 10;      // umbral de prueba del central (ajustable por serial)

// Contadores de estabilidad (chatter) por canal digital, ventana de 2s
bool lastL = false, lastR = false;
unsigned long flipL = 0, flipR = 0;
unsigned long windowStart = 0;
unsigned long lastPrint = 0;

void printHelp() {
  Serial.println();
  Serial.println(F("=== Calibracion IR AttaBot (serial, 115200) ==="));
  Serial.println(F("IZQ=pin33  DER=pin27  (HW-488, LOW=detecta)   CEN=APDS9960 (prox 0..255)"));
  Serial.println(F("Comandos:  t <n> = umbral central de prueba    ?  = esta ayuda"));
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);   // que readStringUntil no trabe el loop si falta '\n'
  delay(300);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  Wire.begin();
  apdsOK = apds.begin();
  if (apdsOK) {
    apds.enableProximity(true);        // mismo modo que lee el firmware
    Serial.println(F("APDS9960 OK."));
  } else {
    Serial.println(F("APDS9960 NO detectado (revisa I2C/cableado). IZQ/DER siguen OK."));
  }
  printHelp();
  windowStart = millis();
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;
  if (line.charAt(0) == 't') {
    int sp = line.indexOf(' ');
    if (sp > 0) {
      centralThreshold = line.substring(sp + 1).toInt();
      Serial.print(F(">> umbral central de prueba = "));
      Serial.println(centralThreshold);
    }
  } else if (line.charAt(0) == '?') {
    printHelp();
  }
}

// Barra ASCII para el valor de proximidad 0..255
void proxBar(int v, int width) {
  int n = (int)((long)v * width / 255);
  Serial.print('[');
  for (int i = 0; i < width; i++) Serial.print(i < n ? '#' : ' ');
  Serial.print(']');
}

void loop() {
  handleSerial();

  bool leftDet  = (digitalRead(IR_LEFT)  == LOW);
  bool rightDet = (digitalRead(IR_RIGHT) == LOW);
  int  prox = apdsOK ? apds.readProximity() : -1;
  bool cenDet = apdsOK && (prox > centralThreshold);

  // Contar cambios de estado (chatter) de los canales digitales
  if (leftDet  != lastL) { flipL++; lastL = leftDet; }
  if (rightDet != lastR) { flipR++; lastR = rightDet; }

  unsigned long now = millis();

  // Renglón de lectura en vivo (~8 Hz)
  if (now - lastPrint >= 120) {
    lastPrint = now;
    Serial.print(F("IZQ "));  Serial.print(leftDet  ? F("DETECTA") : F(" libre "));
    Serial.print(F("   DER ")); Serial.print(rightDet ? F("DETECTA") : F(" libre "));
    Serial.print(F("   CEN prox="));
    if (apdsOK) {
      if (prox < 100) Serial.print(' ');
      if (prox < 10)  Serial.print(' ');
      Serial.print(prox);
    } else {
      Serial.print(F("---"));
    }
    Serial.print(' ');
    proxBar(apdsOK ? prox : 0, 16);
    Serial.print(cenDet ? F(" DETECTA") : F(" libre  "));
    Serial.print(F(" (umbral ")); Serial.print(centralThreshold); Serial.print(')');
    Serial.println();
  }

  // Renglón de estabilidad cada 2 s (chatter de los pots)
  if (now - windowStart >= 2000) {
    Serial.print(F("  --- estabilidad 2s:  IZQ cambios="));
    Serial.print(flipL);
    Serial.print(F("  DER cambios="));
    Serial.print(flipR);
    Serial.println(flipL + flipR == 0 ? F("   (estable)") : F("   (titila: pot en el filo)"));
    flipL = flipR = 0;
    windowStart = now;
  }
}
