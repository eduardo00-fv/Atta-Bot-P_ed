// Perifericos de a bordo: LED RGB de estado, IMU ICM-20948 y el EKF que
// fusiona sus lecturas con los encoders.
//
// El EKF reporta su pose a la Base para poder medir cuanto deriva contra el
// ArUco. Con la camara contestando no controla la navegacion, salvo que se
// encienda EKF_NAV; cuando deja de contestar es la pose de respaldo con la que
// el robot sigue navegando (StateRequestPosition).
//
// Arduino concatena todos los .ino de la carpeta en una sola unidad de
// traduccion — primero AttaBot.ino, despues el resto en orden alfabetico — asi
// que las constantes, las variables globales y las declaraciones forward viven
// en AttaBot.ino y desde aca se ven directo, sin extern ni cabeceras.

// ============================================================================
// FUNCIONES DE LED
// ============================================================================

// Refresca el LED. El parpadeo se hace por tiempo y no con delay, para no
// bloquear el ciclo.
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

// Color fijo.
void setLedColor(uint8_t red, uint8_t green, uint8_t blue) {
  ledCtrl.setSolid(red, green, blue, maxBrightness);
}

// Brillo global del LED.
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

// Arranca la ICM-20948 y su DMP, restaura los sesgos guardados y deja
// imuAvailable en true solo si todo el camino salio bien. Si falla, el robot
// sigue funcionando: los giros caen al modo a ciegas por encoders.
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
    return;
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

  // Restaurar la calibración guardada. El namespace se abre de solo lectura.
  biasStore store;
  preferences.begin("attabot-config", true);
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

// Lee el yaw del DMP y actualiza la variable global. Es la unica fuente de
// orientacion inercial del firmware.
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
