// Control de motores: PID de velocidad por rueda, puente H y el movimiento
// por distancia que consume la FSM.
//
// Arduino concatena todos los .ino de la carpeta en una sola unidad de
// traduccion — primero AttaBot.ino, despues el resto en orden alfabetico — asi
// que las constantes, las variables globales y las declaraciones forward viven
// en AttaBot.ino y desde aca se ven directo, sin extern ni cabeceras.

// ============================================================================
// FUNCIONES DE CONTROL DE MOTORES
// ============================================================================

// Reinicia los dos controladores y los contadores de pulsos. Hay que llamarla
// entre instrucciones, o el error acumulado de la anterior se arrastra.
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
  static unsigned long prevResetEpoch = 0;

  if (!yawInit) {
    prevYaw = yaw;
    yawInit = true;
    return;
  }

  float dYaw = yaw - prevYaw;
  if (dYaw >  180.0f) dYaw -= 360.0f;
  if (dYaw < -180.0f) dYaw += 360.0f;
  prevYaw = yaw;

  // Los encoders son de cuadratura CON SIGNO (la ISR suma o resta segun la
  // transicion), asi que el delta de pulsos ya trae la direccion: retroceder da
  // un delta negativo y hay que respetarlo tal cual. La unica discontinuidad
  // real es ResetPID() poniendo los contadores en cero, y eso se detecta con el
  // epoch, no por el signo. Inferirlo del signo hacia que cada tick de un
  // REVERSE re-sumara el acumulado entero — y con el factor -1 encima, el
  // retroceso entraba como avance: ~400mm fantasma por evasion.
  float avg = (movement.pastLeftPulseCount + movement.pastRightPulseCount) / 2.0f;
  if (movement.resetEpoch != prevResetEpoch) {
    prevResetEpoch = movement.resetEpoch;
    prevAvgPulses = 0.0f;
  }

  // En TURN las ruedas giran en sentidos opuestos: el promedio ya da ~0, pero se
  // descarta explicitamente para que el ruido del giro no entre como traslacion.
  float d = 0.0f;
  if (state == MOVE || state == REVERSE) {
    d = (avg - prevAvgPulses) * millimetersPerPulse;
  }
  prevAvgPulses = avg;

  ekf.Predict(d, dYaw * yawScale);
}

// Clasifica una lectura RGBC del APDS9960 contra un color objetivo. Un canal
// claro por debajo de 10 se descarta: está muy oscuro para que la proporción
// entre canales signifique algo. Umbrales de primera pasada, hay que
// calibrarlos en el lab con COLOR_READ.
bool MatchColor(const char *target, uint16_t r, uint16_t g, uint16_t b,
                uint16_t c) {
  if (c < 10) return false;
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

// Aplica el PWM con signo a cada rueda, eligiendo el par de pines segun el
// sentido. Con cero en ambos, frena.
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

// Velocidad objetivo para lo que falta del tramo. Va a maxSpeed casi todo el
// camino y baja en rampa hasta minSpeed en los ultimos speedReductionThreshold
// mm, para no pasarse de largo. El signo indica el sentido.
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

// Un paso del control de movimiento: mide lo recorrido por cada rueda, corre el
// PID contra la velocidad objetivo y aplica el PWM. Devuelve true cuando ambas
// ruedas llegaron o cuando el robot lleva SteadyStateTime sin moverse, que es
// la salida por atasco.
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
