// Protocolo con la Base: envio de mensajes y despacho de los comandos UDP.
//
// ReadUdpPackets() es el punto de entrada: parsea el datagrama, separa el
// comando de sus argumentos y ejecuta. Todo lo que la Base puede pedirle al
// robot entra por aca.
//
// Arduino concatena todos los .ino de la carpeta en una sola unidad de
// traduccion — primero AttaBot.ino, despues el resto en orden alfabetico — asi
// que las constantes, las variables globales y las declaraciones forward viven
// en AttaBot.ino y desde aca se ven directo, sin extern ni cabeceras.

// ============================================================================
// FUNCIONES DE COMUNICACIÓN
// ============================================================================

// Manda un datagrama UDP al host indicado, en el puerto del protocolo.
void SendMessage(IPAddress host, const char *message) {
  udp.beginPacket(host, localPort);
  udp.write(reinterpret_cast<const uint8_t *>(message), strlen(message));
  udp.endPacket();
}

// printf de depuracion hacia la Base. Solo sale si el modo debug esta activo
// (CONFIG|DEBUG|n), asi que las trazas se prenden sin reflashear.
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

// Le reporta a la Base la pose que el robot cree tener.
void SendPose() {
  obstacles.robotDetected = false;
  const char *message = "CHECK_OBSTACLE|%d|%.1f|%.1f|%.1f";
  char buffer[40];
  snprintf(buffer, sizeof(buffer), message, obstacles.obstacleSensors,
           robotPose.x, robotPose.y, robotPose.angle);
  SendMessage(robots["Broadcast"], buffer);
  movement.previousMillis = millis();
}

// Parte un mensaje del protocolo por su delimitador. Devuelve siempre seis
// campos, con vacios al final si vinieron menos, para que los handlers puedan
// leer argumentos opcionales sin chequear la cantidad.
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
// FUNCIÓN DE LECTURA DE PAQUETES UDP (REFACTORIZADA)
// ============================================================================

  // CONFIG
void HandleConfig(const std::array<String, 6> &arguments) {
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
    ekf.Reset();
    blindNavSteps = 0;
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
void HandleMovementCommand(const std::array<String, 6> &arguments, const String &command) {
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
void HandleReset(const std::array<String, 6> &arguments) {
  ESP.restart();
}

  // PID
void HandlePID(const std::array<String, 6> &arguments) {
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
void HandleKalmanPID(const std::array<String, 6> &arguments) {
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
void HandlePose(const std::array<String, 6> &arguments) {
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
void HandleSetPPR(const std::array<String, 6> &arguments) {
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
void HandleGetPPR(const std::array<String, 6> &arguments) {
  char buffer[100];
  snprintf(buffer, sizeof(buffer),
           "Robot %s - PPR actual: %.2f, Chip ID: %04X%08X", robotID.c_str(),
           pulsesPerRev, (uint16_t)(ESP.getEfuseMac() >> 32),
           (uint32_t)ESP.getEfuseMac());
  SendMessage(robots["Base"], buffer);
}

  // CHECK_OBSTACLE
void HandleCheckObstacle(const std::array<String, 6> &arguments) {
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
void HandleObstacleDetected(const std::array<String, 6> &arguments) {
  obstacles.fromRobotID = arguments[1];
  obstacles.robotDetected = true;
}

  // COUNT_MESSAGE
void HandleCountMessage(const std::array<String, 6> &arguments) {
  countMessages++;
}

  // SEND_COUNT_MESSAGE
void HandleSendCountMessage(const std::array<String, 6> &arguments) {
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
void HandleMeet(const std::array<String, 6> &arguments) {
  congregation.leaderID = "VIRTUAL";
  congregation.isLeader = false;
  congregation.hasGlobalTarget = true;
  congregation.globalTargetX = arguments[1].toFloat();
  congregation.globalTargetY = arguments[2].toFloat();

  // n sale de los vecinos que este robot conoce; el anillo, de la SEPARACIÓN
  // que se quiere entre slots vecinos: R = sep / (2·sin(π/n)) es el radio con
  // el que la cuerda entre dos slots contiguos mide exactamente MEET_SEP.
  //
  // Antes era max(300, n·250/2π): arco por robot (no cuerda) y un piso de
  // 300mm que con pocos robots abría el grupo de gordo — con 4 robots pedía
  // 159mm y el piso lo subía a 300, dejándolos a 424mm entre sí. Con muchos
  // robots ambas fórmulas coinciden (n=10: 398 vs 405mm), así que esto solo
  // aprieta la congregación chica, que es donde sobraba radio.
  //
  // MEET|x|y|R fuerza el radio (mismo rango que NAV_CONFIG|PARKING_DIST) para
  // tantear en vivo sin reflashear; sin el 3er argumento manda la fórmula.
  // Separacion centro a centro entre slots vecinos, y piso del radio: 150mm de
  // cuerpo mas el error de pose.
  const float MEET_SEP  = 250.0f;
  const float MEET_RMIN = 200.0f;
  int n = disperse.nCount + 1;
  if (n > DisperseState::MAX_NEIGHBORS + 1) n = DisperseState::MAX_NEIGHBORS + 1;
  float ring = (n >= 2) ? MEET_SEP / (2.0f * sinf(PI / n)) : MEET_RMIN;
  if (ring < MEET_RMIN) ring = MEET_RMIN;
  if (arguments[3] != "") {
    float forced = arguments[3].toFloat();
    if (forced >= 150.0f && forced <= 600.0f) ring = forced;
  }
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
  blindNavSteps = 0;
  instructionList.clear();

  MessageDebugf("DEBUG: -1, ID: %s, MEET en (%.0f,%.0f), slot %d/%d, anillo %.0fmm",
                robotID.c_str(), congregation.globalTargetX,
                congregation.globalTargetY, congregation.followerIndex,
                congregation.totalFollowers, ring);

  fsmInstruction[0] = WAIT;
  fsmInstruction[1] = robotID.toInt() * 200;
  instructionList.push_back(fsmInstruction);
  fsmInstruction[0] = REQUEST_POSITION;
  fsmInstruction[1] = 0;
  instructionList.push_back(fsmInstruction);
}

// CONGREGATION|<lider>|<indice>|<total> — arranca la congregacion sobre el
// anillo del lider. Con VIRTUAL en vez de un id, el punto de reunion es fijo y
// no hay nadie difundiendo pose.
void HandleCongregation(const std::array<String, 6> &arguments) {
  congregation.leaderID = arguments[1];
  congregation.isLeader = (congregation.leaderID == robotID);
  congregation.positionReceived = false;
  congregation.hasGlobalTarget = false;
  congregation.stagingDone = false;
  congregation.slotAngleSet = false;
  congregation.poseFresh = false;
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
void HandleFormation(const std::array<String, 6> &arguments) {
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
  disperse.Reset();

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

  // GT — Navegación reactiva al objetivo
  // Uso: GT|x|y          — navega al objetivo
  //      GT|x|y|seg      — ídem con segmento personalizado (50–400mm)
  //
  // Hasta 2026-08-05 esto mismo respondía además a GOTO, POSITIONGT y BUG2.
  // No eran variantes: los cuatro nombres caían en este handler y el nombre ni
  // siquiera llegaba acá, así que elegir uno u otro no cambiaba nada. BUG2
  // sobrevivía de cuando existió ese algoritmo, y se quedó como alias cuando se
  // borró el código muerto. Cuatro formas de pedir lo mismo obligan a leer el
  // firmware para descubrir que dan igual.
void HandleNavigationTarget(const std::array<String, 6> &arguments) {
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
  blindNavSteps = 0;
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
void HandlePositionResponse(const std::array<String, 6> &arguments) {
  // Ignorar respuestas no solicitadas (paquetes residuales de sesiones anteriores)
  if (!congregation.waitingForResponse && !nav.pendingInit) {
    MessageDebugf("DEBUG: -1, ID: %s, POSITION_RESPONSE ignorado (no esperado)",
                  robotID.c_str());
    return;
  }
  robotPose.x = arguments[1].toFloat();
  robotPose.y = arguments[2].toFloat();
  robotPose.angle = arguments[3].toFloat();
  congregation.poseFresh = true;

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
void HandleLeaderPosition(const std::array<String, 6> &arguments) {
  String receivedLeaderID = arguments[1];

  if (!congregation.isLeader && receivedLeaderID == congregation.leaderID) {
    UpdateCongregationGoal(arguments[2].toFloat(), arguments[3].toFloat(),
                           arguments[4].toFloat());
  }
}

  // CANCEL_CONGREGATION — también termina la dispersión (mismo "alto enjambre")
void HandleCancelCongregation(const std::array<String, 6> &arguments) {
  congregation.Reset();
  disperse.Reset();
  nav.Reset();
  instructionList.clear();
  state = STOP;
  MessageDebugf("DEBUG: -1, ID: %s, Congregación cancelada", robotID.c_str());
}

  // DISPERSE — dispersión de enjambre: repeler vecinos hasta separación >= mm
  // DISPERSE|<mm>. Los vecinos llegan por NEIGHBOR_POSITIONS (1 Hz de la Base).
void HandleDisperse(const std::array<String, 6> &arguments) {
  congregation.Reset();
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
void HandleNeighborPositions(const std::array<String, 6> &arguments) {
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
        disperse.selfX    = nx;
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
void HandleClearEvasion(const std::array<String, 6> &arguments) {
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
void HandleNavConfig(const std::array<String, 6> &arguments) {
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
void HandleSensorMask(const std::array<String, 6> &arguments) {
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
void HandleSensorThreshold(const std::array<String, 6> &arguments) {
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
void HandleSelftest(const std::array<String, 6> &arguments) {
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

    // Fase 0 mueve solo la rueda izquierda, 1 solo la derecha y 2 las dos. El
    // yaw se relee durante el tramo para mantenerlo fresco, y despues del corte
    // se espera a que frene antes de medir.
    int lp = (fase == 1) ? 0 : pwm;
    int rp = (fase == 0) ? 0 : pwm;
    ConfigureHBridge(lp, rp);
    unsigned long t0 = millis();
    while (millis() - t0 < STEP_MS) {
      LeerYaw();
      delay(10);
    }
    ConfigureHBridge(0, 0);
    delay(400);
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
void HandleEKFNav(const std::array<String, 6> &arguments) {
  ekfNavEnabled = (arguments[1].toInt() != 0);
  char buf[80];
  snprintf(buf, sizeof(buf), "EKF_NAV: nav con pose %s%s",
           ekfNavEnabled ? "EKF" : "ArUco",
           (ekfNavEnabled && !ekf.initialized) ? " (EKF aún sin inicializar)" : "");
  SendMessage(robots["Base"], buf);
}

  // GET_STATUS
void HandleGetStatus(const std::array<String, 6> &arguments) {
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

// RESET_EVASION — borra a mano todo el rastro de evasion. Escotilla de
// emergencia para cuando un robot queda trabado creyendo que sigue evadiendo.
void HandleResetEvasion(const std::array<String, 6> &arguments) {
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
void HandleSearchObject(const std::array<String, 6> &arguments) {
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
  fsmInstruction[1] = 600000;
  instructionList.push_back(fsmInstruction);
  state = READ_INSTRUCTION;
  MessageDebugf("DEBUG: -1, ID: %s, SEARCH: buscando objeto %s",
                robotID.c_str(), search.targetColor);
}

  // COLOR_READ — lectura puntual RGBC para calibrar umbrales de color en lab
void HandleColorRead(const std::array<String, 6> &arguments) {
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

// ABORT_NAV — corta cualquier navegacion o busqueda en curso y deja el robot
// quieto.
void HandleAbortNav(const std::array<String, 6> &arguments) {
  nav.Reset();
  if (search.active && frontSensorInitialized) frontSensor.enableColor(false);
  search.Reset();
  instructionList.clear();
  // Si se aborto a mitad de un giro, el seguimiento por IMU no puede quedar
  // activo: el proximo TURN tiene que reinicializar yaw de arranque, acumulado y
  // objetivo desde cero.
  imuTurnActive       = false;
  imuTurnIsCorrection = false;
  imuTurnCorrCount    = 0;
  state = STOP;
  SendMessage(robots["Base"], "GT abortado");
  MessageDebugf("DEBUG: -1, ID: %s, Navegación abortada manualmente",
                robotID.c_str());
}

  // GET_YAW — retorna el yaw actual de la IMU para validación en Fase 2
  // Uso desde la base: BASE.GET_YAW → responde YAW|<valor>|<imuAvailable>
void HandleGetYaw(const std::array<String, 6> &arguments) {
  char buffer[60];
  snprintf(buffer, sizeof(buffer), "YAW|%.2f|%d|%.3f",
           yaw, (int)imuAvailable, imuGravity);
  SendMessage(robots["Base"], buffer);
}

// Punto de entrada del protocolo: lee un datagrama, lo parte en comando y
// argumentos, y despacha al handler que corresponda.
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

  if (command == "CONFIG") HandleConfig(arguments);
  else if (command == "MOVE" || command == "TURN" || command == "WAIT" ||
           command == "RANDOMW" || command == "MESSAGE_BASE")
    HandleMovementCommand(arguments, command);
  else if (command == "RESET") HandleReset(arguments);
  else if (command == "PID") HandlePID(arguments);
  else if (command == "KFPID") HandleKalmanPID(arguments);
  else if (command == "POSE") HandlePose(arguments);
  else if (command == "SETPPR") HandleSetPPR(arguments);
  else if (command == "GETPPR") HandleGetPPR(arguments);
  else if (command == "CHECK_OBSTACLE") HandleCheckObstacle(arguments);
  else if (command == "OBSTACLE_DETECTED") HandleObstacleDetected(arguments);
  else if (command == "COUNT_MESSAGE") HandleCountMessage(arguments);
  else if (command == "SEND_COUNT_MESSAGE") HandleSendCountMessage(arguments);
  else if (command == "MEET") HandleMeet(arguments);
  else if (command == "CONGREGATION") HandleCongregation(arguments);
  else if (command == "FORMATION") HandleFormation(arguments);
  else if (command == "GT") HandleNavigationTarget(arguments);
  else if (command == "POSITION_RESPONSE") HandlePositionResponse(arguments);
  else if (command == "LEADER_POSITION") HandleLeaderPosition(arguments);
  else if (command == "CANCEL_CONGREGATION") HandleCancelCongregation(arguments);
  else if (command == "DISPERSE") HandleDisperse(arguments);
  else if (command == "NEIGHBOR_POSITIONS") HandleNeighborPositions(arguments);
  else if (command == "CLEAR_EVASION") HandleClearEvasion(arguments);
  else if (command == "NAV_CONFIG") HandleNavConfig(arguments);
  else if (command == "SENSOR_MASK") HandleSensorMask(arguments);
  else if (command == "SENSOR_THRESHOLD") HandleSensorThreshold(arguments);
  else if (command == "SELFTEST") HandleSelftest(arguments);
  else if (command == "EKF_NAV") HandleEKFNav(arguments);
  else if (command == "GET_STATUS") HandleGetStatus(arguments);
  else if (command == "RESET_EVASION") HandleResetEvasion(arguments);
  else if (command == "SEARCH_OBJECT") HandleSearchObject(arguments);
  else if (command == "COLOR_READ") HandleColorRead(arguments);
  else if (command == "ABORT_NAV") HandleAbortNav(arguments);
  else if (command == "GET_YAW") HandleGetYaw(arguments);
}
