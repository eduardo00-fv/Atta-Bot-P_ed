// La maquina de estados: loop() y los estados que despacha.
//
// Cada ciclo lee sensores, atiende la radio y avanza el estado en curso. Los
// estados se encolan en instructionList y READ_INSTRUCTION los va sacando.
//
// Arduino concatena todos los .ino de la carpeta en una sola unidad de
// traduccion — primero AttaBot.ino, despues el resto en orden alfabetico — asi
// que las constantes, las variables globales y las declaraciones forward viven
// en AttaBot.ino y desde aca se ven directo, sin extern ni cabeceras.

// ============================================================================
// ESTADOS DE LA FSM
// ============================================================================

// Espera pasiva de instructionValue ms. Es el unico estado donde se atiende el
// OTA, asi que el robot solo acepta una actualizacion cuando esta quieto. Al
// vencer el plazo pasa a leer la proxima instruccion, salvo que el movimiento
// anterior no haya terminado: ahi retrocede primero.
void StateWait() {
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
}

// Avanza instructionValue mm. Si un sensor ve un obstaculo a mitad de camino,
// guarda en intContext cuanto faltaba y corta: RESUME_AFTER_EVASION retoma
// justo ese resto en vez de repetir el tramo entero.
void StateMove() {
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
}

// Giro en el lugar. Con IMU disponible es lazo cerrado en yaw: acumula el giro
// real con unwrap incremental, re-apunta el arco restante en cada ciclo y al
// terminar mide el residuo y encola correcciones hasta entrar en tolerancia o
// agotar imuTurnMaxCorrections. El objetivo lo define el IMU y no la geometria
// supuesta, asi que la rueda loca, el stiction y un centerToWheelDistance mal
// calibrado dejan de producir deficit.
//
// Si el yaw deja de avanzar (IMU muda o congelada) cierra por encoders para no
// girar infinito. Sin IMU cae al giro a ciegas por distancia de rueda.
void StateTurn() {
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
    imuTurnAccumDeg += dYaw * yawScale;
    imuTurnPrevYaw   = yaw;

    if (imuTurnSettleUntil != 0) {
      // Motores ya cortados: seguir midiendo hasta que el coast termine,
      // así la verificación final incluye la inercia (la cámara mostraba
      // 2-12° de giro extra después del corte que el IMU no contaba)
      if (millis() < imuTurnSettleUntil) {
        return;
      }
      imuTurnSettleUntil = 0;

      // delta viene sin wrap, asi que sirve para arcos de mas de 180°; un error
      // positivo significa que el robot giro de menos.
      float delta = imuTurnAccumDeg;
      float error = imuTurnTargetDeg - delta;
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
        movementReady = false;
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

    imuTurnActive = false;
    imuTurnIsCorrection = false;
    imuTurnCorrCount = 0;
    state = STOP;
  }
}

// Camina al azar durante instructionValue ms, encolando tramos que arma
// SelectMovementRW. Al vencerse vuelve a leer la cola.
void StateRandomWalk() {
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
}

// Retrocede una distancia fija para despegarse de lo que sea que bloquee, antes
// de reintentar.
void StateReverse() {
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
}

// Corta los motores y decide que sigue. Si el movimiento habia terminado bien,
// vuelve a WAIT; si se corto por un obstaculo, va a identificarlo y de paso le
// manda la pose a la Base.
void StateStop() {
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
}

// Saca la proxima instruccion de la cola y salta a su estado. Con la cola vacia
// vuelve a WAIT. El bitmap de obstaculos se limpia solo si no venimos de una
// evasion ni de una instruccion interrumpida, porque en esos casos todavia hace
// falta saber que sensor disparo.
void StateReadInstruction() {
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
}

// Le avisa a la Base que termino la secuencia de instrucciones.
void StateMessageBase() {
  const char *message = "";
  if (instructionValue == 1) {
    message = "READY";
  }

  SendMessage(robots["Base"], message);
  state = WAIT;
  instructionValue = instructionCompletedDelay;
}

// Decide si lo que bloquea es otro robot o un obstaculo del escenario. Espera
// obstacleWaitTime a que llegue un OBSTACLE_DETECTED de algun companero: si
// llega, alcanza con esperar a que se aparte; si no, es fijo y hay que evadir.
void StateIdentifyObstacle() {
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
}

// Le pide la pose a la Base y espera el POSITION_RESPONSE para dar el siguiente
// paso de navegacion. Sin IP de base aborta, y si la respuesta no llega a
// tiempo reintenta, porque un paquete UDP perdido no puede colgar la corrida.
void StateRequestPosition() {
  if (robots.find("Base") == robots.end() ||
      robots["Base"] == IPAddress(0, 0, 0, 0)) {
    MessageDebugf("DEBUG: -1, ID: %s, No hay IP de base — abortando nav",
                  robotID.c_str());
    congregation.CompleteRequest();
    state = WAIT;
    instructionValue = 500;
    return;
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
    return;
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
}

// Maniobra de evasion. Elige el rodeo segun el patron de bits de los sensores y
// encola retroceso, giro y avance. Tras varias evasiones seguidas en el mismo
// punto asume deadlock y hace un escape comprometido hacia el interior de la
// arena, en vez del giro de 180° a ciegas que dejaba al robot rebotando contra
// la misma esquina.
void StateActiveEvasion() {
  if (!obstacles.HasAnyObstacle()) {
    // Sin obstáculo real — puede haber desaparecido entre detección y aquí
    MessageDebugf(
        "DEBUG: -1, ID: %s, ACTIVE_EVASION sin obstáculo. Abortando.",
        robotID.c_str());
    obstacles.Clear();
    state = READ_INSTRUCTION;
    return;
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
    return;
  }

  unsigned long timeSinceDetection = millis() - evasionStartTime;
  if (timeSinceDetection > 1500) {
    MessageDebugf("DEBUG: -1, ID: %s, Datos de obstáculo obsoletos (%lums). "
                  "Re-escaneando.",
                  robotID.c_str(), timeSinceDetection);
    state = STOP;
    return;
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
      return;
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
}

// Retoma la instruccion que la evasion interrumpio, con el resto que quedo
// guardado en intContext, y limpia el rastro de la evasion.
void StateResumeAfterEvasion() {
  if (!intContext.wasInterrupted) {
    resumeScheduled = false;
    obstacles.Clear();
    state = READ_INSTRUCTION;
    return;
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
}

// Aproximacion lenta al candidato hasta el alcance del APDS9960, unos pocos cm,
// que es donde la lectura de color es confiable. Al llegar se detiene, espera
// una lectura valida y clasifica; si el color no era el buscado o no alcanza a
// llegar, evade y vuelve a patrullar. Validado en Webots.
void StateSearchApproach() {
  if (!search.active || !frontSensorInitialized) {
    ConfigureHBridge(0, 0);
    state = READ_INSTRUCTION;
    return;
  }

  if (millis() - search.approachStart > SEARCH_APPROACH_TIMEOUT) {
    ConfigureHBridge(0, 0);
    MessageDebugf("DEBUG: -1, ID: %s, SEARCH: aproximación agotada — evadiendo",
                  robotID.c_str());
    SearchEvadeAndResume();
    return;
  }

  // Todavía lejos: seguir avanzando lento, los motores ya están configurados.
  if (frontSensor.readProximity() < SEARCH_PROX_NEAR) {
    return;
  }

  // Al alcance: detenerse y esperar una lectura de color válida (~100ms)
  ConfigureHBridge(0, 0);
  if (!frontSensor.colorDataReady()) {
    return;
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
}

// ============================================================================
// LOOP PRINCIPAL
// ============================================================================

// Housekeeping de cada ciclo y despacho del estado en curso.
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

  // Telemetría del EKF (2Hz). Es solo salida: no toca el control, corra o no
  // EKF_NAV. Sirve para responder con datos "¿cuánto deriva el EKF?" durante las
  // corridas que ya se hacen, en vez de tener que confiar en él sin medirlo.
  if (ekf.initialized && millis() - lastEkfReport >= ekfReportInterval &&
      robots.find("Base") != robots.end() &&
      robots["Base"] != IPAddress(0, 0, 0, 0)) {
    lastEkfReport = millis();
    char ekfBuf[64];
    snprintf(ekfBuf, sizeof(ekfBuf), "EKF_POSE|%.1f|%.1f|%.1f", ekf.x, ekf.y,
             ekf.AngleDeg());
    SendMessage(robots["Base"], ekfBuf);
  }

#ifdef DebugSerial
  ReadSerialCommands();
#endif

  switch (state) {
  case WAIT:
    StateWait();
    break;
  case MOVE:
    StateMove();
    break;
  case TURN:
    StateTurn();
    break;
  case RANDOM_WALK:
    StateRandomWalk();
    break;
  case REVERSE:
    StateReverse();
    break;
  case STOP:
    StateStop();
    break;
  case READ_INSTRUCTION:
    StateReadInstruction();
    break;
  case MESSAGE_BASE:
    StateMessageBase();
    break;
  case IDENTIFY_OBSTACLE:
    StateIdentifyObstacle();
    break;
  case REQUEST_POSITION:
    StateRequestPosition();
    break;
  case ACTIVE_EVASION:
    StateActiveEvasion();
    break;
  case RESUME_AFTER_EVASION:
    StateResumeAfterEvasion();
    break;
  case SEARCH_APPROACH:
    StateSearchApproach();
    break;
  }
}

// ============================================================================
// FUNCIONES AUXILIARES
// ============================================================================

// Sortea el proximo tramo del random walk: girar a un lado, al otro o avanzar,
// con las probabilidades y las tablas de angulos y distancias de arriba. Si hay
// un obstaculo detectado, la probabilidad de avanzar se pone en cero.
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
// Consola serie de diagnostico, solo con DebugSerial activo.
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
