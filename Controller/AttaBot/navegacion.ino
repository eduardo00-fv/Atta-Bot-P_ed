// Navegacion reactiva y congregacion.
//
// Un solo navegador sirve a GT, a la congregacion y a las formaciones: apunta
// al objetivo, lo sesga si los IR ven un obstaculo y encola el paso. El slot
// de cada robot se calcula LOCALMENTE a partir de datos que todos comparten,
// de modo que el reparto es identico en los N robots sin negociar nada.
//
// Arduino concatena todos los .ino de la carpeta en una sola unidad de
// traduccion — primero AttaBot.ino, despues el resto en orden alfabetico — asi
// que las constantes, las variables globales y las declaraciones forward viven
// en AttaBot.ino y desde aca se ven directo, sin extern ni cabeceras.

// ============================================================================
// NAVEGACIÓN REACTIVA UNIFICADA — GT y Congregación
// ============================================================================

// Ángulo del slot del anillo de congregación, seguro contra paredes y sin
// colisiones entre slots. Determinista con datos que TODOS los seguidores
// comparten (pose del líder, n, arena vía NAV_CONFIG|ARENA): cada uno calcula
// el MISMO anillo y toma el ángulo de su índice, así que la repartición es
// descentralizada y no hace falta negociar.
//
// Por qué no corregir solo el ángulo propio: la versión greedy rotaba cada slot
// invasor hacia el lado libre por separado y los ENCIMABA. Medido el 2026-07-27
// con el líder a 311mm de la pared, 5 de 9 slots quedaron a 31-62mm entre sí y
// el enjambre nunca asentó. Acá los n slots se reparten parejos sobre el arco
// seguro más largo, y si ese arco no alcanza para n cuerpos (MIN_ARC cada uno)
// el radio crece hasta que sí.
//
// Con n==1 (useBearing) se respeta el bearing líder→robot si cae dentro del
// arco, que es el recorrido mínimo; si no, se va al extremo más cercano.
// Escribe el radio efectivo en *outR, que puede ser mayor que el nominal.
float SafeRingSlotAngle(float lx, float ly, int idx, int n, float nominalR,
                        float bearing, bool useBearing, float *outR) {
  const float MARGIN = 200.0f;
  const float MIN_ARC = 250.0f;
  const float growth[6] = {1.0f, 1.2f, 1.4f, 1.7f, 2.0f, 2.5f};
  const int NS = 72;
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
    if (nSafe == NS) {
      *outR = R;
      return fallback;
    }
    if (nSafe == 0) continue;

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

// Agrega la pose actual del líder a la ventana móvil y devuelve la promediada
// por referencia. La cámara le mete σ≈10mm y σ≈3.8° (medido el 2026-07-27 sobre
// un robot quieto) y ese ruido se propagaba tal cual al goal de cada seguidor.
// El ángulo se promedia por seno/coseno para no romperse en el wrap de 360°. Si
// el líder se movió de verdad, con un salto mayor a LEADER_SMOOTH_RESET, la
// ventana se reinicia: así el suavizado no introduce retardo justo cuando hace
// falta seguirlo.
void SmoothLeaderPose(float xIn, float yIn, float angIn,
                      float *xOut, float *yOut, float *angOut) {
  if (leaderSmCount > 0) {
    float ax = 0, ay = 0;
    for (int i = 0; i < leaderSmCount; i++) { ax += leaderSmX[i]; ay += leaderSmY[i]; }
    ax /= leaderSmCount;  ay /= leaderSmCount;
    if (CalculateDistance(xIn, yIn, ax, ay) > LEADER_SMOOTH_RESET) {
      leaderSmCount = 0;  leaderSmIdx = 0;
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

// ¿Ya hay un REQUEST_POSITION esperando en la cola? El guard
// congregation.waitingForResponse solo se levanta cuando la FSM EJECUTA la
// instrucción, no cuando se encola, y entre ambos momentos puede haber cientos
// de ms (el WAIT de arranque de CONGREGATION). Sin este chequeo se encolaban dos
// pedidos, llegaban dos POSITION_RESPONSE, corrían dos ReactiveNavStep y la cola
// quedaba con un ciclo duplicado: de ahí en más el robot ejecutaba giros
// calculados para una pose vieja (medido el 2026-07-27, giros un ciclo atrasados).
bool RequestPositionQueued() {
  for (const auto &ins : instructionList) {
    if ((int)ins[0] == REQUEST_POSITION) return true;
  }
  return false;
}

// Recalcula el slot de congregación y el waypoint de aproximación a partir de la
// pose del líder. La usan DOS caminos: LEADER_POSITION, donde hay un robot líder
// que difunde su pose, y CONGREGATION|VIRTUAL, un punto fijo sin nadie que
// difunda. Vive extraída para que las dos formas de congregar no se
// desincronicen al tocar una sola.
//
// El robot va primero a un waypoint de STAGING y solo después al slot, de modo
// que la recta al objetivo nunca cruce el círculo de parking ni al líder. Cómo
// se arma ese par depende de la forma pedida:
//
//   linea, cuna  Slot perpendicular al heading del líder, con eje opcional de la
//                Base (formationAxis). La cuna además lo desplaza k·s hacia
//                atrás, formando la V detrás del líder. El staging va POR DETRÁS
//                de la fila, opuesto al heading, para que cada robot entre por su
//                propio carril y no cruce los slots vecinos.
//
//   circulo      Slot sobre el anillo wall-safe determinista que arma
//                SafeRingSlotAngle, con aproximación radial: el waypoint queda
//                STAGING_MARGIN más lejos sobre el mismo rayo.
//
// El caso n==1 del círculo saca el slot del bearing líder→ROBOT, así que exige
// una pose propia fresca. LEADER_POSITION suele llegar ANTES del primer
// POSITION_RESPONSE; latchear ahí anclaba el slot a una lectura vieja y el robot
// salía al lado contrario sin recalcular nunca (2026-07-27). Sin pose fresca se
// pide una y se espera al próximo LEADER_POSITION, que llega a 4Hz.
void UpdateCongregationGoal(float leaderX, float leaderY, float leaderAngle) {
  const float STAGING_MARGIN = 150.0f;
  int    n     = max(1, congregation.totalFollowers);
  String shape = congregation.formationShape;
  float  parkX, parkY;

  if (shape == "linea" || shape == "cuna") {
    float s   = congregation.parkingDist;
    float rad = leaderAngle * PI / 180.0f;
    float hx  = cos(rad), hy = sin(rad);
    float pa  = rad + PI / 2.0f + congregation.formationAxis * PI / 180.0f;
    float px  = cos(pa), py = sin(pa);
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
      float bearing = atan2(robotPose.y - leaderY, robotPose.x - leaderX);
      float effR;
      congregation.slotAngle = SafeRingSlotAngle(
          leaderX, leaderY, congregation.followerIndex, n,
          congregation.parkingDist, bearing, n == 1, &effR);
      congregation.slotRadius = effR;
      congregation.slotAngleSet = true;

      // El staging se saltea solo si el slot quedó SOBRE el rayo líder→robot con
      // el radio nominal, porque ahí la recta al slot no cruza al líder. Si el
      // anillo se corrió o creció, hace falta la aproximación radial.
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

  // Banda muerta al reubicar el goal: LEADER_POSITION llega a ~3Hz con la pose
  // CRUDA del líder, así que sin esto el objetivo se corría unos pocos mm en
  // cada mensaje y el navegador terminaba re-apuntando contra el ruido en vez
  // de contra el movimiento real del líder.
  if (nav.isActive) {
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

// Ejecuta un paso de navegación hacia (nav.goalX, nav.goalY): calcula el ángulo
// al objetivo, le aplica bias reactivo si hay obstáculo en los sensores IR y
// encola TURN+WAIT+MOVE+WAIT+REQUEST_POSITION. Se llama desde el handler de
// POSITION_RESPONSE cuando nav.isActive, y desde StateRequestPosition cuando la
// cámara no contestó y se sigue con la pose del EKF.
//
// La fuente de pose depende de EKF_NAV: con EKF_NAV|1 se navega con el estado
// fusionado (encoders+gyro+ArUco) en vez del ArUco crudo, porque el rumbo del
// EKF no trae el σ≈3.8° de la cámara, que es lo que disparaba los giros
// espurios.
void ReactiveNavStep() {
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
    bias    = nav.avoidSideAngle;
    seg     = nav.avoidSegment;
    avoiding = true;
  } else if (leftBlocked) {
    bias    = -nav.avoidSideAngle;
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

  // El orden por id es lo que fija el desempate y hace que los N robots lleguen
  // a la misma reparticion.
  for (int a = 0; a < count - 1; a++) {
    for (int b = a + 1; b < count; b++) {
      if (id[b] < id[a]) {
        int   ti = id[a]; id[a] = id[b]; id[b] = ti;
        float t  = px[a]; px[a] = px[b]; px[b] = t;
        t = py[a];        py[a] = py[b]; py[b] = t;
      }
    }
  }

  float sx[MAXN], sy[MAXN];
  for (int s = 0; s < count; s++) {
    float ang = 2.0f * PI * s / count;
    sx[s] = tx + ring * cosf(ang);
    sy[s] = ty + ring * sinf(ang);
  }

  bool robotDone[MAXN] = {false};
  bool slotDone[MAXN]  = {false};
  // asg[robot] = slot. Greedy: gana el par mas corto, y el que llega se queda.
  int  asg[MAXN];
  for (int k = 0; k < count; k++) asg[k] = k;
  for (int k = 0; k < count; k++) {
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

// Un paso de dispersión de enjambre, port 1:1 de maybe_disperse_hop() del
// controller de sim validado en Webots. Se llama al recibir NEIGHBOR_POSITIONS,
// a 1Hz. Solo actúa si la dispersión está activa, el robot está OCIOSO (sin
// navegación ni instrucciones en cola) y conoce vecinos; un robot ocupado
// espera al siguiente tick para reevaluar. Los saltos se acotan a DISP_ARENA
// para que ninguno apunte a la pared.
void MaybeDisperseHop() {
  if (!disperse.IsActive() || nav.isActive || nav.pendingInit ||
      !instructionList.empty() || disperse.nCount == 0) {
    return;
  }
  float x = robotPose.x, y = robotPose.y;

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
  // Norma nula = el robot esta justo encima del vecino y no hay direccion de
  // huida definida; se sortea una sobre [0, 2π).
  float norm = sqrt(vx * vx + vy * vy);
  if (norm < 1e-9f) {
    float ang = random(0, 62832) / 10000.0f;
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

// Decide si el obstaculo que ven los sensores es el companero que acaba de
// reportar su pose. Compara distancia y angulo relativo, y para los sensores
// laterales exige ademas que el otro robot este del lado que disparo.
bool IsRobotObstacle(float x2, float y2, float angle, int sensors, String id) {
  float deltaX = x2 - robotPose.x;
  float deltaY = y2 - robotPose.y;

  float distanceBetweenRobots = sqrt(deltaX * deltaX + deltaY * deltaY);
  if (distanceBetweenRobots > robotDistanceMargin) {
    return false;
  }

  float angleBetweenRobots = atan2f(deltaY, deltaX) * RAD_TO_DEG + 180;
  float angleDifference = angleBetweenRobots - angle;
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
