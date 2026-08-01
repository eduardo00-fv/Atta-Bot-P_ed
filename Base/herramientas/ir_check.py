#!/usr/bin/env python3
"""
ir_check.py — Banco de prueba EN VIVO de los sensores IR de un AttaBot.

Complementa a `analyze_logs.py --ir` (que es post-hoc sobre logs). Esta
herramienta habla UDP directo con el robot y muestra el estado L/C/R en tiempo
real, para que puedas mover la mano / poner una caja frente a cada sensor y ver
al instante cuál dispara. Sirve para:
  - VERIFICAR que cada canal responde (no está muerto ni desalineado),
  - DETECTAR fantasmas (dispara sin nada enfrente → reflejo del chasis),
  - RECOMENDAR / aplicar SENSOR_MASK por canal.

Usa los comandos del firmware —
  GET_STATUS   → responde  ...|Sensors:L?-C?-R?|Prox:N|Thr:N|Mask:L?-C?-R?|...
                 (Prox/Thr/Mask sólo en firmware ≥ 07/2026; `Prox` = lectura CRUDA
                  0..255 del APDS9960 central, la que se compara contra el umbral)
  SENSOR_MASK|L/C/R|0/1        → des/enmascara un canal (1 = ignorar).
  SENSOR_THRESHOLD|C|N[|SAVE]  → umbral del central (SAVE lo persiste en NVS).

IMPORTANTE — la base debe estar CERRADA:
  El robot responde siempre a  Base-IP:6060, así que este tool toma el puerto
  6060. Si la base sigue corriendo, cerrala (BREAK) antes de usar ir_check.

Canales:  IZQ = L (IR izq, pin 33) · CEN = C (APDS9960) · DER = R (IR der, pin 27)
Los laterales IZQ/DER son módulos HW-488 con DOS potenciómetros ajustables; el
central es el APDS9960 (no tiene pot — su umbral es software).

Uso:
  python3 Base/ir_check.py --ip 192.168.1.101          # calibración guiada (1 robot)
  python3 Base/ir_check.py --ip 192.168.1.101 --monitor # lectura L/C/R en vivo
  python3 Base/ir_check.py --ip 192.168.1.101 --pot R   # tunear los pots del HW-488 derecho
  python3 Base/ir_check.py --ip 192.168.1.101 --pot C --apply  # umbral del central (guiado)
  python3 Base/ir_check.py --ip 192.168.1.101 --thr 25 --save  # fijar umbral a mano
  python3 Base/ir_check.py                              # descubre por broadcast y usa el 1º
  python3 Base/ir_check.py --monitor --all             # monitor de todos los que respondan
"""
import argparse
import json
import os
import platform
import re
import socket
import sys
import time
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(os.path.dirname(HERE), 'configSystem.json')

# Canal → (letra de máscara, etiqueta) — mismo mapeo que analyze_logs.py
CHANNELS = [('L', 'IZQ'), ('C', 'CEN'), ('R', 'DER')]
SENS_RE = re.compile(r'Sensors:L(\d)-C(\d)-R(\d)')
ID_RE = re.compile(r'ID:([^|]+)')
EVAD_RE = re.compile(r'Evading:(\d)')
# Firmware nuevo (SENSOR_THRESHOLD): proximidad cruda del APDS, umbral y máscaras
PROX_RE = re.compile(r'Prox:(-?\d+)')
THR_RE = re.compile(r'Thr:(\d+)')
MASK_RE = re.compile(r'Mask:L(\d)-C(\d)-R(\d)')


def load_net():
    """broadcast_ip, port, interface desde configSystem.json."""
    try:
        u = json.load(open(CONFIG))['udp_communication']
        return u.get('broadcast_ip', '255.255.255.255'), int(u.get('port', 6060)), \
            u.get('network_interface')
    except Exception as e:
        print(f'⚠ No pude leer {CONFIG} ({e}); usando defaults.')
        return '255.255.255.255', 6060, None


def make_socket(port, interface):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if platform.system() == 'Linux':
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except (AttributeError, OSError):
            pass
        if interface:
            try:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE,
                             interface.encode())
            except (PermissionError, OSError):
                pass  # requiere root; seguimos igual
    try:
        s.bind(('', port))
    except OSError as e:
        sys.exit(f'✗ No pude tomar el puerto {port} ({e}).\n'
                 f'  ¿La base sigue corriendo? Cerrala (escribí BREAK) y reintentá.')
    s.settimeout(0.25)
    return s


def parse_status(msg):
    """'STATUS|...|Sensors:L1-C0-R1|Prox:37|Thr:2|Mask:L0-C0-R1|...' → dict o None.

    `prox`/`thr`/`mask` son None si el robot corre firmware viejo (sin esos campos)."""
    m = SENS_RE.search(msg)
    if not m:
        return None
    rid = ID_RE.search(msg)
    ev = EVAD_RE.search(msg)
    prox, thr, mask = PROX_RE.search(msg), THR_RE.search(msg), MASK_RE.search(msg)
    return {
        'id': rid.group(1).strip() if rid else '?',
        'sens': {'L': m.group(1) == '1', 'C': m.group(2) == '1', 'R': m.group(3) == '1'},
        'evade': bool(ev and ev.group(1) == '1'),
        'prox': int(prox.group(1)) if prox else None,
        'thr': int(thr.group(1)) if thr else None,
        'mask': ({'L': mask.group(1) == '1', 'C': mask.group(2) == '1',
                  'R': mask.group(3) == '1'} if mask else None),
    }


def send(sock, target, port, msg):
    sock.sendto(msg.encode(), (target, port))


def drain(sock):
    """Vacía el buffer de recepción (respuestas viejas)."""
    sock.settimeout(0.01)
    try:
        while True:
            sock.recv(2048)
    except socket.timeout:
        pass
    finally:
        sock.settimeout(0.25)


def register(sock, target, port, bcast):
    """CONFIG|START para que el robot nos guarde como Base y responda a este tool."""
    found = {}  # ip → id (por CONFIG|RECEIVED / primer STATUS)
    for _ in range(4):
        send(sock, target, port, f'CONFIG|START|{bcast}')
        send(sock, target, port, 'GET_STATUS')
        t0 = time.time()
        while time.time() - t0 < 0.5:
            try:
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                break
            msg = data.decode(errors='replace')
            st = parse_status(msg)
            if st:
                found[addr[0]] = st['id']
            elif 'CONFIG|RECEIVED' in msg:
                found.setdefault(addr[0], '?')
        if found and target != bcast:
            break
    return found


def set_masks(sock, target, port, values):
    """values: dict letra→0/1. Envía y da tiempo a que el robot aplique."""
    for letter, val in values.items():
        send(sock, target, port, f'SENSOR_MASK|{letter}|{val}')
        time.sleep(0.05)


def sample(sock, target, port, seconds):
    """Sondea GET_STATUS durante `seconds`; devuelve por-ip:
       {ip: {'id':.., 'n':N, 'L':active_count, 'C':.., 'R':.., 'evade':cnt,
             'prox':[lecturas crudas del APDS]}}."""
    acc = {}
    t0 = time.time()
    last_poll = 0.0
    while time.time() - t0 < seconds:
        now = time.time()
        if now - last_poll >= 0.15:
            send(sock, target, port, 'GET_STATUS')
            last_poll = now
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            continue
        st = parse_status(data.decode(errors='replace'))
        if not st:
            continue
        a = acc.setdefault(addr[0], {'id': st['id'], 'n': 0, 'L': 0, 'C': 0, 'R': 0,
                                     'evade': 0, 'prox': []})
        a['n'] += 1
        a['id'] = st['id']
        for letter in ('L', 'C', 'R'):
            if st['sens'][letter]:
                a[letter] += 1
        if st['evade']:
            a['evade'] += 1
        if st['prox'] is not None:
            a['prox'].append(st['prox'])
    return acc


# ── Modo monitor: lectura en vivo ────────────────────────────────────────────
def monitor(sock, target, port, interval, show_all):
    print('\nMonitor IR en vivo — mové la mano/caja frente a cada sensor. Ctrl-C para salir.\n')
    counts = {}  # ip → dict de conteos acumulados
    try:
        last_poll = 0.0
        while True:
            now = time.time()
            if now - last_poll >= interval:
                send(sock, target, port, 'GET_STATUS')
                last_poll = now
            try:
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                continue
            st = parse_status(data.decode(errors='replace'))
            if not st:
                continue
            c = counts.setdefault(addr[0], {'id': st['id'], 'n': 0, 'L': 0, 'C': 0,
                                            'R': 0, 'prox': None})
            c['id'] = st['id']
            c['n'] += 1
            c['prox'] = st['prox']
            for letter in ('L', 'C', 'R'):
                if st['sens'][letter]:
                    c[letter] += 1
            ips = sorted(counts) if show_all else [addr[0]]
            line = '  '.join(_fmt_live(ip, counts[ip],
                                       st['sens'] if ip == addr[0] else None, st['evade'])
                              for ip in ips)
            print('\r' + line + '   ', end='', flush=True)
    except KeyboardInterrupt:
        print('\n')
        _summary_counts(counts)


def _fmt_live(ip, c, sens, evad):
    boxes = ''
    for letter, tag in CHANNELS:
        on = sens[letter] if sens else (c[letter] > 0)
        boxes += f'{tag}[{"█" if on else " "}] '
        if letter == 'C' and c.get('prox') is not None:
            boxes += f'prox={c["prox"]:3d} '
    ev = ' EVADE' if evad else ''
    return f'{c["id"]}({ip.split(".")[-1]}) {boxes}{ev}'


def _summary_counts(counts):
    print('Resumen (fracción de muestras activa por canal):')
    for ip in sorted(counts):
        c = counts[ip]
        n = max(c['n'], 1)
        parts = '  '.join(f'{tag}={c[l] / n * 100:4.0f}%' for l, tag in CHANNELS)
        print(f'  {c["id"]} ({ip}): {parts}   (n={c["n"]})')


# ── Modo calibración guiada ──────────────────────────────────────────────────
def _countdown(prompt, secs):
    """Reemplaza a input(): imprime la consigna y hace una cuenta regresiva.
    Evita depender del ENTER (la terminal de VS Code no siempre traduce CR→LF)."""
    print(prompt)
    for k in range(int(secs), 0, -1):
        print(f'  … empieza en {k}s ', end='\r', flush=True)
        time.sleep(1)
    print('  … ¡muestreando!            ')


def guided(sock, target, port, libre_s, obst_s, countdown, apply):
    print('\n═══ Calibración guiada de IR ═══')
    print('Se prueba UN robot. Todos los canales se des-enmascaran primero (lectura cruda).')
    print(f'(auto-avanza con cuenta regresiva de {countdown:.0f}s; sin ENTER)\n')
    set_masks(sock, target, port, {'L': 0, 'C': 0, 'R': 0})
    drain(sock)

    # Paso 1 — LIBRE: nada enfrente → cualquier canal activo es FANTASMA
    _countdown('PASO 1/4 · LIBRE — despejá TODO el frente del robot.', countdown)
    print(f'  muestreando {libre_s:.0f}s…')
    libre = sample(sock, target, port, libre_s)
    if not libre:
        sys.exit('✗ Sin respuesta del robot. ¿IP correcta? ¿base cerrada? ¿robot encendido/en WiFi?')
    ip = _pick_ip(libre, target)
    a = libre[ip]
    n = max(a['n'], 1)
    phantom_frac = {l: a[l] / n for l, _ in CHANNELS}
    print(f'  robot {a["id"]} ({ip}), n={a["n"]} muestras:')
    for l, tag in CHANNELS:
        f = phantom_frac[l]
        flag = '  ← FANTASMA' if f > 0.2 else ''
        print(f'    {tag}: {f * 100:4.0f}% activo{flag}')

    # Pasos 2-4 — obstáculo frente a cada canal → debe activarse
    responds = {}
    for l, tag in CHANNELS:
        _countdown(f'\nPASO · {tag} — poné una caja/mano a ~10cm frente al sensor {tag}.',
                   countdown)
        print(f'  muestreando {obst_s:.0f}s…')
        drain(sock)
        s = sample(sock, target, port, obst_s)
        aa = s.get(ip, {'n': 0, l: 0})
        nn = max(aa['n'], 1)
        frac = aa.get(l, 0) / nn
        responds[l] = frac
        status = 'OK responde' if frac > 0.5 else (
            'INTERMITENTE' if frac > 0.2 else 'NO RESPONDE (muerto/desalineado)')
        print(f'    {tag}: {frac * 100:4.0f}% activo con obstáculo → {status}')
        # cross-talk informativo
        others = [f'{t}={aa.get(x, 0) / nn * 100:.0f}%' for x, t in CHANNELS if x != l]
        print(f'    (otros canales: {"  ".join(others)})')

    _report(phantom_frac, responds, sock, target, port, apply)


def _pick_ip(acc, target):
    if target in acc:
        return target
    return sorted(acc, key=lambda ip: -acc[ip]['n'])[0]


def _report(phantom_frac, responds, sock, target, port, apply):
    print('\n═══ Diagnóstico ═══')
    masks = {}
    for l, tag in CHANNELS:
        ph = phantom_frac[l] > 0.2
        resp = responds.get(l, 0)
        if ph and resp > 0.5:
            verdict = 'FANTASMA + responde → enmascarar (pierde ese lado en evasión)'
            masks[l] = 1
        elif ph:
            verdict = 'FANTASMA (y no confirmó obstáculo) → enmascarar'
            masks[l] = 1
        elif resp > 0.5:
            verdict = 'OK — limpio y responde'
        elif resp > 0.2:
            verdict = 'INTERMITENTE — revisá alineación/umbral'
        else:
            verdict = 'MUERTO/DESALINEADO — revisá cableado o pin (máscara no ayuda)'
        print(f'  {tag} ({l}): {verdict}')

    if masks:
        print('\nRecomendación de máscaras (fantasmas):')
        print('  En la consola de la base, por robot <id>:  ' +
              '  '.join(f'<id>.SENSOR_MASK|{l}|1' for l in masks))
        if apply:
            set_masks(sock, target, port, masks)
            time.sleep(0.2)
            drain(sock)
            chk = sample(sock, target, port, 1.0)
            ip = _pick_ip(chk, target) if chk else None
            if ip:
                a = chk[ip]
                nn = max(a['n'], 1)
                left = [tag for l, tag in CHANNELS if l in masks and a[l] / nn > 0.2]
                print('  ✓ Máscaras aplicadas.' +
                      (f' ⚠ Aún activo: {left}' if left else ' Fantasmas silenciados.'))
            print('  Nota: las máscaras viven en RAM del robot — si lo reiniciás/apagás se pierden;'
                  ' re-enviá desde la base tras un power-cycle.')
        else:
            print('  (corré con --apply para que ir_check las mande ahora mismo)')
    else:
        print('\nSin fantasmas: no hace falta enmascarar. Los IR están limpios.')


# ── Modo calibración de potenciómetros (HW-488) ──────────────────────────────
def _letter(channel):
    """'IZQ'/'l'/'C'… → 'L'/'C'/'R' (sale con error si no es un canal válido)."""
    letter = {'IZQ': 'L', 'CEN': 'C', 'DER': 'R'}.get(channel.upper(), channel.upper())
    if letter not in ('L', 'C', 'R'):
        sys.exit(f'Canal inválido: {channel} (usá L/C/R o IZQ/CEN/DER)')
    return letter


def _pct(values, q):
    """Percentil q (0..1) sin numpy."""
    if not values:
        return None
    s = sorted(values)
    return s[min(len(s) - 1, max(0, int(round(q * (len(s) - 1)))))]


def _sample_prox(sock, target, port, seconds, label):
    """Muestrea la proximidad CRUDA del APDS mostrando barra en vivo.
    Devuelve la lista de lecturas (vacía si el firmware no reporta Prox)."""
    vals = []
    t0 = last_poll = 0.0
    t0 = time.time()
    while time.time() - t0 < seconds:
        now = time.time()
        if now - last_poll >= 0.1:
            send(sock, target, port, 'GET_STATUS')
            last_poll = now
        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            continue
        st = parse_status(data.decode(errors='replace'))
        if not st or st['prox'] is None:
            continue
        vals.append(st['prox'])
        n = int(st['prox'] * 24 / 255)
        print(f'\r  {label} prox={st["prox"]:3d} [{"#" * n}{" " * (24 - n)}] '
              f'(n={len(vals)})', end='', flush=True)
    print()
    return vals


def central_calibrate(sock, target, port, libre_s, obst_s, countdown, apply):
    """Calibración guiada del APDS9960 central: mide el piso de ruido con el
    frente libre y el nivel a la distancia de disparo deseada, y recomienda el
    umbral (firmware: centralDistance > umbral). Requiere firmware con Prox:."""
    print('\n═══ Calibración del IR CENTRAL (APDS9960) ═══')
    print('El central NO tiene potenciómetros: su umbral es software.')
    set_masks(sock, target, port, {'C': 0})
    drain(sock)

    _countdown('PASO 1/2 · LIBRE — despejá TODO el frente del robot.', countdown)
    libre = _sample_prox(sock, target, port, libre_s, 'LIBRE ')
    if not libre:
        sys.exit('✗ El robot no reporta «Prox:» — tiene firmware viejo.\n'
                 '  Flasheá (OTA) el AttaBot.ino actual y reintentá.')

    _countdown('\nPASO 2/2 · OBSTÁCULO — poné la caja a la distancia a la que '
               'QUERÉS que dispare\n  (medila con regla; ej. 15cm).', countdown)
    obst = _sample_prox(sock, target, port, obst_s, 'OBST  ')

    noise_hi = _pct(libre, 0.95)
    obst_lo = _pct(obst, 0.05)
    print(f'\n  LIBRE:    máx={max(libre)}  p95={noise_hi}  mediana={_pct(libre, 0.5)}')
    if obst:
        print(f'  OBSTÁCULO: mín={min(obst)}  p05={obst_lo}  mediana={_pct(obst, 0.5)}')

    if not obst or obst_lo is None:
        print('\n⚠ Sin lecturas con obstáculo — no puedo recomendar umbral.')
        return
    if obst_lo <= noise_hi + 2:
        print(f'\n⚠ Los rangos se SOLAPAN (ruido≈{noise_hi}, obstáculo≈{obst_lo}): a esa '
              'distancia\n  el APDS no distingue. Acercá el objetivo de disparo o usá '
              'objetos más reflectivos.')
        return
    rec = int((noise_hi + obst_lo) / 2)
    print(f'\n  → Umbral recomendado: {rec}   (ruido {noise_hi} · disparo {obst_lo})')
    print(f'    Firmware: centralDistance > {rec}. Hoy el default es 2 (hiper-sensible:'
          '\n    dispara de lejos y confunde robots con obstáculos).')
    print(f'  Aplicar a mano desde la consola de la base: '
          f'<id>.SENSOR_THRESHOLD|C|{rec}|SAVE')
    if apply:
        send(sock, target, port, f'SENSOR_THRESHOLD|C|{rec}|SAVE')
        time.sleep(0.3)
        drain(sock)
        chk = sample(sock, target, port, 1.0)
        ip = _pick_ip(chk, target) if chk else None
        print(f'  ✓ Enviado SENSOR_THRESHOLD|C|{rec}|SAVE (persiste en NVS).')
        if ip and chk[ip]['n']:
            print(f'    Verificación: C activo {chk[ip]["C"] / chk[ip]["n"] * 100:.0f}% '
                  'del tiempo con el frente como esté ahora.')


def pot_calibrate(sock, target, port, channel):
    """Tuneo en vivo de UN canal mientras girás los pots del HW-488.
    Muestra el estado del OUT (DETECTA/libre) a ~10Hz y avisa si titila
    (chatter = pot en el filo del umbral). Robot alimentado y en WiFi;
    el sensor puede estar desarmado con tal de que siga cableado al ESP32."""
    letter = _letter(channel)
    tag = {'L': 'IZQ', 'C': 'CEN', 'R': 'DER'}[letter]
    set_masks(sock, target, port, {letter: 0})   # des-enmascarar → lectura cruda
    drain(sock)

    print(f'\n═══ Calibración de pots · canal {tag} ({letter}) ═══')
    print('Módulo HW-488 (2 pots). OUT es activo-BAJO: LOW = detecta.\n'
          '  • Un pot ajusta DISTANCIA/sensibilidad (el que corre el punto de disparo).\n'
          '  • El otro ajusta UMBRAL/frecuencia fino (estabilidad).\n'
          '  (los clones varían — identificá cuál es cuál mirando el feedback en vivo).\n'
          'Procedimiento:\n'
          '  1. Frente DESPEJADO → debe quedar «libre». Si dispara solo, bajá sensibilidad.\n'
          '  2. Poné un objeto a la distancia deseada (regla, ej. 15cm) y girá el pot de\n'
          '     distancia hasta que JUSTO pase a «detecta».\n'
          '  3. Sacá el objeto → debe volver a «libre» SIN titilar. Si titila, afiná el 2º pot.\n'
          '  4. Verificá acercando/alejando. Ctrl-C para salir.\n')

    flips = deque()          # timestamps de cambios de estado (ventana 2s)
    last = None
    det = tot = 0
    last_poll = 0.0
    try:
        while True:
            now = time.time()
            if now - last_poll >= 0.1:
                send(sock, target, port, 'GET_STATUS')
                last_poll = now
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                continue
            st = parse_status(data.decode(errors='replace'))
            if not st:
                continue
            state = st['sens'][letter]
            tot += 1
            det += 1 if state else 0
            if last is not None and state != last:
                flips.append(now)
            last = state
            while flips and now - flips[0] > 2.0:
                flips.popleft()
            bar = '██████████' if state else '░░░░░░░░░░'
            lbl = 'DETECTA (LOW) ' if state else 'libre   (HIGH)'
            stab = f'⚠ TITILA ({len(flips)}/2s)' if len(flips) >= 4 else 'estable       '
            print(f'\r  {tag} {bar} {lbl}  {stab}', end='', flush=True)
    except KeyboardInterrupt:
        print('\n')
        print(f'Resumen {tag}: {det / max(tot, 1) * 100:.0f}% del tiempo DETECTA '
              f'(n={tot}). Ideal: ~0% con frente libre, salto limpio al acercar.')


def motors(sock, target, port, pct):
    """Diagnóstico de tracción: lanza SELFTEST y traduce el resultado.

    El firmware mueve cada motor por separado y reporta los pulsos de AMBOS
    encoders más el giro de la IMU. Ese cruce es lo que separa fallas que a
    simple vista son idénticas (el robot "no anda"):
      · motor muerto   → sus pulsos ≈0 Y la IMU no gira
      · encoder muerto → sus pulsos ≈0 PERO la IMU gira (el motor sí empuja)
      · motor flojo    → pulsos muy por debajo del otro lado
    """
    print(f'\n═══ Diagnóstico de motores (PWM {pct}%) ═══')
    print('El robot GIRA sobre su eje en cada paso: dejá ~30cm libres alrededor.')
    for k in range(3, 0, -1):
        print(f'  … empieza en {k}s ', end='\r', flush=True)
        time.sleep(1)
    print('  … ejecutando            ')

    drain(sock)
    send(sock, target, port, f'SELFTEST|{pct}')
    fases, t0 = {}, time.time()
    while time.time() - t0 < 25:
        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            continue
        msg = data.decode(errors='replace')
        if 'SELFTEST' not in msg:
            continue
        print(f'  {msg.split("SELFTEST")[-1].strip(": ")}')
        m = re.search(r'SELFTEST (\w+ \w+): pulsos_izq=(-?\d+) pulsos_der=(-?\d+)'
                      r' dyaw=(-?[\d.]+)', msg)
        if m:
            fases[m.group(1)] = (int(m.group(2)), int(m.group(3)), float(m.group(4)))
        if 'fin' in msg:
            break

    if len(fases) < 3:
        print('\n✗ El robot no completó el test (¿firmware sin SELFTEST? '
              '¿IP correcta?).')
        return

    li, ld, lyaw = fases['IZQ sola']
    ri, rd, ryaw = fases['DER sola']
    bi, bd, byaw = fases['AMBAS']
    print('\n═══ Veredicto ═══')

    def lado(nombre, propios, ajenos, dyaw):
        if abs(propios) < 20:
            if abs(dyaw) > 8:
                return f'{nombre}: ENCODER MUERTO — el motor empuja (IMU {dyaw:+.0f}°) pero no cuenta'
            return f'{nombre}: MOTOR MUERTO — sin pulsos y sin giro ({dyaw:+.0f}°)'
        if abs(ajenos) > abs(propios) * 0.4:
            return (f'{nombre}: ⚠ el otro encoder también contó '
                    f'({propios} vs {ajenos}) — revisá cableado cruzado')
        return f'{nombre}: ok ({propios} pulsos, IMU {dyaw:+.0f}°)'

    print('  ' + lado('IZQUIERDO', li, ld, lyaw))
    print('  ' + lado('DERECHO', rd, ri, ryaw))

    if abs(bi) > 20 and abs(bd) > 20:
        ratio = min(abs(bi), abs(bd)) / max(abs(bi), abs(bd))
        flojo = 'izquierdo' if abs(bi) < abs(bd) else 'derecho'
        if ratio < 0.75:
            print(f'  AMBAS: ⚠ DESBALANCE {ratio*100:.0f}% — el motor {flojo} '
                  f'rinde menos ({bi} vs {bd}); el robot se irá de lado al avanzar')
        elif abs(byaw) > 12:
            print(f'  AMBAS: ⚠ avanza torcido (IMU {byaw:+.0f}° en línea recta)')
        else:
            print(f'  AMBAS: ok ({bi} vs {bd}, desvío {byaw:+.0f}°)')
    else:
        print(f'  AMBAS: ✗ no avanzó ({bi} vs {bd})')


def main():
    ap = argparse.ArgumentParser(description='Banco de prueba en vivo de IR (AttaBot).')
    ap.add_argument('--ip', help='IP del robot a probar (unicast). Si se omite, broadcast.')
    ap.add_argument('--broadcast', help='IP de broadcast (default: configSystem.json)')
    ap.add_argument('--port', type=int, help='puerto UDP (default: configSystem.json)')
    ap.add_argument('--monitor', action='store_true', help='lectura L/C/R en vivo (Ctrl-C sale)')
    ap.add_argument('--pot', metavar='CANAL',
                    help='tuneo en vivo de UN canal (L/R = pots HW-488; '
                         'C = calibración guiada del umbral del APDS)')
    ap.add_argument('--motors', nargs='?', const=40, type=int, metavar='PWM%',
                    help='diagnóstico de tracción: mueve cada motor por separado '
                         'y distingue motor muerto / encoder muerto / motor flojo '
                         '(default 40%%; subilo si el robot no arranca)')
    ap.add_argument('--thr', type=int, metavar='N',
                    help='fija el umbral del IR central (0..255) y sale; usa --save '
                         'para persistirlo en el robot')
    ap.add_argument('--save', action='store_true',
                    help='con --thr: persiste el umbral en NVS (sobrevive apagón)')
    ap.add_argument('--all', action='store_true', help='monitor: mostrar todos los robots')
    ap.add_argument('--interval', type=float, default=0.2, help='periodo de sondeo (s)')
    ap.add_argument('--libre', type=float, default=5.0, help='segundos del muestreo LIBRE')
    ap.add_argument('--obst', type=float, default=3.0, help='segundos por canal con obstáculo')
    ap.add_argument('--countdown', type=float, default=5.0,
                    help='segundos de cuenta regresiva antes de cada paso (sin ENTER)')
    ap.add_argument('--apply', action='store_true',
                    help='aplica las máscaras recomendadas al final (sin preguntar)')
    a = ap.parse_args()

    bcast_cfg, port_cfg, iface = load_net()
    bcast = a.broadcast or bcast_cfg
    port = a.port or port_cfg
    target = a.ip or bcast

    sock = make_socket(port, iface)
    print(f'ir_check — puerto {port}, objetivo {target} '
          f'({"unicast" if a.ip else "broadcast"}).')
    found = register(sock, target, port, bcast)
    if found:
        who = '  '.join(f'{rid}@{ip}' for ip, rid in sorted(found.items()))
        print(f'Robot(s) detectado(s): {who}')
        if not a.ip and len(found) == 1:
            target = next(iter(found))  # enfocá el único que respondió
            print(f'Enfocando {target}.')
    else:
        print('⚠ Nadie respondió todavía (seguirá intentando en el sondeo).')

    drain(sock)
    if a.motors is not None:
        motors(sock, target, port, a.motors)
    elif a.thr is not None:
        send(sock, target, port,
             f'SENSOR_THRESHOLD|C|{a.thr}' + ('|SAVE' if a.save else ''))
        time.sleep(0.3)
        drain(sock)
        chk = sample(sock, target, port, 1.0)
        ip = _pick_ip(chk, target) if chk else None
        thr = None
        if ip:
            send(sock, target, port, 'GET_STATUS')
            try:
                st = parse_status(sock.recvfrom(2048)[0].decode(errors='replace'))
                thr = st['thr'] if st else None
            except socket.timeout:
                pass
        print(f'Umbral central → {a.thr}{" (guardado en NVS)" if a.save else " (solo RAM)"}'
              f'{f"; el robot reporta Thr:{thr}" if thr is not None else ""}')
    elif a.pot:
        if _letter(a.pot) == 'C':
            central_calibrate(sock, target, port, a.libre, a.obst, a.countdown, a.apply)
        else:
            pot_calibrate(sock, target, port, a.pot)
    elif a.monitor:
        monitor(sock, target, port, a.interval, a.all)
    else:
        guided(sock, target, port, a.libre, a.obst, a.countdown, a.apply)


if __name__ == '__main__':
    main()
