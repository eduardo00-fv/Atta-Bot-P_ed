#!/usr/bin/env python3
"""
analyze_logs.py — Métricas de navegación desde los logs de la base
====================================================================
Fase 1 del plan de métricas: parsea los CSV existentes (PositionLogs +
ConsoleLogs, formato lab — también los SIM_ generados desde Webots) y extrae
métricas cuantitativas por "run" de navegación:

  - convergence_time  : segundos desde el inicio del GT hasta 'NAV: llegó'
  - steps             : REQUEST_POSITION durante el run
  - path_length       : Σ|linearDisplacement| del PositionLog en la ventana
  - path_efficiency   : distancia_recta / path_length (1.0 = trayecto perfecto)
  - final_error       : distancia de la pose final al goal (si el goal se conoce
                        por el ack 'GT iniciado: goal=(x,y)')
  - evasions          : evasiones completadas ('Cooldown de evasión completado')
  - obstacle_events   : mensajes CHECK_OBSTACLE en la ventana

Un run empieza con el ack 'GT iniciado' (goal conocido) o, si no hay ack (debug
apagado), con el primer REQUEST_POSITION tras ≥8s sin solicitudes; termina con
'NAV: llegó a (x,y)'. Los runs de congregación se etiquetan con el mensaje
'CONGREGATION: slot' si aparece en la ventana.

Uso:
    python analyze_logs.py                    # sesión más reciente
    python analyze_logs.py --session 03-07_10-04
    python analyze_logs.py --all              # resumen de todas las sesiones
    python analyze_logs.py --all --csv runs.csv
"""

import argparse
import csv
import glob
import math
import os
import re

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
POS_DIR = os.path.join(BASE_DIR, 'PositionLogs')
CON_DIR = os.path.join(BASE_DIR, 'ConsoleLogs')

RE_GT_START = re.compile(r'GT iniciado: goal=\((-?[\d.]+),(-?[\d.]+)\)')
RE_ARRIVED = re.compile(r'NAV: llegó a \((-?[\d.]+),(-?[\d.]+)\)')
IDLE_GAP_S = 8.0     # sin REQUEST_POSITION por este tiempo = run nuevo (fallback)


def session_tag(path):
    """'Position_Log_03-07_10-04_Robots_2.csv' → '03-07_10-04_Robots_2'
    (conserva el prefijo SIM_ si existe)."""
    name = os.path.basename(path)
    return re.sub(r'^(Position|Console)_Log_', '', name).rsplit('.', 1)[0]


def find_sessions():
    """Retorna [(tag, position_csv, console_csv|None)] ordenado por mtime."""
    positions = sorted(glob.glob(os.path.join(POS_DIR, 'Position_Log_*.csv')),
                       key=os.path.getmtime)
    consoles = {session_tag(p): p
                for p in glob.glob(os.path.join(CON_DIR, 'Console_Log_*.csv'))}
    return [(session_tag(p), p, consoles.get(session_tag(p))) for p in positions]


def load_console(path):
    """[(t, robot_id, message)] — tolera filas corruptas."""
    events = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            try:
                events.append((float(row['time']), row['idrobot'], row['message']))
            except (ValueError, KeyError, TypeError):
                continue
    return events


def load_positions(path):
    """{robot_id: [(t, x, y, angle, lin_disp)]} — tolera filas corruptas."""
    tracks = {}
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            try:
                tracks.setdefault(row['idrobot'], []).append(
                    (float(row['time']), float(row['x']), float(row['y']),
                     float(row['angle']), float(row['linearDisplacement'])))
            except (ValueError, KeyError, TypeError):
                continue
    return tracks


def segment_runs(events, robot_id):
    """Corta los eventos de UN robot en runs de navegación.

    Retorna [{'t0', 't1', 'goal': (x,y)|None, 'steps', 'evasions',
              'obstacle_events', 'kind': 'GT'|'CONGREGATION', 'arrived': bool}]
    """
    runs = []
    current = None
    last_request = None

    def close(t1, arrived):
        nonlocal current
        if current is not None:
            current['t1'] = t1
            current['arrived'] = arrived
            runs.append(current)
            current = None

    for t, rid, msg in events:
        if rid != robot_id:
            continue

        m = RE_GT_START.search(msg)
        if m:
            close(t, False)   # run anterior sin llegada = interrumpido
            current = {'t0': t, 'goal': (float(m.group(1)), float(m.group(2))),
                       'steps': 0, 'evasions': 0, 'obstacle_events': 0,
                       'kind': 'GT'}
            continue

        if msg.startswith('REQUEST_POSITION'):
            if current is None:
                # Sin ack de inicio (debug apagado): el primer REQUEST tras un
                # silencio largo abre un run sin goal conocido
                if last_request is None or t - last_request >= IDLE_GAP_S:
                    current = {'t0': t, 'goal': None, 'steps': 0, 'evasions': 0,
                               'obstacle_events': 0, 'kind': 'GT'}
            if current is not None:
                current['steps'] += 1
            last_request = t
            continue

        if current is None:
            continue
        if 'CONGREGATION: slot' in msg:
            current['kind'] = 'CONGREGATION'
        elif 'Cooldown de evasión completado' in msg:
            current['evasions'] += 1
        elif msg.startswith('CHECK_OBSTACLE'):
            current['obstacle_events'] += 1
        elif RE_ARRIVED.search(msg):
            close(t, True)

    return runs


def binned_path(window, bin_s=1.0):
    """Camino recorrido sobre medianas de posición por ventana de 1s.

    Sumar |linearDisplacement| crudo infla el camino ~10-15x: el jitter ArUco
    (σ≈10-30mm) dispara filas de ≥4mm a ~20Hz aunque el robot esté quieto.
    La mediana por bin promedia el ruido de media cero y deja el movimiento
    real (≤180mm/s) intacto.
    """
    bins = {}
    for t, x, y, *_ in window:
        bins.setdefault(int(t / bin_s), []).append((x, y))

    def median(vals):
        vals = sorted(vals)
        n = len(vals)
        return vals[n // 2] if n % 2 else (vals[n // 2 - 1] + vals[n // 2]) / 2

    centers = [(median([p[0] for p in pts]), median([p[1] for p in pts]))
               for _, pts in sorted(bins.items())]
    path = sum(math.dist(centers[i], centers[i + 1])
               for i in range(len(centers) - 1))
    return path, centers


def run_metrics(run, track):
    """Completa el run con las métricas del PositionLog en [t0, t1]."""
    window = [p for p in track if run['t0'] <= p[0] <= run['t1']]
    if len(window) < 2:
        return None
    path, centers = binned_path(window)
    if len(centers) < 2:
        return None
    x0, y0 = centers[0]
    x1, y1 = centers[-1]
    straight = math.dist((x0, y0), (x1, y1))
    metrics = {
        'duration_s': round(run['t1'] - run['t0'], 1),
        'path_mm': round(path, 0),
        'straight_mm': round(straight, 0),
        'efficiency': round(straight / path, 2) if path > 0 else 0.0,
        'final_error_mm': (round(math.dist((x1, y1), run['goal']), 0)
                           if run['goal'] else None),
    }
    metrics.update(run)
    return metrics


def analyze_session(tag, pos_path, con_path, verbose=True):
    """Métricas de una sesión. Retorna lista de runs (dicts)."""
    if con_path is None:
        if verbose:
            print(f'{tag}: sin ConsoleLog pareado — no se pueden segmentar runs')
        return []
    events = load_console(con_path)
    tracks = load_positions(pos_path)
    all_runs = []
    for rid in sorted(tracks):
        for run in segment_runs(events, rid):
            m = run_metrics(run, tracks[rid])
            if m is None:
                continue
            m['robot'] = rid
            m['session'] = tag
            all_runs.append(m)

    if verbose and all_runs:
        print(f'\n=== {tag} — {len(all_runs)} run(s) ===')
        hdr = (f'{"robot":>5} {"tipo":<13} {"llegó":>5} {"dur(s)":>7} '
               f'{"pasos":>5} {"camino":>7} {"recta":>6} {"efic":>5} '
               f'{"err_fin":>7} {"evas":>4} {"obst":>4}')
        print(hdr)
        for m in all_runs:
            err = f"{m['final_error_mm']:.0f}" if m['final_error_mm'] is not None else '—'
            print(f"{m['robot']:>5} {m['kind']:<13} "
                  f"{'sí' if m['arrived'] else 'NO':>5} {m['duration_s']:>7} "
                  f"{m['steps']:>5} {m['path_mm']:>7.0f} {m['straight_mm']:>6.0f} "
                  f"{m['efficiency']:>5} {err:>7} {m['evasions']:>4} "
                  f"{m['obstacle_events']:>4}")
    elif verbose:
        print(f'{tag}: sin runs de navegación detectados')
    return all_runs


def summarize(runs):
    """Resumen agregado (mediana de las métricas de los runs que llegaron)."""
    ok = [r for r in runs if r['arrived']]
    if not runs:
        return
    print(f'\n=== RESUMEN: {len(runs)} runs, {len(ok)} llegadas '
          f'({100 * len(ok) / len(runs):.0f}%) ===')
    if not ok:
        return

    def median(vals):
        vals = sorted(vals)
        n = len(vals)
        return vals[n // 2] if n % 2 else (vals[n // 2 - 1] + vals[n // 2]) / 2

    print(f"convergencia mediana : {median([r['duration_s'] for r in ok]):.1f} s")
    print(f"pasos medianos       : {median([r['steps'] for r in ok]):.0f}")
    print(f"eficiencia mediana   : {median([r['efficiency'] for r in ok]):.2f}")
    errs = [r['final_error_mm'] for r in ok if r['final_error_mm'] is not None]
    if errs:
        print(f"error final mediano  : {median(errs):.0f} mm (n={len(errs)})")
    print(f"evasiones totales    : {sum(r['evasions'] for r in runs)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument('--session', help='tag parcial, ej. 03-07_10-04 o SIM')
    ap.add_argument('--all', action='store_true', help='todas las sesiones')
    ap.add_argument('--csv', help='exportar los runs a un CSV')
    a = ap.parse_args()

    sessions = find_sessions()
    if not sessions:
        print('No hay PositionLogs')
        return

    if a.session:
        sessions = [s for s in sessions if a.session in s[0]]
        if not sessions:
            print(f'Ninguna sesión matchea "{a.session}"')
            return
    elif not a.all:
        sessions = sessions[-1:]   # la más reciente

    runs = []
    for tag, pos, con in sessions:
        runs.extend(analyze_session(tag, pos, con, verbose=not a.all))

    if a.all:
        # En modo --all: una línea por sesión con runs + el agregado global
        for tag in dict.fromkeys(r['session'] for r in runs):
            rs = [r for r in runs if r['session'] == tag]
            ok = sum(1 for r in rs if r['arrived'])
            print(f'{tag:<32} {len(rs):>3} runs  {ok:>3} llegadas')
    summarize(runs)

    if a.csv and runs:
        cols = ['session', 'robot', 'kind', 'arrived', 't0', 't1', 'duration_s',
                'steps', 'path_mm', 'straight_mm', 'efficiency',
                'final_error_mm', 'evasions', 'obstacle_events', 'goal']
        with open(a.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=cols, extrasaction='ignore')
            w.writeheader()
            w.writerows(runs)
        print(f'\nRuns exportados a {a.csv}')


if __name__ == '__main__':
    main()
