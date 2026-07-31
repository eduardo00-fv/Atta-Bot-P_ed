#!/usr/bin/env python3
"""Extrae la geometría de obstáculos de los videos del lab.

Calibra píxel→mm con los propios robots (sus markers ArUco están en el video y
su posición en mm está en el PositionLog), y después detecta las cajas de cartón
por color contra el fondo blanco de la arena.
"""
import collections
import csv
import sys

import cv2
import numpy as np

BASE = '/home/thrain/Documents/Atta-Bot-P_ed/Base'
ARENA_W, ARENA_H = 2400.0, 1750.0
CAM_PANEL_PX = 675   # alto del panel de camara dentro del frame compuesto


def tracks(pos_csv):
    t = collections.defaultdict(list)
    for r in csv.DictReader(open(pos_csv)):
        t[int(r['idrobot'])].append((float(r['time']), float(r['x']), float(r['y'])))
    return {k: sorted(v) for k, v in t.items()}


def at(track, t):
    """Posición interpolada al tiempo t (None si cae fuera o hay hueco largo)."""
    lo, hi = 0, len(track) - 1
    if not track or t < track[0][0] or t > track[-1][0]:
        return None
    while lo < hi - 1:
        mid = (lo + hi) // 2
        if track[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    (t0, x0, y0), (t1, x1, y1) = track[lo], track[hi]
    if t1 - t0 > 1.0:
        return None
    f = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
    return (x0 + f * (x1 - x0), y0 + f * (y1 - y0))


def calibrate(video, pos_csv, step=5):
    """Homografía píxel→mm por RANSAC.

    El tiempo de cada frame se estima repartiendo la duración del log entre los
    frames: el .avi se escribe uno por frame procesado, pero los fps nominales
    del contenedor no coinciden con la tasa real. Los emparejamientos que ese
    supuesto arruina (robot en movimiento) los descarta RANSAC; los que lo
    sostienen (robot quieto, que es la mayor parte de la corrida) mandan.
    """
    tr = tracks(pos_csv)
    dur = max(v[-1][0] for v in tr.values())
    cap = cv2.VideoCapture(video)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    det = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
        cv2.aruco.DetectorParameters())
    src, dst = [], []
    for i in range(0, n, step):
        cap.set(cv2.CAP_PROP_POS_FRAMES, i)
        ok, f = cap.read()
        if not ok:
            break
        corners, ids, _ = det.detectMarkers(f)
        if ids is None:
            continue
        t = i * dur / n
        for c, idx in zip(corners, ids.ravel()):
            idx = int(idx)
            p = c[0].mean(axis=0)
            if idx == 0:
                src.append(p), dst.append((0.0, 0.0))
            elif idx in tr:
                q = at(tr[idx], t)
                if q:
                    src.append(p), dst.append(q)
    cap.release()
    src, dst = np.float32(src), np.float32(dst)
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 40.0)
    res = np.linalg.norm(
        cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2) - dst,
        axis=1)
    keep = mask.ravel().astype(bool)
    return H, len(src), keep.sum(), np.median(res[keep])


def plate(video, n_samples=40):
    """Mediana temporal del video = la escena ESTÁTICA sola.

    Los robots se mueven durante la corrida, así que en la mediana desaparecen y
    quedan solo la arena y las cajas. Sobre un frame suelto, en cambio, un robot
    parado encima de una caja la borra del conteo — que es lo que pasaba."""
    cap = cv2.VideoCapture(video)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frames = []
    for i in np.linspace(0, n - 1, n_samples, dtype=int):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(i))
        ok, f = cap.read()
        if ok:
            frames.append(f)
    cap.release()
    med = np.median(np.stack(frames), axis=0).astype(np.uint8)
    # Solo el panel de la CAMARA. El .avi es un compuesto: debajo va el mapa
    # que dibuja la Base, y sus celdas de color se detectaban como cajas.
    return med[:CAM_PANEL_PX]


def boxes(video, H, frame_i=1):
    """Cajas de cartón: son lo único NO blanco y NO negro sobre la arena."""
    f = plate(video)
    hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
    # cartón: tono cálido (10-35), saturación media, claro. La arena es blanca
    # (saturación ~0) y los markers son negros (valor bajo).
    m = cv2.inRange(hsv, (8, 45, 80), (38, 255, 255))
    # Kernels chicos a propósito: cerrar con 7x7 engordaba cada caja ~20mm por
    # lado y las medidas salían 80mm→120mm.
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cnts:
        if cv2.contourArea(c) < 120:
            continue
        (cx, cy), (w, h), ang = cv2.minAreaRect(c)
        pts = cv2.perspectiveTransform(
            np.float32(cv2.boxPoints(((cx, cy), (w, h), ang))).reshape(-1, 1, 2),
            H).reshape(-1, 2)
        c_mm = pts.mean(axis=0)
        # Margen: la homografía se ajusta al plano de los markers y las cajas son
        # más altas, así que las de los bordes caen un poco fuera por paralaje.
        if not (-150 <= c_mm[0] <= ARENA_W + 150
                and -150 <= c_mm[1] <= ARENA_H + 150):
            continue
        e = [np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4)]
        out.append((c_mm[0], c_mm[1], (e[0] + e[2]) / 2, (e[1] + e[3]) / 2))
    return sorted(out, key=lambda b: (b[0], b[1]))


if __name__ == '__main__':
    for name in sys.argv[1:]:
        v = f'{BASE}/Videos/{name}.avi'
        p = f'{BASE}/PositionLogs/{name}.csv'
        H, ntot, nin, res = calibrate(v, p)
        print(f'\n=== {name} ===')
        print(f'calibración: {nin}/{ntot} correspondencias, '
              f'residuo mediano {res:.0f}mm')
        bs = boxes(v, H)
        print(f'{len(bs)} obstáculos detectados:')
        for x, y, w, h in bs:
            print(f'   ({x:6.0f},{y:6.0f})  {w:4.0f} x {h:4.0f} mm')


def verify(name, out_png):
    """Dibuja lo detectado sobre el plano estático, para revisarlo a ojo."""
    v = f'{BASE}/Videos/{name}.avi'
    p = f'{BASE}/PositionLogs/{name}.csv'
    H, ntot, nin, res = calibrate(v, p)
    img = plate(v)
    Hi = np.linalg.inv(H)
    for x, y, w, h in boxes(v, H):
        c = cv2.perspectiveTransform(np.float32([[[x, y]]]), Hi)[0][0]
        inside = 0 <= x <= ARENA_W and 0 <= y <= ARENA_H
        col = (0, 200, 0) if inside else (0, 0, 255)
        cv2.circle(img, (int(c[0]), int(c[1])), 7, col, 2)
        cv2.putText(img, f'{x:.0f},{y:.0f}', (int(c[0]) + 9, int(c[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1)
    # marco de la arena segun la calibracion
    corners = cv2.perspectiveTransform(
        np.float32([[[0, 0], [ARENA_W, 0], [ARENA_W, ARENA_H], [0, ARENA_H]]]),
        Hi)[0].astype(int)
    cv2.polylines(img, [corners], True, (255, 0, 0), 2)
    cv2.imwrite(out_png, img[:750])
    print(f'{name}: {out_png}')
