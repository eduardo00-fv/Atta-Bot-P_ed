#!/usr/bin/env python3
"""
marker_acrilico.py — Genera los SVG de corte láser para markers ArUco en
acrílico de dos láminas (1.5mm c/u): base BLANCA mate + capa NEGRA mate con
las celdas blancas caladas.

Por cada marker id genera dos archivos en Base/markers_svg/:
  marker_<id>_negro.svg   — capa negra: cuadrado exterior (lado = --size) con
                            las celdas blancas como huecos a calar. Las celdas
                            adyacentes en fila se fusionan en un solo hueco
                            para no cortar dos veces el mismo borde (fusionar
                            el resto con Path→Union en Inkscape antes de cortar).
  marker_<id>_blanco.svg  — base blanca: cuadrado de lado size + 2 módulos
                            (quiet zone de 1 módulo). Incluye en ROJO el
                            contorno del marker para grabar/marcar la posición
                            donde pegar la capa negra.

Geometría (DICT_4X4_50): 4x4 datos + borde negro = 6x6 módulos.
  --size 80  →  módulo = 13.333mm, base blanca = 106.667mm

⚠ AVISOS:
  - El script detecta celdas negras "isla" (sin conexión al borde): esas piezas
    quedan SUELTAS al calar la capa negra y hay que pegarlas una por una.
  - Al desplegar los markers de 80mm: cambiar marker_size_mm de 100 → 80 en
    configSystem.json (la escala métrica de TODAS las posiciones depende de eso).
  - 80mm tiene 36% menos área que los 100mm actuales: cortar UN marker primero
    y validar detección/jitter con la C920 antes de cortar toda la serie.

Uso:
  python3 Base/marker_acrilico.py              # ids 0-8, 80mm (archivos por pieza)
  python3 Base/marker_acrilico.py 1 2 --size 80
  python3 Base/marker_acrilico.py --sheet      # UNA lámina con TODOS los cortes

Modo --sheet (lámina única de acrílico blanco, piezas negras con spray mate):
  Genera markers_svg/sheet_corte_markers_<size>mm.svg con las 9 bases blancas
  y las 9 capas caladas acomodadas en la misma lámina. Convención:
    ROJO  = línea de CORTE (contornos y huecos)
    AZUL  = GRABADO opcional (guía de pegado en la base + id de cada pieza;
            los ids quedan ocultos: bajo la capa negra en la base, bajo el
            spray en la capa calada)
  El tamaño total de lámina requerido se imprime al final — si tu lámina es
  más chica, generá subconjuntos: `python3 marker_acrilico.py 0 1 2 --sheet`.
"""
import os
import sys
import argparse

import cv2
import numpy as np

OUTDIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "markers_svg")
MODULES = 6  # DICT_4X4: 4x4 datos + 1 módulo de borde negro por lado


def markerMatrix(markerId):
    """Matriz 6x6 del marker: True = módulo negro."""
    aruco = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    img = cv2.aruco.generateImageMarker(aruco, markerId, MODULES)
    return img < 128


def blackIslands(black):
    """Celdas negras sin conexión-4 al borde exterior → piezas sueltas al calar."""
    n = black.shape[0]
    connected = np.zeros_like(black)
    stack = [(r, c) for r in range(n) for c in range(n)
             if black[r, c] and (r in (0, n - 1) or c in (0, n - 1))]
    while stack:
        r, c = stack.pop()
        if connected[r, c]:
            continue
        connected[r, c] = True
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            rr, cc = r + dr, c + dc
            if 0 <= rr < n and 0 <= cc < n and black[rr, cc] and not connected[rr, cc]:
                stack.append((rr, cc))
    return [(r, c) for r in range(n) for c in range(n)
            if black[r, c] and not connected[r, c]]


def svgHeader(side):
    return (f'<svg xmlns="http://www.w3.org/2000/svg" '
            f'width="{side:.3f}mm" height="{side:.3f}mm" '
            f'viewBox="0 0 {side:.3f} {side:.3f}">\n')


def rect(x, y, w, h, fill, stroke="none", strokeWidth=0.1):
    s = f'  <rect x="{x:.3f}" y="{y:.3f}" width="{w:.3f}" height="{h:.3f}" fill="{fill}"'
    if stroke != "none":
        s += f' stroke="{stroke}" stroke-width="{strokeWidth}"'
    return s + '/>\n'


def whiteRuns(black):
    """Huecos a calar: corridas horizontales de celdas blancas (fila, col0, largo)."""
    runs = []
    n = black.shape[0]
    for r in range(n):
        c = 0
        while c < n:
            if not black[r, c]:
                c0 = c
                while c < n and not black[r, c]:
                    c += 1
                runs.append((r, c0, c - c0))
            else:
                c += 1
    return runs


def generateMarker(markerId, size):
    module = size / MODULES
    base = size + 2 * module  # quiet zone de 1 módulo por lado
    black = markerMatrix(markerId)

    # Capa negra: cuadrado exterior + huecos blancos
    svg = svgHeader(size)
    svg += rect(0, 0, size, size, "black")
    for r, c0, length in whiteRuns(black):
        svg += rect(c0 * module, r * module, length * module, module, "white")
    svg += '</svg>\n'
    with open(os.path.join(OUTDIR, f"marker_{markerId}_negro.svg"), "w") as f:
        f.write(svg)

    # Base blanca: cuadrado con quiet zone + contorno rojo de posicionamiento
    svg = svgHeader(base)
    svg += rect(0, 0, base, base, "white", stroke="#999")
    svg += rect(module, module, size, size, "none", stroke="red")
    svg += '</svg>\n'
    with open(os.path.join(OUTDIR, f"marker_{markerId}_blanco.svg"), "w") as f:
        f.write(svg)

    return blackIslands(black)


def cutRect(x, y, w, h):
    """Rectángulo de CORTE: contorno rojo hairline, sin relleno."""
    return (f'  <rect x="{x:.3f}" y="{y:.3f}" width="{w:.3f}" height="{h:.3f}" '
            f'fill="none" stroke="red" stroke-width="0.1"/>\n')


def engraveText(x, y, text, sizeMm=5.0):
    """Texto de GRABADO (azul). Convertir a trazos en Inkscape si el servicio
    de corte lo pide (Trayecto → Objeto a trayecto)."""
    return (f'  <text x="{x:.3f}" y="{y:.3f}" font-size="{sizeMm:.1f}" '
            f'fill="blue" font-family="sans-serif" '
            f'text-anchor="middle">{text}</text>\n')


def generateSheet(ids, size, gap=3.0, margin=5.0):
    """Lámina única: bases blancas + capas negras caladas, todo líneas de corte."""
    module = size / MODULES
    base = size + 2 * module
    cols = 3
    legendH = 7.0

    rowsW = (len(ids) + cols - 1) // cols
    rowsB = rowsW
    sheetW = 2 * margin + cols * base + (cols - 1) * gap
    whiteH = rowsW * base + (rowsW - 1) * gap
    blackH = rowsB * size + (rowsB - 1) * gap
    sheetH = margin + legendH + whiteH + 2 * gap + blackH + margin

    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" '
           f'width="{sheetW:.3f}mm" height="{sheetH:.3f}mm" '
           f'viewBox="0 0 {sheetW:.3f} {sheetH:.3f}">\n')
    svg += engraveText(sheetW / 2, margin + 4.5,
                       f'ROJO=corte · AZUL=grabado · marker {size:.0f}mm · '
                       f'base {base:.1f}mm · acrilico blanco 1.5mm', 4.0)

    islandsAll = {}
    yTop = margin + legendH

    # Bloque superior: bases blancas (con guía de pegado + id grabados)
    for i, markerId in enumerate(ids):
        x = margin + (i % cols) * (base + gap)
        y = yTop + (i // cols) * (base + gap)
        svg += cutRect(x, y, base, base)
        svg += (f'  <rect x="{x + module:.3f}" y="{y + module:.3f}" '
                f'width="{size:.3f}" height="{size:.3f}" fill="none" '
                f'stroke="blue" stroke-width="0.1"/>\n')
        svg += engraveText(x + base / 2, y + base / 2, f'id {markerId}')

    # Bloque inferior: capas negras caladas (id grabado en el borde inferior,
    # que es siempre negro — queda bajo el spray)
    yBlack = yTop + whiteH + 2 * gap
    for i, markerId in enumerate(ids):
        x = margin + (i % cols) * (size + gap)
        y = yBlack + (i // cols) * (size + gap)
        black = markerMatrix(markerId)
        svg += cutRect(x, y, size, size)
        for r, c0, length in whiteRuns(black):
            svg += cutRect(x + c0 * module, y + r * module,
                           length * module, module)
        svg += engraveText(x + size / 2, y + size - module / 2 + 1.5,
                           f'id {markerId}', 4.0)
        islands = blackIslands(black)
        if islands:
            islandsAll[markerId] = islands

    svg += '</svg>\n'
    path = os.path.join(OUTDIR, f'sheet_corte_markers_{size:.0f}mm.svg')
    with open(path, 'w') as f:
        f.write(svg)
    return path, sheetW, sheetH, islandsAll


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ids", nargs="*", type=int, default=list(range(9)),
                    help="ids de marker a generar (default: 0-8)")
    ap.add_argument("--size", type=float, default=80.0,
                    help="lado exterior del borde negro en mm (default: 80)")
    ap.add_argument("--sheet", action="store_true",
                    help="una sola lámina con todos los cortes (blancas + negras)")
    a = ap.parse_args()

    os.makedirs(OUTDIR, exist_ok=True)
    module = a.size / MODULES
    print(f"Marker {a.size:.0f}x{a.size:.0f}mm · módulo {module:.3f}mm · "
          f"base blanca {a.size + 2 * module:.3f}mm · salida: {OUTDIR}")

    if a.sheet:
        path, w, h, islands = generateSheet(a.ids, a.size)
        print(f"\nLámina de corte: {path}")
        print(f"Tamaño requerido: {w:.0f} x {h:.0f} mm "
              f"(si tu lámina es menor, generá subconjuntos de ids)")
        for markerId, cells in islands.items():
            print(f"  ⚠ marker {markerId}: {len(cells)} celda(s) SUELTA(S) al "
                  f"calar {cells} — recogerlas de la cama y pegarlas aparte")
        print("Antes de mandar a cortar: revisar en Inkscape, unir huecos "
              "adyacentes con Trayecto→Unión,\ny convertir los textos azules a "
              "trayectos si el servicio lo pide.")
        return

    for markerId in a.ids:
        islands = generateMarker(markerId, a.size)
        note = (f"⚠ {len(islands)} celda(s) negra(s) SUELTA(S) al calar: {islands}"
                if islands else "ok (capa negra en una sola pieza)")
        print(f"  marker {markerId}: {note}")

    print("\nRecordatorios: unir huecos con Path→Union en Inkscape antes de cortar;")
    print("al desplegar cambiar marker_size_mm a", int(a.size), "en configSystem.json.")


if __name__ == "__main__":
    main()
