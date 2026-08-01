#!/usr/bin/env python3
"""Figura de configuraciones de escenario para el paper — laboratorio y simulación.

    python figura_escenarios.py

Deja `scenario_configurations_lab.pdf` y `scenario_configurations_sim.pdf` en la
carpeta del paper.

Las POSICIONES del laboratorio salen de medir los videos cenitales (calibración
con residuo mediano de 3-6 mm); los TAMAÑOS se dibujan nominales, porque la
medición sobre el video infla cada lado ~1.4x — la homografía se ajusta al plano
de los marcadores y las cajas son más altas que ellos. Las de simulación son
exactas, salen del plan que generó los mundos.
"""
import json
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

BASE = os.path.dirname(os.path.abspath(__file__))
PAPER = os.path.join(BASE, '..', 'Docs', 'paper', 'nuevo paper')
D_ROBOT = 105.0

# Tamaño nominal de las cajas, en mm. El delgado es 0.74 y el grueso 2.12 veces
# el área del robot; el lado largo del grueso cruza la barrera.
NOMINAL = {'D': (80, 80), 'G': (80, 230)}

ETIQ = {'BosqueD_D2': r'$0.7a$ obstacles, $2d$ gaps',
        'BosqueD_D4': r'$0.7a$ obstacles, $4d$ gaps',
        'BosqueG_D2': r'$2.1a$ obstacles, $2d$ gaps',
        'BosqueG_D4': r'$2.1a$ obstacles, $4d$ gaps'}
ORDER = ['BosqueD_D2', 'BosqueD_D4', 'BosqueG_D2', 'BosqueG_D4']

OBST, STRUCT, EDGE = '#e8a33d', '#9a9a96', '#4a4a48'
WALL, INK, INK2 = '#2f5aa8', '#0b0b0b', '#5b5c60'


def limpia_lab(obs, nominal, arena):
    """Descarta artefactos de detección y devuelve (x, y) con tamaño nominal.

    Primero descarta lo que cae fuera de la arena en x (una caja no puede estar
    más allá de una pared lateral); en y se tolera más margen, porque el borde
    superior e inferior están lejos del centro de la imagen y ahí el paralaje
    empuja los centros hacia afuera.

    Después agrupa por columnas y descarta las columnas de UNA sola caja. La
    barrera está hecha de columnas, así que una detección suelta y lejos de
    todas es el marco de madera del montaje. Filtrar por proporción no sirve:
    las cajas gruesas son alargadas por diseño y se irían con las astillas.
    """
    W, H = arena
    dentro = [(x, y) for x, y, w, h in obs
              if 0 <= x <= W and -150 <= y <= H + 150]
    col = {}
    for x, y in dentro:
        col.setdefault(round(x / 250) * 250, []).append((x, y))
    if len(col) > 1:
        col = {k: v for k, v in col.items() if len(v) > 1} or col
    pts = sorted((p for v in col.values() for p in v), key=lambda p: (p[0], p[1]))

    # Las cajas contra la pared salen medidas FUERA de la arena: la cámara ve
    # menos que la arena completa, así que en los bordes la homografía
    # extrapola y el paralaje empuja el centro hacia afuera. Físicamente no
    # pueden salirse, de modo que se acotan a quedar completas adentro.
    bw, bh = nominal
    return [(min(max(x, bw / 2), W - bw / 2),
             min(max(y, bh / 2), H - bh / 2)) for x, y in pts]


def panel(ax, arena, cajas, estructurales, target, titulo, hueco=None):
    W, H = arena
    ax.add_patch(Rectangle((0, 0), W, H, fill=False, ec=WALL, lw=1.6))
    for x, y, w, h in estructurales:
        ax.add_patch(Rectangle((x - w / 2, y - h / 2), w, h,
                               fc=STRUCT, ec=EDGE, lw=.5))
    for x, y, w, h in cajas:
        ax.add_patch(Rectangle((x - w / 2, y - h / 2), w, h,
                               fc=OBST, ec=EDGE, lw=.5))
    ax.plot(*target, marker='*', ms=13, color=OBST, mec=EDGE, mew=.5,
            linestyle='none')
    ax.set_xlim(-.05 * W, 1.05 * W)
    ax.set_ylim(-.05 * H, 1.05 * H)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    sub = titulo if hueco is None else f'{titulo}\n(measured gap {hueco:.1f}$d$)'
    ax.set_title(sub, fontsize=8.5, color=INK)


def gap_medido(cajas, lado_transversal):
    """Hueco libre entre cajas contiguas de la columna más poblada."""
    col = {}
    for x, y in cajas:
        col.setdefault(round(x / 250) * 250, []).append(y)
    ys = sorted(max(col.values(), key=len))
    if len(ys) < 2:
        return None
    seps = sorted(ys[i + 1] - ys[i] for i in range(len(ys) - 1))
    # mediana, no media: una caja de la punta mal ubicada no debe mover el hueco
    med = seps[len(seps) // 2] if len(seps) % 2 else (seps[len(seps) // 2 - 1] + seps[len(seps) // 2]) / 2
    return (med - lado_transversal) / D_ROBOT


def figura_lab():
    geo = json.load(open(os.path.join(BASE, 'analisis_30-07',
                                      'geometria_escenarios.json')))
    arena, target = (2400.0, 1750.0), (2200.0, 851.0)
    fig, axes = plt.subplots(2, 2, figsize=(6.6, 5.4))
    for ax, sc in zip(axes.ravel(), ORDER):
        w, h = NOMINAL['G' if 'BosqueG' in sc else 'D']
        pts = limpia_lab(geo[sc]['obstaculos'], (w, h), arena)
        cajas = [(x, y, w, h) for x, y in pts]
        panel(ax, arena, cajas, [], target, ETIQ[sc], gap_medido(pts, h))
    fig.suptitle('Obstacle configurations, physical arena '
                 r'($2.40 \times 1.75$ m)', fontsize=10, y=.98)
    fig.tight_layout(rect=(0, 0, 1, .95))
    out = os.path.join(PAPER, 'scenario_configurations_lab.pdf')
    fig.savefig(out, bbox_inches='tight')
    print(' ', out)


def figura_sim():
    geo = json.load(open(os.path.join(BASE, 'simulacion_31-07',
                                      'geometria_escenarios.json')))
    arena = tuple(float(v) for v in geo['arena_mm'])
    target = tuple(float(v) for v in geo['punto_de_encuentro_mm'])
    fig, axes = plt.subplots(2, 2, figsize=(6.6, 5.4))
    for ax, sc in zip(axes.ravel(), ORDER):
        obs = geo['escenarios'][sc]['obstaculos_x_y_ancho_alto_estructural']
        cajas = [(x, y, w, h) for x, y, w, h, e in obs if not e]
        estr = [(x, y, w, h) for x, y, w, h, e in obs if e]
        pts = [(x, y) for x, y, *_ in cajas]
        alto = cajas[0][3] if cajas else 0
        panel(ax, arena, cajas, estr, target, ETIQ[sc], gap_medido(pts, alto))
    fig.suptitle('Obstacle configurations, simulated arena '
                 r'($3.80 \times 2.80$ m, ten agents)', fontsize=10, y=.98)
    fig.tight_layout(rect=(0, 0, 1, .95))
    out = os.path.join(PAPER, 'scenario_configurations_sim.pdf')
    fig.savefig(out, bbox_inches='tight')
    print(' ', out)


if __name__ == '__main__':
    plt.rcParams.update({'font.size': 9, 'text.color': INK,
                         'figure.facecolor': 'white',
                         'axes.facecolor': 'white'})
    figura_lab()
    figura_sim()
