#!/usr/bin/env python3
"""Figuras del dataset del 30-07 para el paper de topología."""
import csv
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

sys.path.insert(0, '/home/thrain/Documents/Atta-Bot-P_ed/Base')
os.chdir('/home/thrain/Documents/Atta-Bot-P_ed/Base')
import analyze_logs as A

OUT = '/home/thrain/Documents/Atta-Bot-P_ed/Base/analisis_30-07'
ARENA_W, ARENA_H = 2400, 1750
D = A.ATTA_DIAMETER_MM

ORDER = ['SinObs', 'BosqueD_D2', 'BosqueD_D4', 'BosqueG_D2', 'BosqueG_D4']
ARR = ['ASop', 'APar', 'ALin']
# Paleta categórica validada (slots 1-5, orden adyacente)
COL = {'SinObs': '#2a78d6', 'BosqueD_D2': '#eb6834', 'BosqueD_D4': '#1baf7a',
       'BosqueG_D2': '#eda100', 'BosqueG_D4': '#e87ba4'}
INK, INK2, GRID = '#0b0b0b', '#52514e', '#e6e5e1'
GEO = json.load(open(f'{OUT}/geometria_escenarios.json'))

plt.rcParams.update({'font.size': 9, 'axes.edgecolor': GRID,
                     'axes.labelcolor': INK2, 'xtick.color': INK2,
                     'ytick.color': INK2, 'text.color': INK,
                     'figure.facecolor': 'white', 'axes.facecolor': 'white'})


def load():
    runs = {}
    for sc in ORDER:
        for ar in ARR:
            n = f'{sc}_{ar}'
            tr = A.real_tracks(A.load_positions(f'PositionLogs/{n}.csv'))
            ev = A.load_console(f'ConsoleLogs/{n}.csv')
            ph = A.detect_phases(tr, ev)
            runs[n] = {'sc': sc, 'ar': ar, 'tracks': tr, 'phases': ph,
                       'series': A.group_series(tr)}
    return runs


def fig_fases(runs):
    """Barras apiladas: dónde se va realmente el tiempo de cada corrida."""
    fig, ax = plt.subplots(figsize=(9, 6.4))
    labels, y = [], 0
    for sc in ORDER:
        for ar in ARR:
            r = runs[f'{sc}_{ar}']
            p = r['phases']
            ax.barh(y, p['rw_s'], left=0, color=COL[sc], height=.62)
            ax.barh(y, p['dead_s'], left=p['rw_s'], color='#d8d7d2', height=.62)
            ax.barh(y, p['meet_s'], left=p['rw_s'] + p['dead_s'],
                    color=COL[sc], alpha=.45, height=.62)
            tot = p['rw_s'] + p['dead_s'] + p['meet_s']
            ax.text(tot + 4, y, f"{p['meet_s']:.0f}s", va='center',
                    fontsize=8, color=INK2)
            labels.append(f'{sc}  {ar}')
            y += 1
        y += .5
    ax.set_yticks([i + (i // 3) * .5 for i in range(15)])
    ax.set_yticklabels(labels, fontsize=8)
    ax.invert_yaxis()
    ax.set_xlabel('segundos desde el inicio del log')
    ax.set_title('Dónde se va el tiempo de cada corrida\n'
                 'sólido = caminata aleatoria · gris = tiempo muerto del '
                 'operador · claro = congregación', loc='left', fontsize=10)
    for s in ('top', 'right', 'left'):
        ax.spines[s].set_visible(False)
    ax.grid(axis='x', color=GRID, lw=.8)
    ax.set_axisbelow(True)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig_fases.png', dpi=150)


def fig_trayectorias(runs):
    """Qué hicieron los robots, con los obstáculos medidos desde el video."""
    fig, axes = plt.subplots(1, 5, figsize=(16, 3.1))
    for ax, sc in zip(axes, ORDER):
        for x, y, w, h in GEO[sc]['obstaculos']:
            ax.add_patch(Rectangle((x - w / 2, y - h / 2), w, h,
                                   fc='#b9a888', ec='#8a7c60', lw=.6))
        for ar in ARR:
            r = runs[f'{sc}_{ar}']
            t0 = r['phases']['meet_start_s']
            for tr in r['tracks'].values():
                # Cortar donde la cámara perdió el marker: unir los dos extremos
                # de un hueco dibuja una recta larga que atraviesa obstáculos y
                # se lee como una trayectoria que nunca ocurrió.
                seg = []
                last_t = None
                for t, x, y, *_ in tr:
                    if t < t0:
                        continue
                    if last_t is not None and t - last_t > 1.0:
                        if len(seg) > 3:
                            ax.plot([p[0] for p in seg], [p[1] for p in seg],
                                    color=COL[sc], lw=.7, alpha=.55)
                        seg = []
                    seg.append((x, y))
                    last_t = t
                if len(seg) > 3:
                    ax.plot([p[0] for p in seg], [p[1] for p in seg],
                            color=COL[sc], lw=.7, alpha=.55)
        ax.add_patch(Rectangle((0, 0), ARENA_W, ARENA_H, fill=False,
                               ec=GRID, lw=1.2))
        ax.set_xlim(-80, ARENA_W + 80)
        ax.set_ylim(-80, ARENA_H + 80)
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        for s in ax.spines.values():
            s.set_visible(False)
        ax.set_title(sc, fontsize=9, color=INK)
    fig.suptitle('Trayectorias durante la congregación (las 3 corridas de cada '
                 'escenario superpuestas); las cajas son los obstáculos medidos '
                 'desde el video', fontsize=10, x=.01, ha='left')
    fig.tight_layout(rect=(0, 0, 1, .93))
    fig.savefig(f'{OUT}/fig_trayectorias.png', dpi=150)


def fig_compactacion(runs):
    """La métrica de JC en el tiempo, alineada al comando MEET."""
    fig, ax = plt.subplots(figsize=(9, 4.4))
    for sc in ORDER:
        for i, ar in enumerate(ARR):
            r = runs[f'{sc}_{ar}']
            t0 = r['phases']['meet_start_s']
            s = [(x['t'] - t0, x['rms_d']) for x in r['series']
                 if 0 <= x['t'] - t0 <= 150]
            if s:
                ax.plot([p[0] for p in s], [p[1] for p in s], color=COL[sc],
                        lw=1.6, alpha=.75, label=sc if i == 0 else None)
    ax.set_xlabel('segundos desde el comando MEET')
    ax.set_ylabel('dispersión RMS al centroide (diámetros)')
    ax.set_title('Cómo se aprieta el enjambre después del comando',
                 loc='left', fontsize=10)
    ax.legend(frameon=False, fontsize=8, ncol=5)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    ax.grid(color=GRID, lw=.8)
    ax.set_axisbelow(True)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig_compactacion.png', dpi=150)


def fig_resumen():
    """Boxplots del tiempo de congregación y del ratio de ruta."""
    runs = list(csv.DictReader(open(f'{OUT}/lab_runs.csv')))
    rob = list(csv.DictReader(open(f'{OUT}/lab_robots.csv')))
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.2))
    for ax, data, ylab, title in (
            (a1, [[float(r['t_conv_frac_s']) for r in runs
                   if r['scenario'] == sc and r['t_conv_frac_s']]
                  for sc in ORDER], 'segundos',
             'Tiempo de congregación (R=550mm, los 4 robots)'),
            (a2, [[float(r['route_ratio']) for r in rob
                   if r['scenario'] == sc] for sc in ORDER], 'recorrido ÷ recta',
             'Ratio de ruta por robot')):
        bp = ax.boxplot(data, patch_artist=True, widths=.55,
                        medianprops=dict(color=INK, lw=1.6),
                        flierprops=dict(marker='o', ms=4, mfc='none',
                                        mec=INK2))
        for patch, sc in zip(bp['boxes'], ORDER):
            patch.set(facecolor=COL[sc], alpha=.55, edgecolor=COL[sc])
        for w in bp['whiskers'] + bp['caps']:
            w.set(color=GRID, lw=1.2)
        ax.set_xticks(range(1, 6))
        ax.set_xticklabels(ORDER, rotation=18, ha='right', fontsize=8)
        ax.set_ylabel(ylab)
        ax.set_title(title, loc='left', fontsize=10)
        for s in ('top', 'right'):
            ax.spines[s].set_visible(False)
        ax.grid(axis='y', color=GRID, lw=.8)
        ax.set_axisbelow(True)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig_resumen.png', dpi=150)


if __name__ == '__main__':
    r = load()
    fig_fases(r)
    fig_trayectorias(r)
    fig_compactacion(r)
    fig_resumen()
    print('figuras listas en', OUT)
