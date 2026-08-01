#!/usr/bin/env python3
"""Figuras del paper de topología — el MISMO juego para laboratorio y simulación.

    python figuras_paper.py            # lab, sim y la comparación
    python figuras_paper.py sim

Por dataset salen cuatro: reparto de fases por corrida, trayectorias sobre la
geometría real, compactación en el tiempo y boxplots de resumen. Se generan con
el mismo código a propósito — si una diferencia aparece en la figura, es de los
datos y no del tratamiento.
"""
import csv
import json
import os
import statistics as st
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

BASE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE)
os.chdir(BASE)
import analyze_logs as A  # noqa: E402

OUT = os.path.join(BASE, 'analisis_30-07')
PKG_SIM = os.path.join(BASE, 'simulacion_31-07')
SIM_PLAN = os.path.expanduser(
    '~/Documents/AttaBot-Sim/tools/plan_topologia_sim.json')

ORDER = ['SinObs', 'BosqueD_D2', 'BosqueD_D4', 'BosqueG_D2', 'BosqueG_D4']
# Paleta categórica validada (slots 1-5, en orden adyacente)
COL = {'SinObs': '#2a78d6', 'BosqueD_D2': '#eb6834', 'BosqueD_D4': '#1baf7a',
       'BosqueG_D2': '#eda100', 'BosqueG_D4': '#e87ba4'}
LAB_C, SIM_C = '#2a78d6', '#eb6834'
INK, INK2, GRID = '#0b0b0b', '#52514e', '#e6e5e1'

plt.rcParams.update({'font.size': 9, 'axes.edgecolor': GRID,
                     'axes.labelcolor': INK2, 'xtick.color': INK2,
                     'ytick.color': INK2, 'text.color': INK,
                     'figure.facecolor': 'white', 'axes.facecolor': 'white'})


# ── Descriptores de dataset ─────────────────────────────────────────────────
# Lo único que cambia entre lab y sim: cómo se llaman las corridas, de dónde
# sale la geometría y qué tan grande es la arena. El resto es común.

def _parse_obstaculos(specs):
    """'x,y,w[,h][,S]' → [(x, y, w, h)] — mismo formato que gen_world."""
    out = []
    for spec in specs:
        p = [t.strip() for t in spec.split(',')]
        if p[-1].upper() == 'S':
            p = p[:-1]
        v = [float(t) for t in p]
        out.append((v[0], v[1], v[2], v[3] if len(v) >= 4 else v[2]))
    return out


def dataset(name):
    if name == 'lab':
        geo = json.load(open(f'{OUT}/geometria_escenarios.json'))
        return {
            'nombre': 'laboratorio · 4 robots · arena 2400×1750mm',
            'slug': 'lab', 'arena': (2400.0, 1750.0),
            'target': (2200.0, 851.0), 'dir': BASE,
            'runs_csv': f'{OUT}/lab_runs.csv',
            'robots_csv': f'{OUT}/lab_robots.csv',
            'corridas': [(sc, ar, f'{sc}_{ar}')
                         for sc in ORDER for ar in ('ASop', 'APar', 'ALin')],
            'obstaculos': {sc: [tuple(b[:4]) for b in geo[sc]['obstaculos']]
                           for sc in ORDER},
        }
    plan = json.load(open(SIM_PLAN))
    man = list(csv.DictReader(open(f'{PKG_SIM}/manifiesto.csv')))
    w, h = (float(v) * 1000 for v in plan['arena'].split(','))
    return {
        'nombre': 'simulación · 10 robots · arena 3800×2800mm',
        'slug': 'sim', 'arena': (w, h), 'dir': PKG_SIM,
        'target': tuple(float(v) for v in plan['destino']),
        'runs_csv': f'{PKG_SIM}/sim_runs.csv',
        'robots_csv': f'{PKG_SIM}/sim_robots.csv',
        'corridas': [(r['scenario'], f'r{r["rep"]}', r['session'])
                     for r in sorted(man, key=lambda r: (
                         ORDER.index(r['scenario']), r['rep']))],
        'obstaculos': {s['name']: _parse_obstaculos(s.get('obstacles', []))
                       for s in plan['scenarios']},
    }


def load(ds):
    """{tag: {tracks, phases, series}} — recalculado desde los logs crudos."""
    out = {}
    for sc, etiq, tag in ds['corridas']:
        tr = A.real_tracks(A.load_positions(
            os.path.join(ds['dir'], 'PositionLogs', f'{tag}.csv')))
        ev = A.load_console(
            os.path.join(ds['dir'], 'ConsoleLogs', f'{tag}.csv'))
        out[tag] = {'sc': sc, 'etiq': etiq, 'tracks': tr,
                    'phases': A.detect_phases(tr, ev),
                    'series': A.group_series(tr)}
    return out


def _limpiar(ax, ejes='y'):
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    ax.grid(axis=ejes, color=GRID, lw=.8)
    ax.set_axisbelow(True)


# ── Las cuatro figuras ──────────────────────────────────────────────────────

def fig_fases(ds, runs):
    """Barras apiladas: dónde se va realmente el tiempo de cada corrida."""
    n = len(ds['corridas'])
    fig, ax = plt.subplots(figsize=(9, 0.42 * n + 2.0))
    labels, y = [], 0
    for i, (sc, etiq, tag) in enumerate(ds['corridas']):
        p = runs[tag]['phases']
        ax.barh(y, p['rw_s'], color=COL[sc], height=.62)
        ax.barh(y, p['dead_s'], left=p['rw_s'], color='#d8d7d2', height=.62)
        ax.barh(y, p['meet_s'], left=p['rw_s'] + p['dead_s'],
                color=COL[sc], alpha=.45, height=.62)
        tot = p['rw_s'] + p['dead_s'] + p['meet_s']
        ax.text(tot + 4, y, f"{p['meet_s']:.0f}s", va='center', fontsize=8,
                color=INK2)
        labels.append(f'{sc}  {etiq}')
        y += 1 + (.5 if (i + 1) % 3 == 0 else 0)
    ax.set_yticks([i + (i // 3) * .5 for i in range(n)])
    ax.set_yticklabels(labels, fontsize=8)
    ax.invert_yaxis()
    ax.set_xlabel('segundos desde el inicio del log')
    ax.set_title(f'Reparto de tiempo por corrida — {ds["nombre"]}\n'
                 'sólido = caminata aleatoria · gris = tiempo muerto · '
                 'claro = congregación', loc='left', fontsize=10)
    ax.spines['left'].set_visible(False)
    _limpiar(ax, 'x')
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig_fases_{ds["slug"]}.png', dpi=150)


def fig_trayectorias(ds, runs):
    W, H = ds['arena']
    fig, axes = plt.subplots(1, 5, figsize=(16, 2.9 * (H / W) / (1750 / 2400)))
    for ax, sc in zip(axes, ORDER):
        for x, y, w, h in ds['obstaculos'][sc]:
            ax.add_patch(Rectangle((x - w / 2, y - h / 2), w, h,
                                   fc='#b9a888', ec='#8a7c60', lw=.6))
        for _sc, _etiq, tag in [c for c in ds['corridas'] if c[0] == sc]:
            r = runs[tag]
            t0 = r['phases']['meet_start_s']
            for tr in r['tracks'].values():
                # Cortar donde se pierde el marker: unir los dos extremos de un
                # hueco dibuja una recta que atraviesa obstáculos y se lee como
                # un recorrido que nunca ocurrió.
                seg, last = [], None
                for t, x, y, *_ in tr:
                    if t < t0:
                        continue
                    if last is not None and t - last > 1.0:
                        if len(seg) > 3:
                            ax.plot([p[0] for p in seg], [p[1] for p in seg],
                                    color=COL[sc], lw=.6, alpha=.5)
                        seg = []
                    seg.append((x, y))
                    last = t
                if len(seg) > 3:
                    ax.plot([p[0] for p in seg], [p[1] for p in seg],
                            color=COL[sc], lw=.6, alpha=.5)
        ax.plot(*ds['target'], marker='+', ms=11, mew=1.8, color=INK)
        ax.add_patch(Rectangle((0, 0), W, H, fill=False, ec=GRID, lw=1.2))
        ax.set_xlim(-.04 * W, 1.04 * W)
        ax.set_ylim(-.04 * H, 1.04 * H)
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        for s in ax.spines.values():
            s.set_visible(False)
        ax.set_title(sc, fontsize=9, color=INK)
    fig.suptitle(f'Trayectorias durante la congregación — {ds["nombre"]}. '
                 'La cruz es el punto de encuentro; las cajas, los obstáculos.',
                 fontsize=10, x=.01, ha='left')
    fig.tight_layout(rect=(0, 0, 1, .92))
    fig.savefig(f'{OUT}/fig_trayectorias_{ds["slug"]}.png', dpi=150)


def fig_compactacion(ds, runs):
    fig, ax = plt.subplots(figsize=(9, 4.4))
    vistos = set()
    for sc, etiq, tag in ds['corridas']:
        r = runs[tag]
        t0 = r['phases']['meet_start_s']
        s = [(x['t'] - t0, x['rms_d']) for x in r['series']
             if 0 <= x['t'] - t0 <= 150]
        if s:
            ax.plot([p[0] for p in s], [p[1] for p in s], color=COL[sc],
                    lw=1.5, alpha=.75, label=None if sc in vistos else sc)
            vistos.add(sc)
    ax.set_xlabel('segundos desde el comando MEET')
    ax.set_ylabel('dispersión RMS al centroide (diámetros)')
    ax.set_title(f'Cómo se aprieta el enjambre — {ds["nombre"]}',
                 loc='left', fontsize=10)
    ax.legend(frameon=False, fontsize=8, ncol=5)
    _limpiar(ax, 'both')
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig_compactacion_{ds["slug"]}.png', dpi=150)


def fig_resumen(ds):
    runs = list(csv.DictReader(open(ds['runs_csv'])))
    rob = list(csv.DictReader(open(ds['robots_csv'])))
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.2))
    for ax, data, ylab, title in (
            (a1, [[float(r['t_conv_frac_s']) for r in runs
                   if r['scenario'] == sc and r['t_conv_frac_s']]
                  for sc in ORDER], 'segundos', 'Tiempo de congregación'),
            (a2, [[float(r['route_ratio']) for r in rob
                   if r['scenario'] == sc] for sc in ORDER],
             'recorrido ÷ recta', 'Ratio de ruta por robot')):
        bp = ax.boxplot(data, patch_artist=True, widths=.55,
                        medianprops=dict(color=INK, lw=1.6),
                        flierprops=dict(marker='o', ms=4, mfc='none', mec=INK2))
        for patch, sc in zip(bp['boxes'], ORDER):
            patch.set(facecolor=COL[sc], alpha=.55, edgecolor=COL[sc])
        for w in bp['whiskers'] + bp['caps']:
            w.set(color=GRID, lw=1.2)
        ax.set_xticks(range(1, 6))
        ax.set_xticklabels(ORDER, rotation=18, ha='right', fontsize=8)
        ax.set_ylabel(ylab)
        ax.set_title(f'{title} — {ds["nombre"]}', loc='left', fontsize=10)
        _limpiar(ax)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig_resumen_{ds["slug"]}.png', dpi=150)


def fig_comparacion():
    """Lab y sim lado a lado.

    En absoluto NO son comparables: distinto número de robots y distinta arena.
    Por eso el panel derecho normaliza cada uno por su PROPIO control sin
    obstáculos — eso sí compara el efecto de la topología y no el tamaño del
    experimento.
    """
    def serie(path):
        out = {}
        for r in csv.DictReader(open(path)):
            if r['t_conv_frac_s']:
                out.setdefault(r['scenario'], []).append(
                    float(r['t_conv_frac_s']))
        return out

    lab = serie(f'{OUT}/lab_runs.csv')
    sim = serie(f'{PKG_SIM}/sim_runs.csv')
    orden = ['SinObs', 'BosqueD_D4', 'BosqueG_D4', 'BosqueD_D2', 'BosqueG_D2']
    etiq = {'SinObs': 'sin\nobstáculos', 'BosqueD_D4': 'delgado\n4d',
            'BosqueG_D4': 'grueso\n4d', 'BosqueD_D2': 'delgado\n2d',
            'BosqueG_D2': 'grueso\n2d'}
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.3))
    w, xs = .38, range(len(orden))
    for ax, norm, ylab, title in (
            (axes[0], False, 'segundos', 'Tiempo de congregación'),
            (axes[1], True, '× su propio control',
             'Normalizado a cada control sin obstáculos')):
        for off, data, col, name in (
                (-w / 2, lab, LAB_C, 'laboratorio · 4 robots'),
                (w / 2, sim, SIM_C, 'simulación · 10 robots')):
            base = st.mean(data['SinObs']) if norm else 1.0
            vals = [st.mean(data[s]) / base for s in orden]
            err = [st.stdev(data[s]) / base if len(data[s]) > 1 else 0
                   for s in orden]
            ax.bar([x + off for x in xs], vals, w, color=col, alpha=.8,
                   label=name)
            ax.errorbar([x + off for x in xs], vals, yerr=err, fmt='none',
                        ecolor=INK2, elinewidth=1, capsize=3)
        ax.set_xticks(list(xs))
        ax.set_xticklabels([etiq[s] for s in orden], fontsize=8)
        ax.set_ylabel(ylab)
        ax.set_title(title, loc='left', fontsize=10)
        ax.legend(frameon=False, fontsize=8)
        _limpiar(ax)
        if norm:
            ax.axhline(1.0, color=INK2, lw=.8, ls=':')
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig_lab_vs_sim.png', dpi=150)


if __name__ == '__main__':
    for a in (sys.argv[1:] or ['lab', 'sim', 'comparacion']):
        if a == 'comparacion':
            fig_comparacion()
            print('  fig_lab_vs_sim.png')
            continue
        ds = dataset(a)
        fig_fases(ds, load(ds))
        fig_trayectorias(ds, load(ds))
        fig_compactacion(ds, load(ds))
        fig_resumen(ds)
        print(f'  fig_*_{ds["slug"]}.png — {ds["nombre"]}')


# ── Figura de resultados con la estructura del paper ────────────────────────
# Tres paneles, las mismas tres metricas y el mismo orden de escenarios que las
# figuras del laboratorio, para que las dos se lean en paralelo.

PAPER_ORDER = ['BosqueD_D2', 'BosqueD_D4', 'BosqueG_D2', 'BosqueG_D4', 'SinObs']
PAPER_ETIQ = ['0.7a-2d', '0.7a-4d', '2.1a-2d', '2.1a-4d', 'NoObs']


def fig_paper(ds):
    runs = list(csv.DictReader(open(ds['runs_csv'])))
    rob = list(csv.DictReader(open(ds['robots_csv'])))
    fig, axes = plt.subplots(3, 1, figsize=(5.4, 9.6))
    series = (
        ('Normalized Robot Distance',
         [[float(r['route_ratio']) for r in rob if r['scenario'] == sc]
          for sc in PAPER_ORDER]),
        ('Aggregation Time (s)',
         [[float(r['t_conv_frac_s']) for r in runs
           if r['scenario'] == sc and r['t_conv_frac_s']]
          for sc in PAPER_ORDER]),
        # Compactacion segun la ecuacion del paper: raiz de la MEDIA de las
        # distancias al cuadrado. El 1/N adentro de la raiz es lo que la hace
        # comparable entre enjambres de distinto tamano.
        ('Swarm Compactness',
         [[float(r['final_rms_d']) for r in runs if r['scenario'] == sc]
          for sc in PAPER_ORDER]),
    )
    for ax, (ylab, data) in zip(axes, series):
        bp = ax.boxplot(data, patch_artist=True, widths=.55,
                        medianprops=dict(color='#c0392b', lw=1.5),
                        flierprops=dict(marker='o', ms=4, mfc='none', mec=INK2))
        for patch, sc in zip(bp['boxes'], PAPER_ORDER):
            patch.set(facecolor=COL[sc], alpha=.5, edgecolor=COL[sc])
        for w in bp['whiskers'] + bp['caps']:
            w.set(color=GRID, lw=1.2)
        for i, vals in enumerate(data, start=1):
            ax.plot([i] * len(vals), vals, 'o', ms=4, mfc=COL[PAPER_ORDER[i - 1]],
                    mec='none', alpha=.45)
        ax.set_xticks(range(1, 6))
        ax.set_xticklabels(PAPER_ETIQ, fontsize=8.5)
        ax.set_xlabel('Scenario Configuration', fontsize=9)
        ax.set_ylabel(ylab, fontsize=9)
        _limpiar(ax)
    fig.tight_layout()
    out = os.path.join(BASE, '..', 'Docs', 'paper', 'nuevo paper',
                       f'results_{ds["slug"]}.pdf')
    fig.savefig(out, bbox_inches='tight')
    print(' ', os.path.normpath(out))
