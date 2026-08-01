#!/usr/bin/env python3
"""Arma la página de revisión del dataset con las figuras embebidas."""
import base64
import csv
import json
import pathlib

OUT = pathlib.Path('/home/thrain/Documents/Atta-Bot-P_ed/Base/analisis_30-07')
SCRATCH = pathlib.Path(
    '/tmp/claude-1000/-home-thrain-Documents-Atta-Bot-P-ed/'
    'a573861c-9ae3-4c5f-aeab-d76ef49b52de/scratchpad')

ORDER = ['SinObs', 'BosqueD_D2', 'BosqueD_D4', 'BosqueG_D2', 'BosqueG_D4']
COL = {'SinObs': '#2a78d6', 'BosqueD_D2': '#eb6834', 'BosqueD_D4': '#1baf7a',
       'BosqueG_D2': '#eda100', 'BosqueG_D4': '#e87ba4'}


def img(name):
    b = base64.b64encode((OUT / name).read_bytes()).decode()
    return f'data:image/png;base64,{b}'


runs = {r['session']: r for r in csv.DictReader(open(OUT / 'lab_runs.csv'))}
geo = json.load(open(OUT / 'geometria_escenarios.json'))

HUECO = {'SinObs': '—', 'BosqueD_D2': '2.1 d', 'BosqueD_D4': '4.5 d',
         'BosqueG_D2': '2.2 d', 'BosqueG_D4': '4.6 d'}
CAJA = {'SinObs': '—', 'BosqueD_D2': 'delgado', 'BosqueD_D4': 'delgado',
        'BosqueG_D2': 'grueso', 'BosqueG_D4': 'grueso'}


def med(vals):
    v = sorted(vals)
    if not v:
        return None
    n = len(v)
    return v[n // 2] if n % 2 else (v[n // 2 - 1] + v[n // 2]) / 2


rows_esc = []
for sc in ORDER:
    rr = [r for r in runs.values() if r['scenario'] == sc]
    tc = [float(r['t_conv_frac_s']) for r in rr if r['t_conv_frac_s']]
    rows_esc.append(f'''<tr>
      <th scope="row"><span class="dot" style="--c:{COL[sc]}"></span>{sc}</th>
      <td>{CAJA[sc]}</td><td>{HUECO[sc]}</td>
      <td>{len(geo[sc]["obstaculos"])}</td>
      <td class="num">{med(tc):.1f}</td>
      <td class="num">{len(tc)}/3</td>
      <td class="num">{med([float(r["final_compaction_d"]) for r in rr]):.2f}</td>
    </tr>''')

rows_run = []
for sc in ORDER:
    for ar in ('ASop', 'APar', 'ALin'):
        r = runs[f'{sc}_{ar}']
        tc = (f'{float(r["t_conv_frac_s"]):.1f}' if r['t_conv_frac_s']
              else '<span class="no">no convergió</span>')
        rows_run.append(f'''<tr>
          <th scope="row"><span class="dot" style="--c:{COL[sc]}"></span>{sc}</th>
          <td>{ar}</td>
          <td class="num">{float(r["rw_s"]):.0f}</td>
          <td class="num muted">{float(r["dead_s"]):.0f}</td>
          <td class="num">{float(r["meet_s"]):.0f}</td>
          <td class="num">{tc}</td>
          <td class="num">{float(r["final_compaction_d"]):.2f}</td>
        </tr>''')

HTML = f'''<title>Dataset del 30-07 — congregación bajo topologías de obstáculos</title>
<style>
  :root {{
    color-scheme: light;
    --bg: #f6f6f4; --card: #fff; --ink: #141517; --ink-2: #5b5c60;
    --line: #e3e3df; --line-2: #efefec; --accent: #2a78d6; --warm: #b9a888;
  }}
  @media (prefers-color-scheme: dark) {{
    :root:where(:not([data-theme="light"])) {{
      color-scheme: dark;
      --bg: #131415; --card: #1b1c1e; --ink: #f1f1ef; --ink-2: #a2a3a1;
      --line: #2b2c2f; --line-2: #232426; --accent: #5c9ae8; --warm: #c0b092;
    }}
  }}
  :root[data-theme="dark"] {{
    color-scheme: dark;
    --bg: #131415; --card: #1b1c1e; --ink: #f1f1ef; --ink-2: #a2a3a1;
    --line: #2b2c2f; --line-2: #232426; --accent: #5c9ae8; --warm: #c0b092;
  }}
  body {{
    background: var(--bg); color: var(--ink);
    font-family: system-ui, -apple-system, "Segoe UI", sans-serif;
    font-size: 16px; line-height: 1.65;
    margin: 0; padding: 3rem 1.25rem 5rem;
  }}
  .wrap {{ max-width: 62rem; margin: 0 auto;
           display: flex; flex-direction: column; gap: 3.25rem; }}
  .prose {{ max-width: 40rem; display: flex; flex-direction: column; gap: .9rem; }}
  h1, h2, h3 {{ font-family: ui-serif, Georgia, "Times New Roman", serif;
                font-weight: 600; text-wrap: balance; margin: 0; line-height: 1.25; }}
  h1 {{ font-size: 2.1rem; letter-spacing: -.015em; }}
  h2 {{ font-size: 1.4rem; }}
  h3 {{ font-size: 1.05rem; }}
  p {{ margin: 0; }}
  .eyebrow {{ font-family: ui-monospace, "SF Mono", Menlo, monospace;
              font-size: .72rem; letter-spacing: .12em; text-transform: uppercase;
              color: var(--ink-2); margin: 0 0 .55rem; }}
  header {{ border-bottom: 1px solid var(--line); padding-bottom: 2rem; }}
  section {{ display: flex; flex-direction: column; gap: 1.25rem; }}
  figure {{ margin: 0; background: #fff; border: 1px solid var(--line);
            border-radius: 3px; padding: .75rem; overflow-x: auto; }}
  figure img {{ display: block; width: 100%; min-width: 640px; height: auto; }}
  figcaption {{ font-size: .85rem; color: var(--ink-2); margin-top: .7rem;
                padding: 0 .25rem; min-width: 640px; }}
  .stats {{ display: grid; gap: 1px; background: var(--line);
            grid-template-columns: repeat(auto-fit, minmax(11rem, 1fr));
            border: 1px solid var(--line); border-radius: 3px; overflow: hidden; }}
  .stat {{ background: var(--card); padding: 1rem 1.1rem; }}
  .stat b {{ display: block; font-size: 1.7rem; font-weight: 600;
             font-variant-numeric: tabular-nums; letter-spacing: -.02em; }}
  .stat span {{ font-size: .8rem; color: var(--ink-2); }}
  .tw {{ overflow-x: auto; border: 1px solid var(--line); border-radius: 3px; }}
  table {{ border-collapse: collapse; width: 100%; font-size: .875rem;
           background: var(--card); }}
  th, td {{ text-align: left; padding: .55rem .8rem;
            border-bottom: 1px solid var(--line-2); white-space: nowrap; }}
  thead th {{ font-family: ui-monospace, Menlo, monospace; font-weight: 500;
              font-size: .72rem; letter-spacing: .06em; text-transform: uppercase;
              color: var(--ink-2); background: var(--bg); }}
  tbody th {{ font-weight: 500; }}
  tbody tr:last-child td, tbody tr:last-child th {{ border-bottom: 0; }}
  .num {{ text-align: right; font-variant-numeric: tabular-nums; }}
  .muted {{ color: var(--ink-2); }}
  .no {{ color: var(--ink-2); font-style: italic; }}
  .dot {{ display: inline-block; width: .6rem; height: .6rem; border-radius: 50%;
          background: var(--c); margin-right: .5rem; vertical-align: baseline; }}
  .note {{ border-left: 2px solid var(--accent); padding: .1rem 0 .1rem 1rem;
           color: var(--ink-2); font-size: .93rem;
           display: flex; flex-direction: column; gap: .6rem; }}
  code {{ font-family: ui-monospace, Menlo, monospace; font-size: .88em;
          background: var(--line-2); padding: .1em .35em; border-radius: 2px; }}
  ul {{ margin: 0; padding-left: 1.15rem; display: flex;
        flex-direction: column; gap: .45rem; }}
  a {{ color: var(--accent); }}
  a:focus-visible, [tabindex]:focus-visible {{ outline: 2px solid var(--accent);
                                               outline-offset: 2px; }}
</style>

<div class="wrap">
<header class="prose">
  <p class="eyebrow">Laboratorio · 30 julio 2026 · 4 AttaBots</p>
  <h1>Congregación bajo distintas topologías de obstáculos</h1>
  <p>Quince corridas: cinco montajes de arena × tres formas de arrancar. Cada
  corrida es un minuto de caminata aleatoria y después un comando
  <code>MEET</code> que los junta sobre un punto contra la pared. Esta página
  reúne cuánto duró realmente cada fase, qué geometría tenía cada escenario y
  qué dicen las métricas.</p>
</header>

<section>
  <div class="stats">
    <div class="stat"><b>15</b><span>corridas, todas con log y video</span></div>
    <div class="stat"><b>20&nbsp;s</b><span>mediana de tiempo muerto por corrida</span></div>
    <div class="stat"><b>11/15</b><span>convergen a R&nbsp;=&nbsp;550&nbsp;mm</span></div>
    <div class="stat"><b>p&nbsp;=&nbsp;0.09</b><span>efecto del escenario — todavía no significativo</span></div>
  </div>
</section>

<section>
  <div class="prose">
    <h2>El largo del log no es la duración del experimento</h2>
    <p>Los logs duran entre 112 y 347&nbsp;s, pero adentro hay tres cosas
    distintas. La base no registra los comandos que envía, así que el arranque
    de la congregación se deduce del primer <code>REQUEST_POSITION</code>: durante
    la caminata aleatoria el robot nunca pide su pose, y durante el
    <code>MEET</code> la pide continuamente.</p>
    <p>Entre que los robots se detienen y llega el comando hay un hueco —el
    operador mandándolo a mano— que no es experimento. Va de 0 a 46&nbsp;s, con
    mediana de 20. Comparar corridas por el largo del log mezcla eso con lo que
    se quiere medir.</p>
  </div>
  <figure>
    <img src="{img('fig_fases.png')}" alt="Barras apiladas del reparto de tiempo de las 15 corridas">
    <figcaption>Cada barra es una corrida. El número a la derecha es la duración
    de la congregación, que es lo único comparable entre escenarios.</figcaption>
  </figure>
  <div class="note prose">
    <p><strong>Qué arreglar para la próxima sesión:</strong> que la base escriba
    sus propios comandos al ConsoleLog (<code>0,Base,CMD|MEET|x|y</code>). Ahí las
    fases dejan de inferirse y el tiempo muerto se puede recortar de entrada.</p>
  </div>
</section>

<section>
  <div class="prose">
    <h2>La geometría salió de los videos</h2>
    <p>No hacía falta medir la arena a mano. Los markers ArUco de los robots
    aparecen en el video y su posición en milímetros está en el log, así que sirven
    para calibrar la cámara: ajustando una homografía por RANSAC sobre toda la
    corrida el residuo mediano queda en <strong>3 a 6&nbsp;mm</strong>. Con eso, las
    cajas de cartón se detectan por color sobre la mediana temporal del video —que
    borra a los robots y deja solo lo estático— y se convierten a milímetros.</p>
    <p>El hueco medido confirma la nomenclatura: los escenarios <code>D2</code>
    dan 2.1 y 2.2 diámetros de Atta, y los <code>D4</code> dan 4.5 y 4.6. También
    aparece que las columnas están <strong>desfasadas</strong>: la segunda fila cae
    en los huecos de la primera, no detrás de sus cajas.</p>
  </div>
  <div class="tw">
    <table>
      <caption class="visually-hidden"></caption>
      <thead><tr>
        <th scope="col">Escenario</th><th scope="col">Obstáculo</th>
        <th scope="col">Hueco</th><th scope="col">Cajas</th>
        <th scope="col">Congregación (s)</th><th scope="col">Convergen</th>
        <th scope="col">Compactación (d)</th>
      </tr></thead>
      <tbody>{''.join(rows_esc)}</tbody>
    </table>
  </div>
  <figure>
    <img src="{img('fig_trayectorias.png')}" alt="Trayectorias de los robots en los cinco escenarios">
    <figcaption>Solo el tramo posterior al comando <code>MEET</code>. Las líneas se
    cortan donde la cámara perdió el marker, para no dibujar rectas que atraviesen
    obstáculos y parezcan trayectorias reales.</figcaption>
  </figure>
</section>

<section>
  <div class="prose">
    <h2>Cómo se aprieta el enjambre</h2>
    <p>La métrica de compactación es la dispersión RMS de los robots respecto de
    su centroide, en diámetros de Atta. Alineada al comando, se ve la diferencia
    entre bajar limpio hasta un piso y quedarse oscilando porque alguien no cruzó.</p>
  </div>
  <figure>
    <img src="{img('fig_compactacion.png')}" alt="Curvas de compactación en el tiempo por escenario">
    <figcaption>Tres curvas por escenario, una por tipo de arranque.</figcaption>
  </figure>
</section>

<section>
  <div class="prose">
    <h2>El efecto va en la dirección esperada, pero falta n</h2>
    <p>Con obstáculos gruesos la congregación tarda el doble que sin obstáculos
    (45–59&nbsp;s contra 21). El problema es que con tres repeticiones por
    escenario el ANOVA da <strong>F(4,6)&nbsp;=&nbsp;3.34, p&nbsp;=&nbsp;0.092</strong>:
    la tendencia es clara a ojo pero no se sostiene estadísticamente. Juan Carlos
    pedía 5–6 repeticiones y por eso.</p>
    <p>Lo que sí quedó limpio es el control: el <em>tipo de arranque</em> no explica
    nada (p&nbsp;=&nbsp;0.61), así que las tres corridas de un escenario son
    repeticiones intercambiables y el diseño está bien planteado.</p>
  </div>
  <figure>
    <img src="{img('fig_resumen.png')}" alt="Boxplots de tiempo de congregación y ratio de ruta">
    <figcaption>Izquierda, el titular del paper. Derecha, cuánto de más caminó
    cada robot respecto de la línea recta a su destino.</figcaption>
  </figure>
  <div class="note prose">
    <p><strong>Sobre el criterio de convergencia.</strong> Juan Carlos propuso
    parar cuando el 90&nbsp;% esté dentro del radio, pero con cuatro robots
    ⌈0.9·4⌉&nbsp;=&nbsp;4: el 90&nbsp;% equivale a exigir los cuatro, y un solo
    robot atascado anula la corrida. Acá se usó R&nbsp;=&nbsp;550&nbsp;mm (el
    anillo comandado de 300 más dos diámetros), que da 11/15. Con 3&nbsp;de&nbsp;4
    a 500&nbsp;mm serían 12/15 y «cuántos quedaron atascados» pasaría a ser una
    métrica aparte.</p>
  </div>
</section>

<section>
  <div class="prose">
    <h2>Las 15 corridas</h2>
  </div>
  <div class="tw">
    <table>
      <thead><tr>
        <th scope="col">Escenario</th><th scope="col">Arranque</th>
        <th scope="col">Caminata (s)</th><th scope="col">Muerto (s)</th>
        <th scope="col">Congregación (s)</th><th scope="col">t 4/4 (s)</th>
        <th scope="col">Compactación (d)</th>
      </tr></thead>
      <tbody>{''.join(rows_run)}</tbody>
    </table>
  </div>
</section>

<section class="prose">
  <h2>Qué falta</h2>
  <ul>
    <li>Subir a 5–6 repeticiones por escenario. Es lo único que separa la
    tendencia actual de un resultado publicable.</li>
    <li>Loguear los comandos salientes de la base, para que las fases sean
    medidas y no inferidas.</li>
    <li>Decidir el criterio de convergencia con Juan Carlos antes de fijar los
    números del paper.</li>
    <li>Correr la campaña de simulación con 10 robots en la arena grande: el
    comando <code>MEET</code> ya está portado a la réplica de Webots y produce
    exactamente los mismos CSV, así que se procesa con este mismo código.</li>
  </ul>
</section>
</div>
'''

(SCRATCH / 'dataset_30-07.html').write_text(HTML)
print('ok', len(HTML) // 1024, 'KB')
