#!/usr/bin/env python3
"""
recortar_video.py — Recorta los videos de la base por el RELOJ DE LOS LOGS.

El problema que resuelve
------------------------
Los .avi de la base se escriben con un fps FIJO (el que declara la cámara, 20)
pero la cámara entrega menos cuadros de los que promete: en las corridas del
13-08 el ritmo real fue 19.1-19.5 fps. El archivo entonces se reproduce ~3% más
rápido que la realidad, y el reloj del reproductor NO es el de los logs: a los
400 segundos el desfase ya son 12 s, de sobra para cortar fuera de la maniobra.

Además el encabezado AVI queda sin duración ni conteo de cuadros (`duration=0`,
`nb_frames=N/A`), así que ni ffprobe ni el reproductor saben cuánto dura.

La referencia exacta es el sello `Time: NN.N s` que la base imprime sobre cada
cuadro, y ese sello es el mismo reloj de ConsoleLogs/PositionLogs. Esta
herramienta trabaja SIEMPRE en ese reloj, y para traducirlo a número de cuadro
usa Base/Logs/Time_Log_*.csv, que lleva una fila por cuadro procesado en el
mismo orden en que el hilo del video los graba: **la fila k es el cuadro k**.

Ese mapa importa porque el ritmo tampoco es constante: en la corrida 13-22 el
promedio global da 19.12 fps pero durante la maniobra la cámara iba a 19.46, y
un modelo lineal se corría medio segundo. Con el mapa el corte cae en el cuadro
pedido, y `--verificar` lo prueba leyendo el sello del clip resultante.

Al recortar re-etiqueta la salida al ritmo medido DENTRO de la ventana, así el
clip dura lo que dice durar y su reloj es el del log.

Uso
---
    # panorama de todos los videos, con la sesión que le toca a cada uno
    python3 Base/herramientas/recortar_video.py --listar

    # qué pasó y cuándo en la sesión de ese video (para elegir el corte)
    python3 Base/herramientas/recortar_video.py prueba_cuna.avi --eventos

    # recortar por tiempo de log (segundos, o mm:ss)
    python3 Base/herramientas/recortar_video.py prueba_cuna.avi --desde 338 --hasta 411

    # recortar alrededor de un comando, con margen
    python3 Base/herramientas/recortar_video.py prueba_cuna.avi \
        --evento FORMATION --antes 10 --despues 65

    # solo la vista de cámara, al doble de velocidad, y verificar el sello
    python3 Base/herramientas/recortar_video.py prueba_linea.avi --desde 36 --hasta 130 \
        --mitad arriba --velocidad 2 --verificar

Opciones
--------
  --desde / --hasta T   inicio y fin en el reloj del log (s, o mm:ss)
  --evento NOMBRE       ancla el corte en un CMD del ConsoleLog (FORMATION, ...)
  --antes / --despues S margen alrededor del evento (default 5 / 60)
  -o, --salida ARCH     nombre de salida (default: auto, en Videos/recortes/)
  --mitad arriba|abajo|ambas   el .avi apila cámara (arriba) y cobertura (abajo)
  --velocidad N         acelera el clip N veces (no descarta cuadros)
  --copy                sin recodificar: instantáneo, pero corta en el keyframe
                        más cercano y CONSERVA el desfase de fps
  --verificar           guarda un PNG con el sello del primer y último cuadro
  --sesion DD-MM_HH-MM  forzar el log (si el emparejamiento automático falla)
  --crf N               calidad x264 (default 20; menor = mejor y más pesado)
"""
import argparse
import bisect
import csv
import glob
import json
import os
import re
import subprocess
import sys

# La herramienta vive en Base/herramientas/, así que Base/ — donde están
# Videos/, ConsoleLogs/ y PositionLogs/ — es la de arriba.
BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DIR_VIDEOS = os.path.join(BASE, 'Videos')
DIR_SALIDA = os.path.join(DIR_VIDEOS, 'recortes')
DIR_CONSOLE = os.path.join(BASE, 'ConsoleLogs')
DIR_POSICION = os.path.join(BASE, 'PositionLogs')
DIR_TIEMPOS = os.path.join(BASE, 'Logs')
# Contar cuadros obliga a decodificar el archivo entero: ~4 s por cada 200 MB.
# Se cachea contra (tamaño, mtime) para no pagarlo en cada corte.
CACHE = os.path.join(DIR_VIDEOS, '.recortar_cache.json')
# Sello de tiempo: cv2.putText(frame, 'Time: %.1f s', (2, 26)) sobre la MITAD
# DE ARRIBA del cuadro apilado (ver videoWriter en AttaBot_Base.py).
SELLO = (0, 0, 320, 44)   # x, y, ancho, alto


def morir(msg):
    sys.exit(f'✗ {msg}')


def correr(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


# ── Video ────────────────────────────────────────────────────────────────────

def _cache_leer():
    try:
        with open(CACHE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def _cache_escribir(d):
    try:
        with open(CACHE, 'w') as f:
            json.dump(d, f, indent=1)
    except OSError:
        pass          # el cache es un lujo, no una dependencia


def sondear(video):
    """(cuadros_reales, fps_declarado, ancho, alto) — cuadros por decodificación.

    `nb_frames` del encabezado no sirve: los AVI de la base salen con el campo
    vacío porque VideoWriter no reescribe el índice al cerrar.
    """
    st = os.stat(video)
    clave = f'{os.path.basename(video)}|{st.st_size}|{int(st.st_mtime)}'
    cache = _cache_leer()

    r = correr(['ffprobe', '-v', 'error', '-select_streams', 'v:0',
                '-show_entries', 'stream=r_frame_rate,width,height',
                '-of', 'default=noprint_wrappers=1:nokey=1', video])
    if r.returncode != 0:
        morir(f'ffprobe falló sobre {os.path.basename(video)}:\n{r.stderr.strip()}')
    campos = r.stdout.split()
    ancho, alto = int(campos[0]), int(campos[1])
    num, den = campos[2].split('/')
    fps_dec = float(num) / float(den)

    if clave in cache:
        return cache[clave], fps_dec, ancho, alto

    r = correr(['ffprobe', '-v', 'error', '-count_frames', '-select_streams', 'v:0',
                '-show_entries', 'stream=nb_read_frames', '-of', 'csv=p=0', video])
    try:
        cuadros = int(r.stdout.strip().splitlines()[-1])
    except (ValueError, IndexError):
        morir(f'no pude contar los cuadros de {os.path.basename(video)}')
    cache[clave] = cuadros
    _cache_escribir(cache)
    return cuadros, fps_dec, ancho, alto


# ── Logs ─────────────────────────────────────────────────────────────────────

RE_SESION = re.compile(r'Console_Log_(?:SIM_)?(\d{2}-\d{2}_\d{2}-\d{2})_Robots_(\d+)\.csv')


def _ultimo_t(ruta):
    """Último sello de tiempo de un CSV de log, o None."""
    try:
        with open(ruta, errors='replace') as f:
            ultima = None
            for fila in csv.reader(f):
                if fila and fila[0] != 'time':
                    ultima = fila[0]
        return float(ultima)
    except (OSError, ValueError, TypeError):
        return None


def sesiones():
    """[(etiqueta, ruta_console, duración_s)] de todas las sesiones.

    La duración sale del PositionLog, no del ConsoleLog. El ConsoleLog solo
    tiene filas cuando alguien habla: la sesión 13-08_13-34 termina ahí en
    279 s cuando la corrida siguió hasta 621 s, y con esa duración el ritmo
    calculado daba 43 fps — el doble de lo posible. El PositionLog en cambio
    escribe por cada detección, así que su última fila es el final real.
    """
    out = []
    for ruta in sorted(glob.glob(os.path.join(DIR_CONSOLE, '*.csv'))):
        m = RE_SESION.match(os.path.basename(ruta))
        if not m:
            continue
        pos = os.path.join(DIR_POSICION,
                           os.path.basename(ruta).replace('Console_Log_', 'Position_Log_'))
        dur = _ultimo_t(pos) if os.path.exists(pos) else None
        if dur is None:
            dur = _ultimo_t(ruta)
        if dur is None or dur <= 0:
            continue
        out.append((m.group(1), ruta, dur))
    return out


def emparejar(video, forzada=None):
    """Sesión que corresponde al video.

    Los videos auto-nombrados (Video_13-08_13-10_Robots_4.avi) traen la
    etiqueta en el nombre. Los renombrados a mano (prueba_cuna.avi) no, y ahí
    se usa el mtime: el archivo se cierra al terminar la corrida, así que
    mtime ≈ inicio_de_sesión + duración. Se exige además que el ritmo que
    resulta (cuadros/duración) sea creíble para esta cámara — si no, el
    emparejamiento se declara dudoso en vez de mentir.
    """
    todas = sesiones()
    if forzada:
        cand = [s for s in todas if s[0] == forzada]
        if not cand:
            morir(f'no hay ConsoleLog para la sesión {forzada}')
        return cand[0], 'forzada'

    nombre = os.path.basename(video)
    m = re.search(r'(\d{2}-\d{2}_\d{2}-\d{2})', nombre)
    if m:
        cand = [s for s in todas if s[0] == m.group(1)]
        if cand:
            return cand[0], 'por nombre'

    mtime = os.stat(video).st_mtime
    mejor, mejor_err = None, None
    for etiqueta, ruta, dur in todas:
        dia, hora = etiqueta.split('_')
        d, mes = dia.split('-')
        hh, mm = hora.split('-')
        import datetime as dt
        anio = dt.datetime.fromtimestamp(mtime).year
        try:
            inicio = dt.datetime(anio, int(mes), int(d), int(hh), int(mm)).timestamp()
        except ValueError:
            continue
        err = abs((inicio + dur) - mtime)
        if mejor_err is None or err < mejor_err:
            mejor, mejor_err = (etiqueta, ruta, dur), err
    if mejor is None or mejor_err > 180:
        morir(f'no pude emparejar {nombre} con ninguna sesión '
              f'(usá --sesion DD-MM_HH-MM)')
    return mejor, f'por mtime (±{mejor_err:.0f} s)'


def mapa_cuadros(ruta_console):
    """Tiempo de log de CADA cuadro grabado, indexado por número de cuadro.

    Base/Logs/Time_Log_*.csv lleva una fila por cuadro PROCESADO, y el hilo del
    video graba exactamente esos cuadros en ese orden: la fila k es el cuadro k.
    Es el mapa exacto, y hace falta porque el ritmo NO es constante — en la
    corrida 13-22 el promedio global da 19.12 fps pero durante la maniobra la
    cámara iba a 19.46, y un modelo lineal se corría medio segundo.

    Devuelve None si no hay Time_Log (sesiones viejas): ahí se cae al promedio.
    """
    ruta = os.path.join(DIR_TIEMPOS,
                        os.path.basename(ruta_console).replace('Console_Log_', 'Time_Log_'))
    if not os.path.exists(ruta):
        return None
    try:
        with open(ruta, errors='replace') as f:
            t = [float(fila['time']) for fila in csv.DictReader(f)]
    except (OSError, ValueError, KeyError):
        return None
    return t or None


def cuadro_de(mapa, t):
    """Índice del cuadro cuyo sello está más cerca de t."""
    i = bisect.bisect_left(mapa, t)
    if i <= 0:
        return 0
    if i >= len(mapa):
        return len(mapa) - 1
    return i if (mapa[i] - t) < (t - mapa[i - 1]) else i - 1


def eventos(ruta_console):
    """[(t, robot, mensaje)] de los comandos de la sesión, sin repetir."""
    vistos, out = set(), []
    with open(ruta_console, errors='replace') as f:
        for fila in csv.reader(f):
            if len(fila) < 4 or fila[0] == 'time':
                continue
            try:
                t = float(fila[0])
            except ValueError:
                continue
            msg = fila[3]
            if not msg.startswith('CMD|'):
                continue
            # El robot va en la clave: en un FORMATION el líder y el primer
            # seguidor reciben el mismo texto (idx 0), y sin el destinatario uno
            # de los dos desaparecía de la lista.
            clave = (round(t, 1), fila[2], msg)
            if clave in vistos:
                continue
            vistos.add(clave)
            out.append((t, fila[2], msg))
    return out


# ── Recorte ──────────────────────────────────────────────────────────────────

def a_segundos(txt):
    """'95', '95.5' o '1:35' → segundos."""
    txt = str(txt).strip()
    if ':' in txt:
        partes = [float(p) for p in txt.split(':')]
        s = 0.0
        for p in partes:
            s = s * 60 + p
        return s
    return float(txt)


def ventana(t0, t1, mapa, cuadros, fps_dec, dur_log):
    """(cuadro0, cuadro1, t_real0, t_real1, fps_local) para la ventana pedida.

    Con Time_Log el corte es exacto: se busca el cuadro cuyo sello está más
    cerca del instante pedido. Sin él queda el promedio, que es lo que había.
    """
    if mapa:
        # El Time_Log suele tener unas filas de más: son los cuadros que se
        # procesaron pero que el writer no alcanzó a volcar antes de cerrar el
        # .avi (10 de más en 13-22, 5 en 13-47). Esos cuadros no existen.
        util = mapa[:cuadros]
        k0, k1 = cuadro_de(util, t0), cuadro_de(util, t1)
        k1 = max(k1, k0 + 1)
        # Ritmo LOCAL de la ventana, no el global: es el que hace que el reloj
        # del clip coincida con el del log dentro del recorte.
        span = util[k1] - util[k0]
        fps_local = (k1 - k0) / span if span > 0 else cuadros / dur_log
        return k0, k1, util[k0], util[k1], fps_local
    fps_real = cuadros / dur_log
    k0, k1 = int(round(t0 * fps_real)), int(round(t1 * fps_real))
    k1 = min(max(k1, k0 + 1), cuadros - 1)
    return k0, k1, t0, t1, fps_real


def recortar(video, k0, k1, fps_local, salida, fps_dec, alto,
             mitad='ambas', velocidad=1.0, copiar=False, crf=20):
    """Corta los cuadros [k0, k1] y re-etiqueta la salida al ritmo medido."""
    # El contenedor coloca el cuadro n en n/fps_declarado, sin importar cuándo
    # se capturó de verdad: esa es la única conversión cuadro→tiempo que ffmpeg
    # entiende sobre este archivo.
    ct0 = k0 / fps_dec
    dur_ct = (k1 - k0 + 1) / fps_dec

    os.makedirs(os.path.dirname(salida) or '.', exist_ok=True)
    # `-ss` y `-t` van ANTES de `-i`, o sea sobre la ENTRADA. Como opciones de
    # salida se aplicarían después del setpts, midiendo la duración en la base
    # de tiempo nueva: un corte de 94 s salía 2.4 s corto porque los 91.4 s de
    # contenedor se interpretaban como 91.4 s ya re-etiquetados.
    cmd = ['ffmpeg', '-y', '-v', 'error', '-stats',
           '-ss', f'{ct0:.3f}', '-t', f'{dur_ct:.3f}', '-i', video, '-an']

    if copiar:
        cmd += ['-c:v', 'copy']
    else:
        filtros = []
        if mitad == 'arriba':
            filtros.append(f'crop=iw:{alto // 2}:0:0')
        elif mitad == 'abajo':
            filtros.append(f'crop=iw:{alto // 2}:0:{alto // 2}')
        # x264 con yuv420p exige dimensiones pares.
        filtros.append('scale=trunc(iw/2)*2:trunc(ih/2)*2')
        fps_out = fps_local * velocidad
        # Reescribe el PTS por índice de cuadro: descarta el timebase mentiroso
        # del AVI y deja la salida con el ritmo real medido.
        filtros.append(f'setpts=N/({fps_out:.6f}*TB)')
        cmd += ['-vf', ','.join(filtros), '-r', f'{fps_out:.6f}',
                '-c:v', 'libx264', '-crf', str(crf), '-preset', 'medium',
                '-pix_fmt', 'yuv420p', '-movflags', '+faststart']
    cmd.append(salida)

    r = subprocess.run(cmd)
    if r.returncode != 0:
        morir(f'ffmpeg falló ({" ".join(cmd[:6])} ...)')


def verificar(video, salida, t0, t1, alto, mitad):
    """PNG con el sello del primer y último cuadro del recorte.

    Es la única comprobación honesta: el sello lo escribió la base con el mismo
    reloj de los logs, así que si dice t0 y t1 el corte quedó donde se pidió.
    """
    if mitad == 'abajo':
        print('  (sin verificación: el sello vive en la mitad de arriba)')
        return None
    x, y, w, h = SELLO
    png = os.path.splitext(salida)[0] + '_sello.png'
    r = correr(['ffprobe', '-v', 'error', '-count_frames', '-select_streams', 'v:0',
                '-show_entries', 'stream=nb_read_frames', '-of', 'csv=p=0', salida])
    try:
        n = int(r.stdout.strip().splitlines()[-1])
    except (ValueError, IndexError):
        n = 0
    if n < 2:
        print('  (sin verificación: el recorte no tiene cuadros suficientes)')
        return None
    filtro = (f"[0:v]select='eq(n\\,0)+eq(n\\,{n - 1})',crop={w}:{h}:{x}:{y},"
              f"scale={w * 2}:{h * 2}[c];[c]tile=1x2[out]")
    r = correr(['ffmpeg', '-y', '-v', 'error', '-i', salida,
                '-filter_complex', filtro, '-map', '[out]', '-frames:v', '1', png])
    if r.returncode != 0 or not os.path.exists(png):
        print('  (no pude armar el PNG de verificación)')
        return None
    print(f'  sello esperado: arriba ≈ {t0:.1f} s · abajo ≈ {t1:.1f} s')
    print(f'  → {os.path.relpath(png, BASE)}')
    return png


# ── Modos de consulta ────────────────────────────────────────────────────────

def listar():
    videos = sorted(glob.glob(os.path.join(DIR_VIDEOS, '*.avi')))
    if not videos:
        morir(f'no hay .avi en {DIR_VIDEOS}')
    print(f'{"video":<34}{"cuadros":>8}{"fps":>7}{"dur.log":>9}  {"":<3} sesión')
    print('─' * 80)
    for v in videos:
        cuadros, fps_dec, _, _ = sondear(v)
        try:
            (etiqueta, ruta, dur), como = emparejar(v)
            fps_real = cuadros / dur
            # '·' = hay Time_Log, el corte va a ser exacto; '~' = solo promedio.
            marca = ('·' if mapa_cuadros(ruta) else '~')
            marca += '' if abs(fps_real - fps_dec) < 0.05 * fps_dec else ' ⚠'
            evs = [e for e in eventos(ruta) if 'FORMATION' in e[2]
                   or 'CONGREGATION' in e[2] or 'DISPERSE' in e[2]]
            extra = ''
            if evs:
                t, _, msg = evs[0]
                extra = f'  ← {msg.split("|")[1]} {msg.split("|")[2]} @ {t:.0f}s'
            print(f'{os.path.basename(v):<34}{cuadros:>8}{fps_real:>7.2f}'
                  f'{dur:>8.0f}s  {marca:<3} {etiqueta} ({como}){extra}')
        except SystemExit:
            print(f'{os.path.basename(v):<34}{cuadros:>8}{fps_dec:>7.2f}'
                  f'{"?":>9}  {"":<3} sin sesión')
    print('\n  ·  cuadro↔tiempo exacto (hay Time_Log)     ~  solo el promedio'
          '\n  ⚠  el fps declarado miente >5%: el reloj del reproductor no sirve')


def mostrar_eventos(video, sesion_forzada):
    (etiqueta, ruta, dur), como = emparejar(video, sesion_forzada)
    cuadros, fps_dec, _, _ = sondear(video)
    fps_real = cuadros / dur
    print(f'{os.path.basename(video)} → sesión {etiqueta} ({como})')
    print(f'  {cuadros} cuadros · {dur:.1f} s de log · {fps_real:.2f} fps reales '
          f'(declarados {fps_dec:.2f}, {100 * (fps_dec / fps_real - 1):+.1f}%)')
    mapa = mapa_cuadros(ruta)
    if mapa:
        perdidos = len(mapa) - cuadros
        print(f'  Time_Log: {len(mapa)} cuadros procesados → mapeo exacto'
              + (f' ({perdidos} no llegaron al .avi al cerrar)' if perdidos > 0 else ''))
    else:
        print('  sin Time_Log: el corte usa el ritmo promedio (±0.5 s)')
    evs = eventos(ruta)
    if not evs:
        print('  (la sesión no registró comandos)')
        return
    print(f'\n  {"t_log":>8}  comando')
    print('  ' + '─' * 60)
    anterior = None
    for t, robot, msg in evs:
        if anterior is not None and t - anterior > 30:
            print(f'  {"":>8}  ··· {t - anterior:.0f} s sin comandos ···')
        print(f'  {t:>8.1f}  {robot:<8} {msg[4:]}')
        anterior = t


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        description='Recorta los videos de la base usando el reloj de los logs.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split('Uso\n---\n')[1] if 'Uso\n---\n' in __doc__ else None)
    p.add_argument('video', nargs='?', help='archivo en Base/Videos (o ruta)')
    p.add_argument('--listar', action='store_true', help='panorama de todos los videos')
    p.add_argument('--eventos', action='store_true', help='comandos de la sesión del video')
    p.add_argument('--desde', help='inicio en el reloj del log (s o mm:ss)')
    p.add_argument('--hasta', help='fin en el reloj del log (s o mm:ss)')
    p.add_argument('--evento', help='anclar el corte en un CMD (ej. FORMATION)')
    p.add_argument('--antes', type=float, default=5.0, help='margen previo (default 5 s)')
    p.add_argument('--despues', type=float, default=60.0, help='margen posterior (default 60 s)')
    p.add_argument('-o', '--salida', help='archivo de salida')
    p.add_argument('--mitad', choices=('arriba', 'abajo', 'ambas'), default='ambas',
                   help='qué mitad del cuadro apilado conservar')
    p.add_argument('--velocidad', type=float, default=1.0, help='acelerar N veces')
    p.add_argument('--copy', action='store_true', help='sin recodificar (rápido, impreciso)')
    p.add_argument('--verificar', action='store_true', help='PNG con el sello de tiempo')
    p.add_argument('--sesion', help='forzar sesión DD-MM_HH-MM')
    p.add_argument('--crf', type=int, default=20, help='calidad x264 (default 20)')
    a = p.parse_args()

    if a.listar:
        listar()
        return

    if not a.video:
        p.print_help()
        return

    video = a.video
    if not os.path.exists(video):
        video = os.path.join(DIR_VIDEOS, a.video)
    if not os.path.exists(video):
        morir(f'no existe {a.video} (ni en {DIR_VIDEOS})')

    if a.eventos or (a.desde is None and a.evento is None):
        mostrar_eventos(video, a.sesion)
        if not a.eventos:
            print('\n  Elegí la ventana y volvé a correr con --desde/--hasta, '
                  'o usá --evento FORMATION.')
        return

    (etiqueta, ruta, dur), como = emparejar(video, a.sesion)
    cuadros, fps_dec, _, alto = sondear(video)

    if a.evento:
        coincide = [e for e in eventos(ruta) if a.evento.upper() in e[2].upper()]
        if not coincide:
            morir(f'la sesión {etiqueta} no tiene ningún CMD con "{a.evento}"')
        t_ev = coincide[0][0]
        t0 = max(0.0, t_ev - a.antes)
        t1 = min(dur, t_ev + a.despues)
        print(f'evento "{coincide[0][2][4:]}" en t={t_ev:.1f} s')
    else:
        t0 = a_segundos(a.desde)
        t1 = a_segundos(a.hasta) if a.hasta else dur

    t0, t1 = max(0.0, t0), min(dur, t1)
    if t1 <= t0:
        morir(f'ventana vacía: desde {t0:.1f} hasta {t1:.1f}')

    salida = a.salida
    if not salida:
        raiz = os.path.splitext(os.path.basename(video))[0]
        salida = os.path.join(DIR_SALIDA, f'{raiz}_{t0:.0f}-{t1:.0f}s.mp4')
    elif not os.path.isabs(salida) and os.path.dirname(salida) == '':
        salida = os.path.join(DIR_SALIDA, salida)

    mapa = mapa_cuadros(ruta)
    k0, k1, r0, r1, fps_local = ventana(t0, t1, mapa, cuadros, fps_dec, dur)

    print(f'{os.path.basename(video)} → sesión {etiqueta} ({como})')
    print(f'  ventana de log {r0:.1f}-{r1:.1f} s ({r1 - r0:.1f} s) '
          f'= cuadros {k0}-{k1}' + ('' if mapa else '  (aproximado)'))
    # Sin la corrección, el reproductor pondría el mismo instante acá:
    ct0 = k0 / fps_dec
    print(f'  {fps_local:.2f} fps reales vs {fps_dec:.2f} declarados → '
          f'el reloj del reproductor marcaba {ct0:.1f} s donde el log dice {r0:.1f} s')
    if a.velocidad != 1.0:
        print(f'  velocidad ×{a.velocidad:g}')
    if a.copy:
        print('  ⚠ --copy: corta en el keyframe más cercano y deja el desfase de fps')

    recortar(video, k0, k1, fps_local, salida, fps_dec, alto,
             mitad=a.mitad, velocidad=a.velocidad, copiar=a.copy, crf=a.crf)
    t0, t1 = r0, r1

    mb = os.path.getsize(salida) / 1e6
    print(f'✓ {os.path.relpath(salida, BASE)}  ({mb:.1f} MB)')
    if a.verificar and not a.copy:
        verificar(video, salida, t0, t1, alto, a.mitad)


if __name__ == '__main__':
    main()
