"""
Sube el firmware por OTA a los AttaBot en paralelo.

Uso:
    python3 OTA.py                 # a todas las IPs de la lista
    python3 OTA.py 101 104         # solo a .101 y .104 (atajo por último octeto)
    python3 OTA.py 192.168.1.101   # IP completa también vale
    python3 OTA.py --build 101     # compila primero y despues sube

Antes de subir verifica que el .bin sea más nuevo que el código fuente. Esa
guarda existe porque el 2026-07-29 el .bin del build tenía SEIS SEMANAS: subirlo
habría flasheado firmware de junio sin ninguno de los arreglos, y el robot habría
arrancado perfecto — con los bugs viejos adentro. Un OTA que sube un binario
viejo es indistinguible de uno que funciona hasta que se prueba el robot.
"""

import glob
import os
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))

# Ruta relativa al script: la absoluta hardcodeada rompía el repo en cualquier
# otra máquina. Se aceptan los dos layouts de build: el del IDE, que mete el .bin
# en un subdirectorio con el nombre del FQBN, y el de `arduino-cli --build-path`,
# que lo deja plano.
BIN_CANDIDATOS = [
    os.path.join(HERE, 'build', 'esp32.esp32.esp32', 'AttaBot.ino.bin'),
    os.path.join(HERE, 'build', 'AttaBot.ino.bin'),
]
# Fuentes que tienen que ser MÁS VIEJAS que el .bin para que subir tenga sentido.
# Se listan por glob y NO a mano: cuando el sketch se repartió en varios .ino
# (2026-08-01) la lista fija seguía nombrando solo AttaBot.ino y utils.h, así que
# editar comandos.ino y subir pasaba el chequeo con un binario viejo — que es
# exactamente el fallo que esta guarda existe para evitar.
SOURCES = sorted(glob.glob(os.path.join(HERE, '*.ino'))
                 + glob.glob(os.path.join(HERE, '*.h')))

# Puerto por defecto de ArduinoOTA y la clave de ArduinoOTA.setPassword().
PORT = 3232
PASS = "attabot1234"

# arduino-cli para compilar sin abrir el IDE. El primero es el que el propio IDE
# trae adentro, asi que en esta maquina no hay nada que instalar; el segundo es
# por si algun dia se instala suelto.
CLI_CANDIDATOS = [
    '/opt/arduino-ide/resources/app/lib/backend/resources/arduino-cli',
    os.path.expanduser('~/.local/bin/arduino-cli'),
    '/usr/bin/arduino-cli',
]
# El PartitionScheme NO es decorativo: partitions.csv le da al sketch un slot de
# 1.97MB en vez de los 1.31MB del layout por defecto. Sin declararlo, arduino-cli
# mide contra el slot equivocado y reporta 93% de uso cuando el real es 62%.
FQBN = 'esp32:esp32:esp32:PartitionScheme=min_spiffs'

IPS = [
    "192.168.1.101",
    "192.168.1.102",
    "192.168.1.103",
    "192.168.1.104",
    "192.168.1.105",
    "192.168.1.106",
    "192.168.1.107",
    "192.168.1.108",
]


def find_espota():
    """
    Ubica espota.py del core ESP32 instalado, tomando la versión más nueva.

    Antes estaba hardcodeado a 3.3.8 y al actualizar el core a 3.3.11 ese
    directorio desapareció, así que OTA.py fallaba entero. Se resuelve por glob
    para que sobreviva a la próxima actualización.
    """
    pattern = os.path.expanduser(
        '~/.arduino15/packages/esp32/hardware/esp32/*/tools/espota.py')
    found = sorted(glob.glob(pattern))
    if not found:
        sys.exit(f'✗ No se encontró espota.py.\n  Buscado en: {pattern}\n'
                 '  ¿Está instalado el core ESP32 en el IDE de Arduino?')
    return found[-1]


def find_bin():
    """El .bin más nuevo entre los layouts de build conocidos."""
    hay = [b for b in BIN_CANDIDATOS if os.path.isfile(b)]
    if not hay:
        sys.exit('✗ No existe el binario. Buscado en:\n  '
                 + '\n  '.join(BIN_CANDIDATOS)
                 + '\n  Compilá primero: python3 OTA.py --build')
    return max(hay, key=os.path.getmtime)


def check_binary_fresh(BIN):
    """Aborta si el .bin es más viejo que alguna fuente del sketch."""
    bin_time = os.path.getmtime(BIN)
    stale = [(s, os.path.getmtime(s)) for s in SOURCES
             if os.path.isfile(s) and os.path.getmtime(s) > bin_time]
    if stale:
        edad_h = (time.time() - bin_time) / 3600.0
        print(f'✗ El binario es más VIEJO que el código fuente.')
        print(f'  bin: {time.strftime("%Y-%m-%d %H:%M", time.localtime(bin_time))}'
              f'  ({edad_h:.1f}h de antigüedad)')
        for s, t in stale:
            print(f'  más nuevo: {os.path.basename(s)} '
                  f'{time.strftime("%Y-%m-%d %H:%M", time.localtime(t))}')
        print('\n  Recompilá (python3 OTA.py --build). Si subís así, el robot '
              'va a arrancar bien\n  pero con el firmware viejo adentro.')
        sys.exit(1)

    print(f'✓ Binario: {time.strftime("%Y-%m-%d %H:%M", time.localtime(bin_time))} '
          f'({os.path.getsize(BIN) / 1024:.0f} KB)')


def resolve_targets(args):
    """Convierte argumentos en IPs. Acepta el último octeto como atajo."""
    if not args:
        return IPS
    out = []
    for a in args:
        out.append(a if '.' in a else f'192.168.1.{a}')
    return out


def compilar():
    """Compila el sketch con el arduino-cli que trae el IDE, sin abrir el IDE.

    No hace falta instalar nada: el binario viene adentro de /opt/arduino-ide y
    el core ESP32 ya está en ~/.arduino15. Deja el .bin en build/, que es donde
    lo busca este mismo script.
    """
    cli = next((c for c in CLI_CANDIDATOS if os.path.isfile(c)), None)
    if cli is None:
        sys.exit('✗ No encontré arduino-cli. Buscado en:\n  '
                 + '\n  '.join(CLI_CANDIDATOS))
    print(f'Compilando con {cli}')
    r = subprocess.run([cli, 'compile', '--fqbn', FQBN,
                        '--build-path', os.path.join(HERE, 'build'), HERE])
    if r.returncode != 0:
        sys.exit('✗ La compilación falló; no se sube nada.')
    print()


def upload(ip, espota, BIN, results):
    print(f"🚀 Subiendo firmware a {ip} ...")
    try:
        subprocess.run(
            ["python3", espota, "-i", ip, "-p", str(PORT), "--auth", PASS,
             "--file", BIN],
            check=True, capture_output=True, text=True)
        print(f"✅ {ip}: actualizado correctamente")
        results[ip] = True
    except subprocess.CalledProcessError as e:
        # Sin esto el error de espota se perdía y todos los fallos se veían igual
        detalle = (e.stderr or e.stdout or '').strip().splitlines()
        pista = detalle[-1] if detalle else 'sin detalle'
        print(f"❌ {ip}: error en la actualización — {pista}")
        results[ip] = False


def main():
    args = sys.argv[1:]
    if '--build' in args:
        args.remove('--build')
        compilar()

    espota = find_espota()
    print(f'espota: {espota}')
    BIN = find_bin()
    check_binary_fresh(BIN)

    targets = resolve_targets(args)
    print(f'Objetivos: {", ".join(targets)}\n')

    results = {}
    threads = [threading.Thread(target=upload, args=(ip, espota, BIN, results))
               for ip in targets]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    ok = sum(1 for v in results.values() if v)
    print(f"\n🎉 {ok}/{len(targets)} actualizados")
    if ok < len(targets):
        fallidos = [ip for ip, v in results.items() if not v]
        print(f'   Sin actualizar: {", ".join(fallidos)}')
        print('   Un robot apagado o en otra red no responde al OTA.')
        sys.exit(1)


if __name__ == '__main__':
    main()
