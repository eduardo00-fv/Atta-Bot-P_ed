#!/usr/bin/env python3
"""
Genera .vscode/c_cpp_properties.json para que VS Code entienda los .ino.

    python3 vscode_setup.py

Los .ino no son C++ valido por su cuenta: no incluyen Arduino.h y los prototipos
los genera el build. Sin esto, la extension de C/C++ marca en rojo medio archivo
y no ofrece "ir a la definicion".

En vez de listar los includes a mano, se le piden a arduino-cli: compila solo la
base de datos (segundos, no la compilacion entera) y de ahi salen las rutas y los
defines REALES del core que se esta usando. Volver a correrlo despues de
actualizar el core o de instalar una libreria.
"""

import json
import os
import shlex
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
FQBN = 'esp32:esp32:esp32'

CLI_CANDIDATOS = [
    '/opt/arduino-ide/resources/app/lib/backend/resources/arduino-cli',
    os.path.expanduser('~/.local/bin/arduino-cli'),
    '/usr/bin/arduino-cli',
]


def find_cli():
    cli = next((c for c in CLI_CANDIDATOS if os.path.isfile(c)), None)
    if cli is None:
        sys.exit('✗ No encontré arduino-cli. Buscado en:\n  '
                 + '\n  '.join(CLI_CANDIDATOS))
    return cli


def compilation_database(cli):
    """Las flags reales con que se compila el sketch, sin compilarlo entero."""
    with tempfile.TemporaryDirectory() as tmp:
        r = subprocess.run(
            [cli, 'compile', '--fqbn', FQBN, '--only-compilation-database',
             '--build-path', tmp, HERE],
            capture_output=True, text=True)
        db = os.path.join(tmp, 'compile_commands.json')
        if not os.path.isfile(db):
            sys.exit('✗ arduino-cli no generó la base de datos.\n'
                     + (r.stderr or r.stdout))
        entradas = json.load(open(db))
    sketch = next((e for e in entradas if e['file'].endswith('.ino.cpp')), None)
    if sketch is None:
        sys.exit('✗ La base de datos no trae la entrada del sketch.')
    return sketch.get('arguments') or shlex.split(sketch['command'])


def main():
    arg = compilation_database(find_cli())
    incs = [a[2:] for a in arg if a.startswith('-I')]
    defs = [a[2:] for a in arg if a.startswith('-D')]
    compilador = arg[0]

    # El core vive fuera del repo, asi que las rutas son absolutas a proposito:
    # este archivo describe ESTA maquina y por eso .vscode/ no se versiona.
    core = next((i for i in incs if i.endswith('cores/esp32')), None)
    conf = {
        'version': 4,
        'configurations': [{
            'name': 'AttaBot ESP32',
            'compilerPath': compilador,
            'cStandard': 'c11',
            'cppStandard': 'c++17',
            'intelliSenseMode': 'gcc-x86',
            'includePath': incs + [os.path.join(HERE, '**')],
            'defines': defs,
            'forcedInclude': ([os.path.join(core, 'Arduino.h')] if core else []),
        }],
    }

    destino = os.path.join(REPO, '.vscode')
    os.makedirs(destino, exist_ok=True)
    ruta = os.path.join(destino, 'c_cpp_properties.json')
    with open(ruta, 'w', encoding='utf-8') as f:
        json.dump(conf, f, indent=4)
        f.write('\n')

    print(f'✓ {ruta}')
    print(f'  compilador: {compilador}')
    print(f'  {len(incs)} rutas de include · {len(defs)} defines')
    if core:
        print(f'  Arduino.h forzado desde {core}')
    print('\n  Recargá la ventana de VS Code para que tome los cambios:')
    print('  Ctrl+Shift+P → "Developer: Reload Window"')


if __name__ == '__main__':
    main()
