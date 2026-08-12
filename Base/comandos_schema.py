#!/usr/bin/env python3
"""
comandos_schema.py — firmas verificadas de los comandos, para validar y para
exponerlas como herramientas a un agente.

¿Por qué otro archivo si _ROBOT_CMDS ya es la fuente única?
--------------------------------------------------------------------------
_ROBOT_CMDS mapea verbo → string de AYUDA ('x|y[|radio]'). Eso sirve para que
un humano recuerde el orden de los argumentos, pero no es una gramática: hay
entradas ambiguas ('L|C|R|0o1' son dos argumentos, no cuatro) y llegó a haber
cuatro que NO coincidían con lo que el firmware lee. Un parser de esos strings
adivinaría, y adivinar mal acá significa mandar un comando que el robot
descarta EN SILENCIO — el modo de falla que ya costó tiempo de laboratorio.

Entonces las firmas de acá salen de leer los Handle*() de comandos.ino, no de
los strings de ayuda. Para que las dos tablas no se separen con el tiempo,
revisar_vocabulario() falla si alguien agrega un comando a _ROBOT_CMDS y se
olvida de describirlo acá. Esa comprobación es la que mantiene la propiedad de
fuente única; el string de ayuda sigue siendo para el humano.

Uso:
    python Base/comandos_schema.py            # reporte de consistencia
    python Base/comandos_schema.py --tools    # esquema JSON para el agente

    from comandos_schema import validar
    ok, error = validar('1.MOVE|500')
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any


# =============================================================================
# TIPOS
# =============================================================================

@dataclass(frozen=True)
class Arg:
    """Un argumento posicional de un comando.

    'requerido' es literal: el firmware lee arguments[i] sin validar, así que un
    argumento faltante llega como "" y termina en toInt() → 0. Un GT sin Y no
    falla, navega a y=0, que es la pared. Por eso conviene rechazar antes.
    """
    nombre: str
    tipo: str                                   # int | float | str | enum | ip
    requerido: bool = True
    opciones: tuple[str, ...] = ()
    ayuda: str = ''


@dataclass(frozen=True)
class Cmd:
    destino: str                                # 'robot' | 'base'
    args: tuple[Arg, ...] = ()
    desc: str = ''
    # Algunos comandos son en realidad varios: CONFIG|START|<ip> no tiene nada
    # que ver con CONFIG|ROBOTS. Se modelan como subcomandos por el primer arg.
    sub: dict[str, tuple[Arg, ...]] = field(default_factory=dict)


def _i(n, req=True, ayuda=''): return Arg(n, 'int', req, (), ayuda)
def _f(n, req=True, ayuda=''): return Arg(n, 'float', req, (), ayuda)
def _s(n, req=True, ayuda=''): return Arg(n, 'str', req, (), ayuda)
def _e(n, ops, req=True, ayuda=''): return Arg(n, 'enum', req, tuple(ops), ayuda)


NAV_KEYS = ('SEGMENT_DIST', 'ARRIVAL_THRESHOLD', 'PARKING_DIST', 'WHEEL_DIST',
            'REALIGN', 'GOAL_DEADBAND', 'ARENA', 'YAW_SCALE')

COLORES = ('rojo', 'verde', 'azul')


# =============================================================================
# FIRMAS — verificadas contra Controller/AttaBot/comandos.ino
# =============================================================================

SCHEMA: dict[str, Cmd] = {

    # --- movimiento abierto -------------------------------------------------
    'MOVE':   Cmd('robot', (_i('mm', ayuda='positivo adelante, negativo atrás'),),
                  'Avanza una distancia en línea recta.'),
    'TURN':   Cmd('robot', (_i('grados', ayuda='positivo antihorario'),),
                  'Gira en el sitio.'),
    'WAIT':   Cmd('robot', (_i('ms'),), 'Espera sin moverse.'),
    'RANDOMW': Cmd('robot', (_i('segmento_mm', req=False),),
                   'Camina al azar. Sin argumento usa el segmento por defecto.'),

    # --- navegación ---------------------------------------------------------
    'GT':     Cmd('robot', (_f('x'), _f('y'), _f('segmento_mm', req=False)),
                  'Navega a un punto global evadiendo obstáculos.'),
    'MEET':   Cmd('robot', (_f('x'), _f('y'), _f('radio', req=False)),
                  'Va a un punto de encuentro; el radio fuerza la distancia de parada.'),
    'ABORT_NAV': Cmd('robot', (), 'Cancela la navegación en curso.'),

    # --- enjambre -----------------------------------------------------------
    'CONGREGATION': Cmd('robot',
                        (_s('liderID'), _i('indiceSeguidor'),
                         _i('totalSeguidores', req=False)),
                        'Se suma a la congregación alrededor de un líder.'),
    'FORMATION': Cmd('robot',
                     (_e('figura', ('linea', 'cuna', 'circulo')), _s('liderID'),
                      _i('indiceSeguidor'), _i('totalSeguidores', req=False),
                      _f('eje_grados', req=False)),
                     'Toma su lugar en una figura alrededor del líder.'),
    'DISPERSE': Cmd('robot', (_f('separacion_mm', req=False,
                                 ayuda='por defecto 600'),),
                    'Se aleja hasta que el vecino más cercano esté a esa distancia.'),
    'CANCEL_CONGREGATION': Cmd('robot', (), 'Sale de congregación o formación.'),

    # --- percepción ---------------------------------------------------------
    'SEARCH_OBJECT': Cmd('robot', (_e('color', COLORES),),
                         'Patrulla buscando un objeto de ese color y avisa al encontrarlo.'),
    'COLOR_READ': Cmd('robot', (), 'Devuelve una lectura RGBC cruda para calibrar umbrales.'),

    # --- calibración y ajustes ----------------------------------------------
    'GETPPR': Cmd('robot', (), 'Reporta los pulsos por revolución vigentes.'),
    'SETPPR': Cmd('robot', (_f('valor'), _e('persistencia', ('TEMP', 'SAVE'),
                                            req=False)),
                  'Fija los pulsos por revolución. SAVE lo guarda en NVS.'),
    'PID':    Cmd('robot', (_f('kp'), _f('ki'), _f('kd'),
                            _e('guardar', ('SAVE',), req=False)),
                  'Ajusta el PID de los motores.'),
    # HandleKalmanPID lee TRES y en este orden: R, H, Q.
    'KFPID':  Cmd('robot', (_f('R'), _f('H'), _f('Q')),
                  'Ajusta el filtro de Kalman de la velocidad de rueda.'),
    'NAV_CONFIG': Cmd('robot', (_e('clave', NAV_KEYS), _f('valor'),
                                _e('guardar', ('SAVE',), req=False)),
                      'Cambia un parámetro de navegación.'),
    'SENSOR_MASK': Cmd('robot', (_e('sensor', ('L', 'C', 'R')),
                                 _e('enmascarar', ('0', '1'))),
                       'Ignora o vuelve a atender un sensor IR.'),
    # El firmware exige la 'C' delante del valor.
    'SENSOR_THRESHOLD': Cmd('robot', (_e('sensor', ('C',)), _i('umbral'),
                                      _e('guardar', ('SAVE',), req=False)),
                            'Umbral de proximidad del APDS9960 central (0..255).'),

    # --- estado y diagnóstico -----------------------------------------------
    'GET_STATUS': Cmd('robot', (), 'Reporta el estado de la máquina de estados.'),
    'GET_YAW':    Cmd('robot', (), 'Reporta el yaw de la IMU.'),
    'SELFTEST':   Cmd('robot', (_i('porcentaje_pwm', req=False,
                                   ayuda='por defecto 40'),),
                      'Prueba motores y sensores.'),
    'RESET':      Cmd('robot', (), 'Reinicia la máquina de estados.'),
    'CLEAR_EVASION': Cmd('robot', (), 'Limpia el estado de evasión activa.'),
    'RESET_EVASION': Cmd('robot', (), 'Reinicia el rastreador de evasiones.'),
    'SEND_COUNT_MESSAGE': Cmd('robot', (), 'Devuelve el contador de mensajes recibidos.'),
    'EKF_NAV':    Cmd('robot', (_e('activo', ('0', '1')),),
                      'Enciende o apaga el EKF como fuente de pose para navegar.'),

    'CONFIG': Cmd('robot', (), 'Configuración de red e identidad.', sub={
        'START':  (Arg('ip', 'ip'),),
        'SAVE':   (_s('nombre'),),
        'ROBOTS': (),
        'DEBUG':  (_e('activo', ('0', '1')),),
    }),

    # --- comandos de la Base ------------------------------------------------
    'BASE.STATUS':    Cmd('base', (), 'Resumen de robots conectados y sus poses.'),
    'BASE.CALIBRATE': Cmd('base', (_s('robotID'),), 'Rutina de calibración de un robot.'),
    'BASE.CONGREGATION': Cmd('base', (_s('liderID'), _f('espaciado_mm', req=False)),
                             'Orquesta la congregación: asigna slots a todos.'),
    'BASE.FORMATION': Cmd('base', (_e('figura', ('linea', 'cuna', 'circulo')),
                                   _s('liderID'), _f('espaciado_mm', req=False)),
                          'Orquesta una formación: valida límites y reparte slots.'),
    'BASE.OCCLUDE':   Cmd('base', (_f('segundos'),),
                          'Simula pérdida de marcador. Solo con --sim.'),
    'BASE.HELP':      Cmd('base', (_s('verbo', req=False),), 'Ayuda.'),
}


# =============================================================================
# VALIDACIÓN
# =============================================================================

def _valor_ok(arg: Arg, crudo: str) -> str | None:
    """Devuelve un mensaje de error, o None si el valor sirve."""
    if arg.tipo == 'enum':
        if crudo not in arg.opciones:
            return (f"'{arg.nombre}' debe ser uno de "
                    f"{'|'.join(arg.opciones)}, no '{crudo}'")
        return None
    if arg.tipo == 'int':
        try:
            int(crudo)
        except ValueError:
            return f"'{arg.nombre}' debe ser entero, no '{crudo}'"
        return None
    if arg.tipo == 'float':
        try:
            float(crudo)
        except ValueError:
            return f"'{arg.nombre}' debe ser numérico, no '{crudo}'"
        return None
    if arg.tipo == 'ip':
        partes = crudo.split('.')
        if len(partes) != 4 or not all(p.isdigit() and int(p) < 256 for p in partes):
            return f"'{arg.nombre}' debe ser una IPv4, no '{crudo}'"
        return None
    return None if crudo else f"'{arg.nombre}' no puede ir vacío"


def _validar_args(esperados: tuple[Arg, ...], dados: list[str],
                  etiqueta: str) -> str | None:
    minimos = sum(1 for a in esperados if a.requerido)
    if len(dados) < minimos:
        firma = '|'.join(a.nombre for a in esperados)
        return (f'{etiqueta} necesita al menos {minimos} argumento(s) '
                f'({firma}) y recibió {len(dados)}')
    if len(dados) > len(esperados):
        return (f'{etiqueta} acepta {len(esperados)} argumento(s) y '
                f'recibió {len(dados)}')
    for arg, crudo in zip(esperados, dados):
        error = _valor_ok(arg, crudo)
        if error:
            return f'{etiqueta}: {error}'
    return None


def validar(texto: str) -> tuple[bool, str | None]:
    """Valida un comando completo en la forma DESTINO.VERBO|arg|arg.

    No consulta el estado del sistema: no sabe si el robot '3' está conectado.
    Eso lo sigue resolviendo la Base. Acá solo se comprueba que el comando
    exista y que sus argumentos tengan la forma correcta, que es justo lo que
    el firmware NO comprueba antes de descartar en silencio.
    """
    texto = texto.strip()
    if '.' not in texto:
        return False, ("falta el destino: se escribe 'BASE.VERBO', "
                       "'<id>.VERBO' o 'BROADCAST.VERBO'")

    destino, resto = texto.split('.', 1)
    partes = [p.strip() for p in resto.split('|')]
    verbo, args = partes[0].upper(), [p for p in partes[1:] if p != '']

    clave = f'BASE.{verbo}' if destino.upper() == 'BASE' else verbo
    cmd = SCHEMA.get(clave)
    if cmd is None:
        ambito = 'de la Base' if destino.upper() == 'BASE' else 'del firmware'
        return False, f"'{verbo}' no es un comando {ambito}"

    if cmd.sub:
        if not args:
            return False, (f'{verbo} necesita un subcomando: '
                           f"{'|'.join(sorted(cmd.sub))}")
        sub = args[0].upper()
        if sub not in cmd.sub:
            return False, (f"'{sub}' no es un subcomando de {verbo}. "
                           f"Son: {'|'.join(sorted(cmd.sub))}")
        error = _validar_args(cmd.sub[sub], args[1:], f'{verbo}|{sub}')
        return (False, error) if error else (True, None)

    error = _validar_args(cmd.args, args, verbo)
    return (False, error) if error else (True, None)


# =============================================================================
# EXPOSICIÓN COMO HERRAMIENTAS DE AGENTE
# =============================================================================

_JSON_TIPO = {'int': 'integer', 'float': 'number', 'str': 'string',
              'enum': 'string', 'ip': 'string'}


def tool_schemas() -> list[dict[str, Any]]:
    """Las firmas como definiciones de herramienta en JSON Schema.

    El destino va como parámetro propio en vez de estar en el nombre para que
    el agente no tenga que construir strings: emite argumentos y quien llama
    arma 'DESTINO.VERBO|...' con formatear().
    """
    herramientas = []
    for clave, cmd in sorted(SCHEMA.items()):
        props: dict[str, Any] = {}
        requeridos: list[str] = []

        if cmd.destino == 'robot':
            props['destino'] = {
                'type': 'string',
                'description': "id del robot, o 'BROADCAST' para todos",
            }
            requeridos.append('destino')

        listas = [(cmd.args, '')] if not cmd.sub else \
                 [(a, f'{s}.') for s, a in sorted(cmd.sub.items())]
        if cmd.sub:
            props['subcomando'] = {'type': 'string',
                                   'enum': sorted(cmd.sub),
                                   'description': 'variante del comando'}
            requeridos.append('subcomando')

        vistos = set()
        for args, _prefijo in listas:
            for arg in args:
                if arg.nombre in vistos:
                    continue
                vistos.add(arg.nombre)
                esquema: dict[str, Any] = {'type': _JSON_TIPO[arg.tipo]}
                if arg.opciones:
                    esquema['enum'] = list(arg.opciones)
                if arg.ayuda:
                    esquema['description'] = arg.ayuda
                props[arg.nombre] = esquema
                if arg.requerido and not cmd.sub:
                    requeridos.append(arg.nombre)

        herramientas.append({
            'name': clave.replace('.', '_').lower(),
            'description': cmd.desc,
            'input_schema': {'type': 'object', 'properties': props,
                             'required': requeridos},
        })
    return herramientas


def formatear(clave: str, valores: dict[str, Any]) -> str:
    """Arma el string DESTINO.VERBO|... a partir de un dict de argumentos."""
    cmd = SCHEMA[clave]
    verbo = clave.split('.', 1)[1] if clave.startswith('BASE.') else clave
    destino = 'BASE' if cmd.destino == 'base' else str(valores['destino'])

    piezas: list[str] = []
    if cmd.sub:
        sub = valores['subcomando']
        piezas.append(sub)
        esperados = cmd.sub[sub]
    else:
        esperados = cmd.args
    for arg in esperados:
        if arg.nombre in valores and valores[arg.nombre] is not None:
            piezas.append(str(valores[arg.nombre]))
        elif arg.requerido:
            raise KeyError(f'falta el argumento requerido {arg.nombre}')
        else:
            break                                # posicional: no se puede saltar
    return f'{destino}.{verbo}' + ('|' + '|'.join(piezas) if piezas else '')


# =============================================================================
# CONSISTENCIA CONTRA EL VOCABULARIO DE LA BASE
# =============================================================================

def _ayuda_cuadra(cmd: Cmd, ayuda: str) -> bool:
    """¿El string de ayuda describe la misma cantidad de argumentos que la firma?

    Contar los '|' a secas no sirve: en la ayuda las enumeraciones se escriben
    con el mismo separador que los argumentos, así que 'L|C|R|0o1' parecen
    cuatro y son dos. Acá se recorren firma y ayuda en paralelo y, cuando el
    argumento es una enumeración y la ayuda está listando sus opciones, se las
    come todas de una.

    Limitación conocida: solo compara CANTIDAD. 'slot|x|y' para un comando que
    en realidad recibe líder, índice y total tiene la aridad correcta y pasa
    igual. Así fue como CONGREGATION estuvo mal descrito sin que nadie lo
    notara — para eso no hay atajo, hay que leer el handler.
    """
    piezas = [p.strip(' []') for p in ayuda.split('|') if p.strip(' []')]
    i = 0
    for arg in cmd.args:
        if i >= len(piezas):
            return not arg.requerido
        if arg.tipo == 'enum' and piezas[i] in arg.opciones:
            while i < len(piezas) and piezas[i] in arg.opciones:
                i += 1
        else:
            i += 1
    return i == len(piezas)


def revisar_vocabulario() -> list[str]:
    """Compara este esquema con _ROBOT_CMDS/_BASE_CMDS de la Base.

    Que esto pase es lo que permite tratar a _ROBOT_CMDS como fuente única: si
    alguien agrega un comando allá, acá falta y esta función lo grita. Se corre
    a mano o desde un test; no se importa en caliente para no arrastrar la Base
    entera (y su cv2) a un proceso que solo quiere validar strings.
    """
    from AttaBot_Base import _ROBOT_CMDS, _BASE_CMDS      # import diferido

    problemas = []
    mios_robot = {k for k, v in SCHEMA.items() if v.destino == 'robot'}
    mios_base = {k[5:] for k in SCHEMA if k.startswith('BASE.')}

    for falta in sorted(set(_ROBOT_CMDS) - mios_robot):
        problemas.append(f'FALTA en el esquema: {falta} (está en _ROBOT_CMDS)')
    for sobra in sorted(mios_robot - set(_ROBOT_CMDS)):
        problemas.append(f'SOBRA en el esquema: {sobra} (no está en _ROBOT_CMDS)')
    for falta in sorted(set(_BASE_CMDS) - mios_base):
        problemas.append(f'FALTA en el esquema: BASE.{falta}')
    for sobra in sorted(mios_base - set(_BASE_CMDS)):
        problemas.append(f'SOBRA en el esquema: BASE.{sobra}')

    # Aridad del string de ayuda contra la firma real. No es un error del
    # esquema: es un aviso de que la ayuda que lee el humano miente.
    for verbo in sorted(mios_robot & set(_ROBOT_CMDS)):
        cmd = SCHEMA[verbo]
        if cmd.sub:
            continue
        if not _ayuda_cuadra(cmd, _ROBOT_CMDS[verbo]):
            firma = '|'.join(a.nombre for a in cmd.args) or '(ninguno)'
            problemas.append(
                f"AYUDA DUDOSA: _ROBOT_CMDS['{verbo}'] = "
                f"'{_ROBOT_CMDS[verbo]}' no cuadra con la firma real ({firma})")
    return problemas


def _main() -> int:
    import sys
    if '--tools' in sys.argv:
        print(json.dumps(tool_schemas(), indent=2, ensure_ascii=False))
        return 0

    problemas = revisar_vocabulario()
    if not problemas:
        print(f'OK — {len(SCHEMA)} comandos, consistentes con el vocabulario.')
        return 0
    print(f'{len(problemas)} hallazgo(s):\n')
    for p in problemas:
        print(f'  · {p}')
    return 1


if __name__ == '__main__':
    raise SystemExit(_main())
