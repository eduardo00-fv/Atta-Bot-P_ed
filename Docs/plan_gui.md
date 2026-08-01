# Plan: poner la GUI al día

Estado al 2026-08-01. La GUI (`Base/AttaBot_GUI.py`, 344 líneas) se escribió
cuando el firmware tenía la mitad de los comandos que tiene hoy, y quedó atrás.

## Diagnóstico

### El problema de fondo: hay DOS despachadores

`Base.inputInstruction()` (consola) y `AttaBotGUI._dispatch()` parsean los dos el
mismo formato `robotId.instrucción`, cada uno por su cuenta. La copia de la GUI
**ya se quedó atrás**: la consola entiende `FORMATION.`, `CALIBRATE.` y
`OCCLUDE.`, y la GUI no. Nadie lo notó porque el que usa la GUI simplemente
teclea el comando crudo.

Mientras sigan siendo dos, cualquier verbo nuevo hay que acordarse de agregarlo
en ambos lados. Es el mismo patrón que rompió `OTA.py` cuando el sketch se
repartió en varios archivos.

**Primer paso, antes que cualquier botón:** extraer un solo
`Base.dispatch(robotId, instruccion, log=print)` y que consola y GUI lo llamen.
El parámetro `log` es lo único que cambia entre las dos: la consola imprime, la
GUI emite una señal.

### 13 comandos del firmware no tienen botón

De los 39 comandos, 10 son internos robot↔base y no van a la interfaz. De los 29
que un humano usaría, la GUI expone 16.

| Falta | Para qué | Prioridad |
|---|---|---|
| `MEET` | congregación sin líder — **el experimento del paper** | alta |
| `SENSOR_MASK` | enmascarar un IR defectuoso (Atta_1 tiene el derecho fantasma) | alta |
| `NAV_CONFIG` | arena, distancia de ruedas, escala de yaw, alcance IR, parking | alta |
| `SELFTEST` | prueba de motores e IMU en banco | alta |
| `DISPERSE` | dispersión de enjambre | media |
| `FORMATION` | línea, cuña, círculo | media |
| `SENSOR_THRESHOLD` | umbral del IR central | media |
| `EKF_NAV` | conmutar la navegación al EKF | media |
| `SEARCH_OBJECT`, `COLOR_READ` | búsqueda por color y su calibración | baja |
| `SETPPR`, `KFPID`, `POSITIONGT` | calibración fina, se usan poco | baja |

`SENSOR_MASK` y `NAV_CONFIG` son los que más se teclean hoy en el lab, así que
son los que más tiempo ahorran.

### No se ve el estado de los robots

La GUI muestra cámara, mapa de cobertura y log. El robot ya reporta cosas que no
se muestran en ninguna parte:

- `GET_STATUS` — batería, patrón IR, umbral central, máscaras, evasiones;
- `EKF_POSE` a 2Hz — con eso se puede graficar la deriva del EKF contra el ArUco
  en vivo, que es justo lo que hay que medir para decidir si el EKF sirve;
- `GET_YAW` — yaw y disponibilidad de la IMU.

Todo eso llega al log como texto y se pierde entre los mensajes de debug.

## Plan

### Fase 1 — un solo despachador

Extraer `Base.dispatch()` de `inputInstruction()`. La consola y la GUI lo llaman.
Verificación: `FORMATION.linea 1` funciona desde la GUI, que hoy no.

Sin cambio visible para el usuario, pero es lo que hace que las fases siguientes
no dupliquen trabajo.

### Fase 2 — los comandos de alta prioridad

Botones para `MEET`, `SENSOR_MASK`, `NAV_CONFIG` y `SELFTEST`.

`SENSOR_MASK` y `NAV_CONFIG` no son un botón sino un diálogo chico: el primero
elige sensor (L/R/C) y estado; el segundo, la clave a cambiar y su valor. Vale la
pena porque son los que se equivocan al teclear.

### Fase 3 — panel de estado por robot

Una tabla con una fila por robot: pose, batería, patrón IR, máscaras, evasiones,
deriva del EKF. Se refresca con `GET_STATUS` cada pocos segundos y con los
`EKF_POSE` que ya llegan.

Es lo que convierte la GUI de "una consola con cámara" en algo que dice de un
vistazo si el enjambre está sano — y hoy esa pregunta se contesta leyendo el log.

### Fase 4 — enjambre y experimentos

`DISPERSE`, `FORMATION` con su figura, y un botón que lance una corrida completa
del protocolo del paper (random walk → MEET) con los parámetros del escenario.

## Orden sugerido

Fase 1 primero, siempre. Después la 2, que es la que ahorra tiempo en la próxima
sesión de lab. La 3 y la 4 cuando el enjambre ya esté validado en hardware: hoy
el firmware del enjambre no se ha flasheado nunca, así que hacerle interfaz a
algo sin probar es adelantarse.

## Antes de empezar

La GUI necesita una pasada de verificación en hardware. Se probó por última vez
con dos robots y desde entonces cambiaron el firmware, el reparto de archivos y
la organización de `Base/`. Que importe sin errores ya está verificado; que
maneje robots reales, no.
