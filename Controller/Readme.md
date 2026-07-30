# Robot controller

C++ files for the robot microcontroller. Uses Arduino IDE for compiling and uploading.

## Versiones (leer antes de aceptar un update del IDE)

El firmware se compila hoy contra:

| Componente | Versión |
|---|---|
| ESP32 Arduino core | 3.3.11 |
| FastLED | 3.10.5 |
| Adafruit APDS9960 | 1.3.1 |
| SparkFun ICM-20948 | 1.3.2 |

Estas dependencias tienen requisitos que **no** se ven al compilar: el sketch
construye igual y el robot arranca igual, la falla aparece recién al moverlo.

### El PWM de los motores necesita el reloj APB (no meter otro periférico)

Los 4 timers low-speed del LEDC **comparten un solo mux de reloj**. Los motores
van a 1kHz/14-bit, o sea 16.384MHz de fuente, y eso solo sale de APB (80MHz).
Si otro periférico se queda con un reloj lento, los `ledcAttach()` de los
motores fallan y **las ruedas no reciben PWM: el robot no gira ni avanza**.

Pasó el 2026-07-29 con el servo frontal. ESP32Servo 3.0.9 → 3.2.1 bajó su ancho
de timer por defecto a 10 bits en el ESP32 clásico, y a 10 bits el servo pide
50Hz×2¹⁰ = 51.2kHz, que desde APB daría divisor 1562 y no entra en el registro
(~10 bits enteros) → el driver se iba al reloj lento y arrastraba a los motores.
Con el core ≤3.3.8 no se notaba: asignaba timers de forma estática (`canal/2`),
el primer motor pisaba el timer del servo y lo devolvía a APB (con el servo
corriendo callado a 1kHz en vez de 50Hz). El 3.3.11 asigna timers dinámicamente,
le respeta el timer al servo, y el choque salió a la luz.

**El servo se retiró** (no se usaba) y con él `ESP32Servo`. El `setup()` deja el
reloj explícito con `ledcSetClockSource(LEDC_USE_APB_CLK)` antes de cualquier
canal, así el reparto no se hereda de quien se attachee primero.

Si algún día se vuelve a montar un servo en el robot:

- Dale **16 bits** de ancho de timer (pide 3.28MHz → divisor 24 sobre APB), no
  los 10 por defecto.
- **No uses ESP32Servo 3.2.1**: su `attach()` pisa el ancho con el default
  (`ESP32Servo.cpp:97`) aunque el comentario de la librería diga que hay que
  llamar `setTimerWidth()` antes. Sale más barato `ledcAttach(pin, 50, 16)` +
  `ledcWrite(pin, us << 16 / 20000)`.

Para verificar sin destapar nada, mirá el arranque por serie:

```
[3] Motores PWM: 1000Hz 14-bit (max=16383) OK — real=1000Hz
```

`FALLO`, o un `real=` distinto de 1000Hz, es exactamente este problema.

### FastLED secuestra `radians()` — nunca la uses en este sketch

`FastLED.h` hace `#undef radians` (`fl/stl/undef.h`) y después
`using fl::radians` (`FastLED.h:226`). O sea que en el sketch `radians()` **no
es la macro de Arduino**, es esta plantilla (`fl/math/math.h:521`):

```cpp
template<typename T>
constexpr inline T radians(T deg) { return deg * static_cast<T>(0.017453292519943295); }
```

La constante se castea al **tipo del argumento**. Con un ángulo entero,
`static_cast<int>(0.01745…)` es **0**, así que `radians(180)` devuelve **0** y
cualquier giro calculado desde un entero se vuelve un no-op silencioso. Con
`float` funciona bien.

Ese fue el bug que rompió los giros (diagnosticado 2026-07-29, llegó con FastLED
3.10.5 el 2026-07-27). Síntoma característico, porque separa los caminos por
tipo: el robot **navegaba** hacia el objetivo (la nav usa floats) pero **no
esquivaba obstáculos** (`int avoidanceAngle`), el `RANDOM_WALK` no giraba y
`TURN|grados` no hacía nada en ningún robot. Es fácil confundirlo con un
problema de calibración por robot, y no lo es.

Usá **`DegToRad()`** y **`DegToArc()`** (definidas al principio de
`AttaBot.ino`). Toman `float` a propósito, para forzar la promoción en el call
site, y usan la constante literal sin depender de FastLED ni de `DEG_TO_RAD`.

La misma cabecera también reemplaza `abs`, `min`, `max`, `degrees` y `map`. Hoy
el firmware no usa `degrees()`, y `map()` sigue siendo la de Arduino (es función,
no macro), pero si aparece un cálculo raro con enteros, este es el primer lugar
donde mirar.

### ICM-20948: el DMP se activa editando la librería

`ICM_20948_USE_DMP` tiene que estar descomentado en
`src/util/ICM_20948_C.h` (línea ~34). **Un update de la librería pisa esa
edición.** Sin DMP no hay yaw, `imuAvailable` queda en `false` y los giros
caen al lazo abierto por encoders: salen cortos o largos y la navegación
acumula error.

## Sketches

| Sketch | Para qué |
|---|---|
| `AttaBot/AttaBot.ino` | Firmware normal |
| `HW_Test/HW_Test.ino` | Diagnóstico de hardware sin WiFi — ver `Docs/HW_Test.md` |
| `IR_Calibration/IR_Calibration.ino` | Calibración de los IR |
