# Campaña de simulación — 31 de julio de 2026

Mismo protocolo que las pruebas físicas del 30-07, en Webots: caminata aleatoria
de 60 s y después un comando `MEET` que congrega al enjambre sobre un punto fijo.
**5 escenarios × 3 repeticiones = 15 corridas, todas convergen.**

La diferencia de escala con el laboratorio es deliberada: **10 robots en una
arena de 3800×2800 mm**, contra 4 robots en 2400×1750. Los obstáculos y las
aperturas miden lo mismo en las dos (el robot no cambia de tamaño), así que lo
que cambia es la congestión, no la geometría del paso.

## Estructura

| | |
|---|---|
| `PositionLogs/<Escenario>_r<N>.csv` | pose de cada robot por fotograma — mismo formato que el lab |
| `ConsoleLogs/<Escenario>_r<N>.csv` | mensajes de los robots |
| `Videos/<Escenario>_r<N>.avi` | grabación de la corrida |
| `manifiesto.csv` | una fila por corrida: escenario, repetición, punto de encuentro y contacto |
| `sim_runs.csv` | métricas agregadas, una fila por corrida |
| `sim_robots.csv` | métricas por robot por corrida (10 × 15 = 150 filas) |
| `geometria_escenarios.json` | posición y tamaño exactos de cada obstáculo |

No hay `Logs/` (los tiempos de procesamiento) porque la base en modo simulación
no los escribe.

## Escenarios

`SinObs` es el control. Los otros cuatro cruzan el eje de avance con una barrera
de dos columnas desfasadas —la segunda cae en los huecos de la primera, no
detrás de sus cajas— y se nombran igual que en el laboratorio:

- **BosqueD** = obstáculo delgado (80×80 mm) · **BosqueG** = grueso (80×160 mm,
  el lado largo cruzando la barrera)
- **D2** = apertura de 2 diámetros de Atta · **D4** = 4 diámetros

El pasaje efectivo medido —el cuello de botella de la ruta más holgada, no el
mínimo entre pares— es 1.81 d y 2.00 d en los D2, y 3.81 d y 3.92 d en los D4.

## Columnas de `sim_runs.csv`

| columna | qué es |
|---|---|
| `t_conv_frac_s` | **tiempo de congregación**: segundos desde el comando `MEET` hasta que el 90 % de los robots está dentro del radio y se mantiene 3 s |
| `t_conv_all_s` | lo mismo exigiendo los 10 |
| `arrived` | cuántos robots quedaron dentro del radio al final |
| `centroid_to_target_mm` | distancia del centroide del enjambre al punto de encuentro |
| `final_compaction_d` | compactación: √Σd² al centroide, en diámetros de Atta |
| `final_rms_d` | la anterior por robot (√Σd²/N) — **es la comparable entre corridas con distinto N**, porque `final_compaction_d` crece con √N |
| `rw_s`, `dead_s`, `meet_s` | duración de cada fase: caminata aleatoria, tiempo muerto, congregación |
| `passage_d` | pasaje efectivo en diámetros · `passage_min_d` el mínimo entre pares |
| `occupancy_pct`, `obstacle_area_d` | ocupación de superficie y área relativa del obstáculo |

En `sim_robots.csv`: `route_ratio` es el recorrido dividido por la distancia
euclidiana directa (1.0 = fue en línea recta), `t_stop_s` cuándo se detuvo ese
robot, y `rw_active_s` / `meet_active_s` cuánto tiempo estuvo efectivamente en
movimiento en cada fase.

## Dos cosas que hay que saber para leer los números

**El radio de convergencia no es el mismo que en el lab, y no puede serlo.** El
punto de encuentro está pegado a la pared a propósito, para que el enjambre se
congregue en semicírculo. Eso hace que el firmware agrande el anillo cuando los
puestos caerían sobre la pared: con 10 robots el anillo teórico es de 405 mm y el
medido fue de 687, que es 405 × 1.7, uno de los factores de crecimiento del
firmware. El criterio usado es **R = anillo efectivo + 2 diámetros**, que da
900 mm acá y 610 mm en el laboratorio con 4 robots. Es la misma regla, no dos
números elegidos a dedo, y responde a lo que pedía el diseño: radio en función
del diámetro y de la cantidad de robots.

**El alcance del sensor infrarrojo está fijado en 50 mm**, que es el medido en
los HW-488 reales. Es crítico: el modelo traía 200 mm y con ese valor los dos
escenarios de 2 diámetros no se cruzaban nunca. Con un robot centrado en una
apertura de 210 mm el sensor lateral ve la pared a 158 mm, así que con 200 mm de
alcance dispara siempre y el robot entra en evasión perpetua — mientras que en el
laboratorio esas mismas aperturas se cruzan sin problema. Con 50 mm las 15
corridas convergen. Cualquier comparación entre laboratorio y simulación depende
de que ese número sea el mismo en los dos lados.

## Resultado

| escenario | pasaje | congregación | compactación |
|---|---|---|---|
| SinObs | — | 17.1 s | 17.6 d |
| BosqueG_D4 | 3.92 d | 23.2 s | 17.7 d |
| BosqueD_D4 | 3.81 d | 29.3 s | 18.4 d |
| BosqueD_D2 | 1.81 d | 34.0 s | 18.2 d |
| BosqueG_D2 | 2.00 d | 49.2 s | 19.4 d |

ANOVA de una vía sobre el escenario: **F(4,10) = 13.81, p = 0.00044**.

Separando los dos factores del diseño:

- **el ancho de pasaje explica el tiempo** — 2 diámetros 42.1 s contra 4
  diámetros 26.3 s, F = 15.96, **p = 0.0025**;
- **el grosor del obstáculo no** — delgado 33.3 s contra grueso 35.1 s,
  F = 0.09, p = 0.78.

Las corridas rozan los obstáculos entre un 0.01 % y un 0.19 % del tiempo, con
una penetración máxima de 1.4 mm; está dentro de tolerancia y queda registrado en
`clearance_mm` y `contact_pct` del manifiesto. Si en alguna repetición el
contacto subiera de forma consistente, eso ya no sería ruido sino señal de que la
apertura está en el límite de lo navegable.

## Cómo se regenera

```
python analyze_logs.py --campaign simulacion_31-07/manifiesto.csv --radius 900
python figuras_paper.py sim
```
