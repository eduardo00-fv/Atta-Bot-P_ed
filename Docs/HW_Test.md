# HW_Test — diagnóstico de hardware del AttaBot

Sketch independiente para saber **qué se rompió** en un robot: motores, encoders,
IMU, IR o batería. No usa WiFi, base ni cámara, así que sirve justo cuando el
robot no conecta o "dejó de andar" y no se puede diagnosticar por UDP.

`Controller/HW_Test/HW_Test.ino`

## Cuándo usar cuál

| Situación | Herramienta |
|---|---|
| El robot **no conecta** o no responde | **HW_Test.ino** (este) |
| El robot conecta y querés no desflashear | `herramientas/ir_check.py --motors` (comando `SELFTEST`) |
| Los IR disparan solos / no detectan | `herramientas/ir_check.py --pot C` · `--monitor` |
| Los giros salen cortos o largos | `CALIBRATE.<id>` desde la consola de la base |

## Uso

1. Flashear `Controller/HW_Test/HW_Test.ino`.
2. Monitor Serie a **115200**, terminación **"Nueva línea"**.
3. Escribir `t` para el test completo.
4. Al terminar, **re-flashear el firmware normal** (`Controller/AttaBot/AttaBot.ino`).

> El robot **gira sobre su eje** durante los tests de motor: dejale ~30 cm libres
> alrededor. Si lo sostenés en el aire los pulsos siguen siendo válidos, pero el
> giro que reporta la IMU no.

## Comandos

| Comando | Qué hace |
|---|---|
| `t` | Test **completo**: batería → motores → IR → IMU |
| `m` | Solo motores (izquierda sola → derecha sola → ambas) |
| `e` | Encoders en vivo — **girá las ruedas a mano** y mirá los contadores |
| `i` | IR en vivo (IZQ/DER digitales + APDS central crudo) |
| `g` | IMU en vivo (yaw) |
| `b` | Batería |
| `p <n>` | Cambia el PWM de prueba en % (default 40; subilo si el robot no arranca) |
| `s` | STOP de emergencia |
| `?` | Ayuda |

## Cómo leer el resultado de motores

El test mueve **cada motor por separado** y cruza tres fuentes: pulsos del
encoder propio, pulsos del ajeno, y el giro medido por la IMU. Ese cruce es lo
que separa fallas que a simple vista son idénticas ("el robot no anda"):

| Medición | Diagnóstico | Qué revisar |
|---|---|---|
| 0 pulsos **y** la IMU no gira | **Motor muerto** | Cableado del motor, canal del puente H |
| 0 pulsos **pero** la IMU gira | **Encoder muerto** | El motor empuja pero no cuenta: cable del encoder |
| Pulsos < 75% del otro lado | **Motor flojo** | Se irá de lado al avanzar; escobillas, fricción, batería |
| El encoder **ajeno** también cuenta | **Cableado cruzado** | Canales de encoder intercambiados |

Ejemplo de salida sana:

```
  IZQ sola  pulsos_izq=  312  pulsos_der=    2  dyaw= +38.4°
  DER sola  pulsos_izq=    1  pulsos_der=  308  dyaw= -37.9°
  AMBAS     pulsos_izq=  301  pulsos_der=  298  dyaw=  +1.2°

--- Veredicto ---
  IZQUIERDO  ok (312 pulsos, IMU +38°)
  DERECHO    ok (308 pulsos, IMU -38°)
  AMBAS      ok (301 vs 298, desvío +1°)
```

Con un motor izquierdo muerto se vería así:

```
  IZQ sola  pulsos_izq=    3  pulsos_der=    0  dyaw=  +0.2°
  ...
  IZQUIERDO  MOTOR MUERTO — sin pulsos y sin giro (+0°)
  AMBAS      DESBALANCE 1% — el motor izquierdo rinde menos (3 vs 297)
```

### El comando `e` es el complemento

`m` prueba motor **+** encoder juntos. Si `m` da 0 pulsos, `e` desempata: girá
la rueda con la mano y mirá si el contador se mueve.

- Cuenta al girarla a mano → el **encoder está bien**, el problema es el motor.
- No cuenta → el problema es el **encoder o su cableado**.

## Notas

- Los pines, la decodificación en cuadratura de los encoders y la lectura de yaw
  (drenaje del FIFO del DMP, Quat9) son **copia del firmware**, para que lo que
  midas acá valga en el firmware real.
- Si la IMU aparece como `NO DETECTADA`, el test igual corre pero **no puede
  distinguir motor muerto de encoder muerto** — avisa al empezar.
- La batería baja afecta la fuerza de los motores: si sale `BAJA`, cargá antes
  de sacar conclusiones sobre un motor "flojo".
- El PPR nominal (574) solo se usa para el "≈ mm por rueda" informativo. Cada
  robot tiene el suyo calibrado; ver `CALIBRATE`.
