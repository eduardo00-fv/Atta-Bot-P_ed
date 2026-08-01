# Notas de trabajo sobre el paper — 31/07/2026

Repaso de `main.tex` contra los datos, revisión de las figuras de Juan Carlos, y
material para las secciones que me tocan. Todo lo que va entre `%%%` es texto
listo para pegar en LaTeX.

---

## 1. Cosas del texto nuevo que hay que resolver antes de escribir resultados

### 1.1 El área del obstáculo grueso está en dos valores distintos

El párrafo dice `1.48 · a_robot` y la lista dice `2.12 · a_robot`; las figuras
dicen `2.1a`. Con el robot de 105 mm de diámetro el área es 8659 mm², así que:

| caja | área | ratio |
|---|---|---|
| 8×8 cm | 6400 mm² | 0.74 a — coincide con el paper |
| 8×16 cm | 12800 mm² | **1.48 a** |
| 8×23 cm | 18400 mm² | **2.12 a** |

Medí las cajas desde los videos del laboratorio. La medida absoluta se infla
~1.4× por lado (la homografía se ajusta al plano de los marcadores y las cajas
son más altas), pero **el cociente entre grueso y delgado sí es fiable porque la
inflación se cancela: 2.52**. Con el delgado en 0.74 a, eso pone al grueso en
**~1.9 a**, lo que descarta que sea una caja de 8×16 cm. Hay que medirla con
regla y corregir el párrafo, no la lista.

### 1.2 «Aggregation time» está definido desde el inicio del experimento

> *defined from the start of the experiment and finalizes when the 90\% of agents
> reach the aggregation zone*

Medido sobre los logs, cada corrida contiene tres cosas: la caminata aleatoria,
un **tiempo muerto mientras el operador manda el comando a mano**, y la
congregación. El tiempo muerto va de 0 a 46 s, con mediana de 20. Contarlo mete
en la métrica una variable que no es del escenario, y explica buena parte de la
dispersión de la Fig. 2 (la caja de `0.7a-2d` va de 21 a 179 s).

Propuesta: definir el tiempo de congregación **desde el comando**, no desde el
inicio del experimento. Lo tengo separado corrida por corrida en
`analisis_30-07/lab_runs.csv` (columnas `rw_s`, `dead_s`, `meet_s`).

### 1.3 Falta definir la «aggregation zone»

La métrica dice «llegar a la zona de congregación» pero el radio no está
definido en ninguna parte, y **no puede ser una constante**: el comportamiento
reparte a los robots sobre un anillo cuyo radio depende de cuántos son, y además
el firmware lo agranda si los puestos caerían sobre una pared. Con 4 robots el
anillo pedido es de 300 mm; con 10 sube a 405 mm y, medido, terminó en 687 mm
por la corrección de pared.

Lo que uso y creo que hay que declarar en el paper:

> R = radio efectivo del anillo + 2 diámetros de robot

Da 610 mm con 4 robots y 900 mm con 10. Es una sola regla, coherente con lo que
pedía el diseño (radio en función del diámetro y de la cantidad de robots), y no
dos números elegidos a dedo.

### 1.4 El criterio del 90 % se vuelve trivial con 4 robots

⌈0.9 × 4⌉ = 4. Con cuatro robots el 90 % **exige los cuatro**, o sea cero
tolerancia a rezagados, mientras que con diez tolera uno. Si el laboratorio y la
simulación usan «90 %» literalmente, no están midiendo lo mismo.

Para que sean comparables usé **3 de 4 en el laboratorio y 9 de 10 en la
simulación** — la misma tolerancia, un rezagado. Hay que decidirlo en grupo y
declararlo explícitamente, porque cambia los números.

### 1.5 La compactación de la ecuación (1) y la de la Fig. 3 no coinciden

La ecuación tiene el `1/N` dentro de la raíz, o sea es un RMS. Comparando las
medianas de la Fig. 3 contra lo que da esa fórmula sobre los mismos logs:

| escenario | Fig. 3 | ecuación (1) | cociente |
|---|---|---|---|
| NoObs | 4.7 | 3.34 | 1.41 |
| 0.7a-2d | 4.9 | 3.38 | 1.45 |
| 0.7a-4d | 5.6 | 3.98 | 1.41 |
| 2.1a-2d | 7.5 | 5.35 | 1.40 |
| 2.1a-4d | 5.8 | 4.20 | 1.38 |

El cociente es constante (~1.41 = √2), así que la forma coincide y lo que hay es
un factor de escala: seguramente una normalización distinta. Vale la pena
alinearlo antes de publicar el número.

**Y esto es importante para enlazar la simulación:** con el `1/N` dentro de la
raíz la métrica **sí** es comparable entre 4 y 10 robots. Sin el `1/N`, √Σd²
crece con √N, y la simulación parecería un 58 % «menos compacta» solo por tener
más robots. Si el número de la Fig. 3 no lleva el `1/N`, no se puede poner al
lado del de la simulación.

---

## 2. Sobre las figuras de Juan Carlos

**Fig. 2 (tiempo)** — el orden general es razonable, pero la dispersión de
`0.7a-2d` (21 a 179 s) es casi toda tiempo muerto del operador, ver 1.2.

**Fig. 1 (distancia normalizada)** — hay valores **negativos**, hasta −2 en
`NoObs`. Un recorrido dividido por una distancia euclidiana no puede dar
negativo, así que ahí hay algo que revisar. Sospecha: el robot líder se queda
quieto y su distancia en línea recta es de milímetros, con lo que el cociente
diverge o cambia de signo según cómo se calcule. En mi análisis excluyo a los
robots cuyo desplazamiento neto es menor que un diámetro, justamente por eso;
medí un cociente de 56.2 en un robot con 17 mm de recta.

**Fig. 3 (compactación)** — ver 1.5. Además el bigote de `2.1a-4d` llega a 22.8,
que probablemente sea una corrida donde un robot nunca cruzó; conviene decidir si
esas entran o se reportan aparte como fallo de congregación.

---

## 3. Mis secciones

### 3.1 Robot tracking system

Lo del borrador anterior sigue valiendo tal cual: cámara cenital, ArUco
`DICT_4X4_50`, `solvePnP` con `IPPE_SQUARE`, ángulo tomado de la arista del
marcador en vez del vector de rotación, marcador fijo como origen, mediana de
tres detecciones y descarte por lado aparente mínimo. **Un párrafo para agregar**,
porque salió de trabajar los datos de estas corridas:

%%%
The recorded video and the pose log share a common time base, since each frame is
timestamped at capture time rather than after detection; the ArUco stage takes a
variable time per frame and would otherwise contaminate the interval between
consecutive samples. The pipeline sustains between 10 and 15 frames per second
depending on how many markers are simultaneously visible, and marker recovery
after an occlusion converges within two frames.
%%%

### 3.2 Robot behaviors

También se conserva. Lo único que cambió desde el borrador es **la fórmula del
radio**, que el nuevo texto pide explícitamente («Poner fórmula para el radio de
congregación»). La versión vigente reparte los puestos por la CUERDA entre
vecinos, no por el arco:

%%%
Agents are distributed over a ring centred on the meeting point. The ring radius
is not a fixed parameter but is derived from the desired separation between
neighbouring slots: for $n$ agents placed at equal angular intervals, requiring
that the chord between two consecutive slots equal $s$ gives

\begin{equation}
    R_{ring} = \max\left(\frac{s}{2\sin(\pi/n)},\; R_{min}\right)
    \label{eq:ring}
\end{equation}

with $s = 250$~mm, slightly above two chassis diameters, and a floor
$R_{min} = 200$~mm that keeps the group from collapsing when only a few agents
are present. Every agent evaluates (\ref{eq:ring}) from the same shared
neighbour list and assigns slots with the same nearest-first rule followed by a
pairwise exchange pass, so all agents reach an identical assignment without
negotiation and no two of them claim the same slot. When the meeting point lies
close to a wall, the slots that would fall outside the arena are redistributed
over the largest safe arc and the radius is grown accordingly; the swarm then
congregates in a half ring rather than a full one.
%%%

**Nota para el grupo:** vale la pena decir explícitamente que el punto de
encuentro se eligió **pegado a la pared** para forzar la congregación en
semicírculo. No es un detalle de implementación, es lo que hace que el radio
efectivo dependa del escenario y del número de robots.

### 3.3 Swarm simulation

El borrador anterior está bien y no lo tocaría, salvo por **un párrafo nuevo que
ahora sí se puede escribir con datos**, sobre el alcance del infrarrojo. Es el
hallazgo más útil de la validación y conviene que esté en el paper porque es
justamente el tipo de detalle que hace que una simulación reproduzca o no al
laboratorio:

%%%
One modelling parameter proved decisive and is reported explicitly. The infrared
proximity sensors are threshold devices whose triggering distance is set by a
potentiometer on each unit, so their range is a calibration choice rather than a
physical constant. With the range initially set to 200~mm the simulated swarm
never traversed the narrow configurations: for an agent centred in a $2 \cdot
d_{robot}$ gap the lateral sensors, mounted at $30^\circ$, see the obstacle at
158~mm and therefore fire continuously, driving the agent into perpetual
avoidance. The physical robots cross the same gaps without difficulty because
their measured range is between 40 and 60~mm. Setting the simulated range to the
measured value reproduces the laboratory outcome and all runs converge. The
implication generalizes beyond this platform: for a reactive agent the relevant
passage width is not measured against its body but against its sensing range, and
a gap that is geometrically ample can be functionally impassable.
%%%

---

## 4. Resultados de la simulación — borrador

Números finales, con R = anillo efectivo + 2 diámetros = 900 mm y el criterio de
9 de 10 robots:

| escenario | pasaje efectivo | tiempo | compactación (ec. 1) |
|---|---|---|---|
| NoObs | — | 17.1 s | 5.56 |
| 2.1a-4d | 3.92 d | 23.2 s | 5.61 |
| 0.7a-4d | 3.81 d | 29.3 s | 5.82 |
| 0.7a-2d | 1.81 d | 34.0 s | 5.76 |
| 2.1a-2d | 2.00 d | 49.2 s | 6.13 |

ANOVA de una vía sobre el escenario: **F(4,10) = 13.81, p = 0.00044**.

%%%
The ten-agent simulation resolves the effect that the physical experiments
suggest but cannot establish. Congregation time grows monotonically as the
passage narrows, from $17.1$~s in the obstacle-free control to $49.2$~s in the
most constrained configuration, and a one-way ANOVA over the five configurations
rejects the null hypothesis of equal means ($F(4,10) = 13.81$, $p = 0.00044$).

Separating the two factors of the design shows which one carries the effect. The
gap width accounts for the difference: configurations with $2 \cdot d_{robot}$
gaps take $42.1$~s on average against $26.3$~s for $4 \cdot d_{robot}$
($F = 15.96$, $p = 0.0025$). The obstacle size does not: the small and large
obstacle configurations take $33.3$~s and $35.1$~s respectively, a difference
indistinguishable from noise ($F = 0.09$, $p = 0.78$). This is consistent with
the hypothesis that what governs aggregation under obstacles is the width of the
passage rather than the amount of material blocking the arena, and it holds even
though the total obstacle area was kept constant across configurations by
construction.
%%%

### Enlazar con el laboratorio — y hay que hacerlo con cuidado

Aplicando el **mismo criterio** en los dos lados, el laboratorio da: NoObs 11.5 s,
0.7a-4d 23.2, 0.7a-2d 30.0, 2.1a-4d 46.8, 2.1a-2d 56.1. El orden coincide con la
simulación, pero **el laboratorio no alcanza significancia** (p = 0.40), y
separando factores tampoco: pasaje p = 0.61, tamaño p = 0.50.

No se puede escribir que el laboratorio confirma la simulación. Lo honesto:

%%%
The physical experiments are consistent with this ordering but do not resolve it:
with three repetitions per configuration neither factor reaches significance
($p = 0.61$ for the gap width and $p = 0.50$ for the obstacle size). Two
explanations are compatible with the available data and the present design cannot
separate them. The first is simply statistical power. The second is that the
effect is driven by congestion at the passage, which four agents crossing a
2.4~m barrier hardly produce while ten do; under this reading swarm size is not a
scaling parameter of the phenomenon but a precondition for it. Distinguishing the
two requires either more repetitions at the current swarm size or a larger
physical swarm, and this is the immediate continuation of the work.
%%%

Esto además le da contenido concreto a Conclusions and future work.

---

## 5. Preguntas para el grupo

1. ¿El obstáculo grueso es de 8×16 o de 8×23 cm? Hay que medirlo (§1.1).
2. ¿Contamos el tiempo de congregación desde el comando o desde el inicio de la
   corrida? (§1.2)
3. ¿Qué radio declaramos como zona de congregación y cómo lo justificamos? (§1.3)
4. ¿90 % literal, o «todos menos uno» en ambos experimentos? (§1.4)
5. ¿La compactación de la Fig. 3 lleva el `1/N`? De eso depende que se pueda
   poner al lado de la de la simulación. (§1.5)
6. ¿Las corridas donde un robot nunca cruzó entran en los boxplots o se reportan
   aparte como fallo? Cambia las medianas y los bigotes.
