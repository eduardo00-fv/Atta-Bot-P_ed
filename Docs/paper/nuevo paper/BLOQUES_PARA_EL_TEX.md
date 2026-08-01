# Bloques para el `.tex` — 31/07/2026

Extraídos del `main.tex` que compila sin errores (7 páginas), así que lo
que está acá es exactamente lo verificado.

## Archivos que hay que subir

| archivo | qué es |
|---|---|
| `scenario_configurations_lab.pdf` | configuraciones de obstáculos, arena física |
| `scenario_configurations_sim.pdf` | ídem, arena simulada |
| `results_sim.pdf` | los tres boxplots de la simulación |
| `trajectories_sim.pdf` | trayectorias de las 15 corridas simuladas |

Los cuatro están en `Docs/paper/nuevo paper/`.

---

## 0. Preámbulo

**AGREGAR una línea.** Junto a los otros `\usepackage`. Sin esto el pseudocódigo no compila: `algorithmic` es el cuerpo, `algorithm` es el flotante que le da `\caption` y `\label`.

```latex
\usepackage{algorithm}
```

---

## 1. Palabras clave

**REEMPLAZAR el contenido de `IEEEkeywords`.** Están las de la plantilla (*component, formatting, style…*).

```latex
swarm robotics, aggregation, environment topology, obstacle avoidance,
physical experimentation, robot simulation
```

---

## 2. Robot tracking system

**REEMPLAZAR la subsección entera.** Donde dice `EDUARDO` y las indicaciones en español.

```latex
\subsection{Robot tracking system}

Global localization of the swarm is provided by an external overhead vision
system rather than by onboard absolute sensing. A monocular camera is mounted on
the ceiling with its optical axis approximately perpendicular to the floor,
capturing at $1280 \times 720$ pixels, which yields a scale of approximately
$1.70$~mm per pixel at the working height. Exposure, focus and white balance are
fixed manually so that detection conditions remain repeatable across runs. The
camera intrinsics are calibrated beforehand, and every frame is undistorted
through precomputed remapping tables.

Each robot carries a unique ArUco marker from the \texttt{DICT\_4X4\_50}
dictionary on its upper face, so agent identity is resolved directly from the
marker ID, without colour segmentation or nearest-neighbour association between
consecutive frames. Marker pose is obtained with \texttt{solvePnP} using the
\texttt{IPPE\_SQUARE} solver on the undistorted corner points. The heading,
however, is taken from the direction of the marker's top edge rather than from
the PnP rotation vector: for a flat marker observed almost perpendicularly,
\texttt{IPPE\_SQUARE} returns two solutions with almost equal reprojection
error, and the flickering tie-break between them dominated the orientation
noise, producing a standard deviation of approximately $3.8^\circ$ with a
stationary robot, against the $\approx 0.5^\circ$ attributable to corner jitter
alone. Using the edge direction removes that ambiguity.

A fixed marker placed on the floor defines the origin of the world frame, so all
poses are expressed relative to it and remain consistent across sessions even if
the camera mounting shifts slightly. A per-robot angular offset compensates
markers that are not perfectly aligned with the chassis. Two filters protect the
pose stream: a per-marker median of three consecutive detections, which removes
single-frame misreads, and a minimum apparent marker side of $15$~px, which
discards small false positives. The latter guard matters because a spurious
detection carrying a valid robot ID would inject a fabricated pose into both the
swarm and the recorded data.

The pipeline runs at approximately $20$~Hz. Each pose is written to a log for
offline computation of the metrics and, simultaneously, transmitted to the
corresponding robot over UDP. Since the robots have no other source of absolute
position, the vision system acts both as the measurement instrument and as a
component of the control loop.
```

---

## 3. Robot behaviors

**REEMPLAZAR la subsección entera.** Incluye la tabla de evasión, las tres ecuaciones y los dos algoritmos.

```latex
\subsection{Robot behaviors}

Two behaviors are combined during the experiments: a reactive obstacle avoidance
layer and a goal-directed congregation behavior. Both run onboard; the base
station only supplies pose and target information.

\subsubsection{Goal-directed navigation}
Motion towards a target is executed in segments. On each cycle the robot
requests its pose, computes the bearing to the goal, and realigns with a
rotation if the heading error exceeds $12^\circ$; it then advances a straight
segment of at most $250$~mm and repeats. Arrival is declared when the robot is
within $50$~mm of the goal. Rotations are closed-loop on the yaw estimated by
the IMU rather than commanded open-loop from wheel odometry: the remaining arc
is re-aimed on every cycle, the motors are cut $3^\circ$ before the target so
that inertia completes the rotation, and up to four iterative corrections are
issued until the residual error falls below $3^\circ$. The yaw is obtained
from the quaternion produced by the sensor's digital motion processor, and a
per-robot gyroscope scale factor compensates the unit-to-unit differences
measured during calibration.

\subsubsection{Obstacle avoidance}
Obstacle sensing combines two lateral infrared sensors, one on each side, with
a central optical proximity sensor facing forward. The infrared pair are
threshold devices whose triggering distance is set by a potentiometer on each
unit, so each one reports a single bit; the central sensor is an APDS9960, which
returns a graded proximity reading over the I\textsuperscript{2}C bus and is
thresholded in firmware. The three together produce a three-bit occupancy
pattern. Each pattern maps to a rotation away from the obstructed side followed by a
clearance advance, as listed in Table~\ref{tab:avoidance}. Purely
lateral detections produce mild corrections, whereas frontal and bilateral
detections produce larger rotations and shorter advances. Symmetric patterns,
which carry no directional information, resolve the turn direction randomly to
avoid deterministic deadlocks between facing robots.

\begin{table}[htbp]
\caption{Avoidance maneuver as a function of the occupancy pattern, with L
and R the lateral infrared sensors and C the central proximity sensor.}
\label{tab:avoidance}
\centering
\begin{tabular}{cccl}
\hline
\textbf{Pattern} & \textbf{Rotation} & \textbf{Advance} & \textbf{Situation} \\
\hline
100 & $45^\circ$          & 120~mm & Left obstacle \\
001 & $45^\circ$          & 120~mm & Right obstacle \\
010 & $60^\circ$ (random) & 150~mm & Frontal obstacle \\
110 & $90^\circ$          & 150~mm & Left and frontal \\
011 & $90^\circ$          & 150~mm & Right and frontal \\
111 & $135^\circ$ (random)& 100~mm & Fully blocked \\
\hline
\end{tabular}
\end{table}

Because the base station broadcasts the position of every agent, a detection can
be classified as a peer rather than as a static obstacle when it falls within
$260$~mm and within $80^\circ$ of the sensor bearing. This distinction is
relevant in the congregation task, where agents necessarily converge and would
otherwise treat each other as walls. Detections are not acted upon immediately:
a candidate obstacle must persist for $t_{confirm} = 600$~ms before a maneuver
is committed, and if the obstruction is identified as another agent the robot
yields for $t_{yield} = 200$~ms and re-evaluates, since two agents evading each
other simultaneously tend to produce oscillation rather than progress.

Repeated evasions within a short window, typical of a corner or of a narrow
passage, trigger a committed escape maneuver: a reverse, a rotation of
$\varphi = 95^\circ$ and a detour of $L = 300$~mm. The turning side is not
chosen blindly; both candidates are projected forward and the one ending farther
from the walls is selected, as expressed in (\ref{eq:escape}), where
$(x,y,\theta)$ is the robot pose and $W \times H$ the arena dimensions:

\begin{equation}
s^{*} = \operatorname*{arg\,max}_{s \in \{-1,+1\}}
\min \left( e_x,\; W - e_x,\; e_y,\; H - e_y \right)
\label{eq:escape}
\end{equation}

\begin{equation}
e_x = x + L\cos(\theta + s\varphi), \quad
e_y = y + L\sin(\theta + s\varphi)
\label{eq:escape_proj}
\end{equation}

The escape maneuver is armed by a counter of consecutive evasions: three
evasions within a $5$~s window indicate that the local avoidance rule is not
making progress, which is the signature of a corner or of a passage narrower
than the maneuvering envelope of the robot. Algorithm~\ref{alg:nav}
summarizes the complete navigation cycle.

\begin{algorithm}
\caption{Goal-directed navigation with obstacle avoidance}
\label{alg:nav}
\begin{algorithmic}[1]
\REQUIRE goal $p_g$; pose $(x,y,\theta)$ supplied by the vision system
\STATE $k \leftarrow 0$ \COMMENT{consecutive evasion counter}
\WHILE{$\lVert p_g - (x,y) \rVert > d_{arrival}$}
    \STATE request current pose from the base station
    \STATE $\alpha \leftarrow \operatorname{atan2}(y_g - y,\; x_g - x) - \theta$
    \IF{$\lvert \alpha \rvert > \alpha_{realign}$}
        \STATE rotate $\alpha$, closed loop on IMU yaw
    \ENDIF
    \STATE $d \leftarrow \min \left( d_{segment},\; \lVert p_g - (x,y) \rVert \right)$
    \STATE advance $d$
    \IF{proximity sensors triggered during the segment}
        \STATE $b \leftarrow$ occupancy pattern (L,\,C,\,R)
        \STATE broadcast own pose and $b$; wait $t_{confirm}$
        \IF{a peer claims to be the sensed body}
            \STATE wait $t_{yield}$ \COMMENT{yield, do not evade}
        \ELSE
            \STATE $(\varphi_a, d_a) \leftarrow \textsc{Lookup}(b)$
            \STATE rotate $\varphi_a$ away from the obstructed side
            \STATE advance $d_a$
            \STATE $k \leftarrow k + 1$
        \ENDIF
        \IF{$k \geq k_{max}$}
            \STATE reverse; rotate $s^{*}\varphi$ using (\ref{eq:escape});
                   advance $L$
            \STATE $k \leftarrow 0$
        \ENDIF
    \ENDIF
\ENDWHILE
\STATE stop
\end{algorithmic}
\end{algorithm}

\subsubsection{Congregation}
The congregation behavior aggregates the swarm around a target point without a
designated leader, so that agents must cross the obstacle field to reach a
region where no other agent is initially present. Every robot receives the same
target and the same list of peer positions, broadcast at $1$~Hz.

Driving all agents to the identical point would end in mutual evasion and
crowding around the target, so each robot instead occupies a distinct slot on a
ring centred on it. The ring radius grows with the swarm size in order to keep a
constant arc length per agent, with a lower bound for small swarms, as defined
in (\ref{eq:ring}), where $n$ is the number of known agents,
$s_{arc} = 250$~mm is the arc allocated per agent and $R_{min} = 300$~mm:

\begin{equation}
R_{ring} = \max \left( R_{min},\; \frac{n \cdot s_{arc}}{2\pi} \right)
\label{eq:ring}
\end{equation}

The $n$ slots are then distributed uniformly over the ring around the target
$p_{t} = (x_t, y_t)$, following (\ref{eq:slots}):

\begin{equation}
p_{s} = p_{t} + R_{ring} \left( \cos \frac{2\pi s}{n},\;
\sin \frac{2\pi s}{n} \right), \quad s = 0, \ldots, n-1
\label{eq:slots}
\end{equation}

Slot assignment is decided independently by each agent and requires no
negotiation. Every robot sorts the shared list of peers by identifier and then
greedily matches robot--slot pairs in increasing order of distance, followed by
a pairwise exchange pass that removes any crossing between assigned paths. Since
all robots sort the same list and apply the same deterministic rule, they all
derive the same assignment and no slot is claimed twice. The approach to the
assigned slot is performed in two stages: the robot first navigates to a staging
waypoint located $150$~mm outside its slot along the radial direction, and only
then enters radially, which prevents an agent from cutting across the ring and
traversing slots that are already occupied.
Algorithm~\ref{alg:cong} details the procedure.

\subsubsection{Congregation}
The congregation behavior aggregates the swarm around a target point without a
designated leader, so that agents must cross the obstacle field to reach a
region where no other agent is initially present. Every robot receives the same
target and the same list of peer positions, broadcast at $1$~Hz.

Driving all agents to the identical point would end in mutual evasion and
crowding around the target, so each robot instead occupies a distinct slot on a
ring centred on it. The ring radius grows with the swarm size in order to keep a
constant arc length per agent, with a lower bound for small swarms, as defined in
equation \ref{eq:ring}, where $n$ is the number of known agents,
$s_{arc} = 250$~mm is the arc allocated per agent and $R_{min} = 300$~mm:

\begin{equation}
R = \max \left( R_{min},\; \frac{n \cdot s_{arc}}{2\pi} \right)
\label{eq:ring}
\end{equation}

The $n$ slots are then distributed uniformly over the ring around the target
$p_{t} = (x_t, y_t)$, following equation \ref{eq:slots}:

\begin{equation}
p_{s} = p_{t} + R \left( \cos \frac{2\pi s}{n},\; \sin \frac{2\pi s}{n} \right),
\quad s = 0, \ldots, n-1
\label{eq:slots}
\end{equation}

Slot assignment is decided independently by each agent and requires no
negotiation. Every robot sorts the shared list of peers by identifier and then
greedily matches robot--slot pairs in increasing order of distance, which
minimizes path crossing between agents. Since all robots sort the same list and
apply the same deterministic rule, they all derive the same assignment and no
slot is claimed twice. The approach to the assigned slot is performed in two
stages: the robot first navigates to a staging waypoint located $150$~mm outside
its slot along the radial direction, and only then enters radially. Without this
two-stage entry an agent may cut across the ring and traverse slots that are
already occupied. Algorithm~\ref{alg:cong} details the procedure.

\begin{algorithm}
\caption{Leaderless congregation on a target point}
\label{alg:cong}
\begin{algorithmic}[1]
\REQUIRE target $p_t$; peer positions $P$, broadcast at $1$~Hz
\STATE $A \leftarrow P \cup \{\textit{self}\}$;\quad $n \leftarrow \lvert A \rvert$
\STATE $R \leftarrow \max \left( R_{min},\; n \, s_{arc} / 2\pi \right)$
       \COMMENT{ring radius, (\ref{eq:ring})}
\FOR{$s = 0$ \TO $n-1$}
    \STATE $p_s \leftarrow p_t + R \left( \cos \tfrac{2\pi s}{n},\;
           \sin \tfrac{2\pi s}{n} \right)$
\ENDFOR
\STATE sort $A$ by identifier
       \COMMENT{identical ordering on every agent}
\STATE $U_a \leftarrow A$;\quad $U_s \leftarrow \{p_0, \ldots, p_{n-1}\}$
\WHILE{$U_a \neq \emptyset$}
    \STATE $(a^{*}, s^{*}) \leftarrow
           \operatorname*{arg\,min}_{a \in U_a,\, s \in U_s}
           \lVert \textit{pos}(a) - p_s \rVert$
    \STATE assign $a^{*} \rightarrow s^{*}$; remove both from $U_a$, $U_s$
\ENDWHILE
\STATE $p_{slot} \leftarrow$ slot assigned to \textit{self}
\STATE $\hat{u} \leftarrow (p_{slot} - p_t) / \lVert p_{slot} - p_t \rVert$
       \COMMENT{outward radial direction}
\STATE navigate to $p_{slot} + m \, \hat{u}$
       \COMMENT{staging waypoint, $m = 150$~mm}
\STATE navigate to $p_{slot}$
       \COMMENT{radial entry}
\STATE stop and hold position
\end{algorithmic}
\end{algorithm}

When the target point lies close to a wall, the slots that would fall outside
the arena are redistributed over the largest safe arc and the radius is grown by
a factor $k$ until $n$ slots of arc $s_{arc}$ fit within it; the swarm then
congregates in a half ring rather than a full one. This is not incidental to the
experiment, since the target was deliberately placed against the far wall: the
effective congregation radius is $k \cdot R_{ring}$ and therefore depends on both
the swarm size and the arena geometry. In the physical runs the algorithm
selected $k = 1$, giving a commanded radius of $300$~mm; in the ten-agent
simulation it selected $k = 1.7$, giving $688$~mm.
```

---

## 4. Swarm simulation

**REEMPLAZAR la subsección entera.** Sin la figura del arena de Webots: esa imagen no existe en el repositorio.

```latex
\subsection{Swarm simulation}

The physical arena bounds both the number of agents and the range of obstacle
configurations that can be tested. To examine whether the dependence on topology
persists at a larger swarm size, the swarm was replicated in the Webots robotics
simulator.

The simulated agent reproduces the Atta-bot physically and logically. The model
matches the chassis diameter, mass distribution and differential-drive geometry
of the platform, and instruments it with the same sensor set: wheel encoders
feeding the same PID motion control, and the two lateral infrared sensors plus
the central proximity sensor with equivalent placement and detection cones. The behaviors described above are
ported without modification of their logic or of their numeric constants, so
segment lengths, confirmation windows, avoidance angles and ring geometry are
identical in both settings.

The central property of the setup is protocol identity: simulated robots
exchange the same UDP messages as the physical ones. Consequently the same base
station software commands both, and the overhead vision system is replaced by a
supervisor that publishes poses in the same reference frame, units and message
format as the ArUco pipeline. The behavior under evaluation therefore traverses
the same code path in simulation and in the laboratory, and the metrics are
computed by the same analysis scripts over logs of the same structure.

Two sources of realism are injected deliberately, because a noiseless simulation
would overstate the performance of the congregation behavior. First, positional
and angular jitter comparable to that characterized for the vision pipeline is
added to the published poses, and marker occlusion events are reproduced, so
that agents occasionally lose their absolute reference exactly as they do in the
laboratory. Second, the agents are not identical to each other: per-unit
asymmetries measured on the physical robots, such as differences in effective
wheel scale and in infrared sensitivity, are assigned from a profile file.

Worlds are generated parametrically from the topology description, so each
obstacle configuration is instantiated with the same procedure that defines it
in the physical experiments, and repetitions differ only in the initial
placement of the agents.

One modelling parameter proved decisive and is reported explicitly. The range of
the lateral infrared sensors is set by a potentiometer on each unit, so it is a
calibration choice rather than a physical constant. With the range initially set to $200$~mm the simulated swarm
never traversed the narrow configurations: for an agent centred in a
$2 \cdot d_{robot}$ gap the lateral sensors, mounted at $30^\circ$, see the
obstacle at $158$~mm and therefore fire continuously, driving the agent into
perpetual avoidance. The physical robots cross the same gaps without difficulty
because their measured range is between $40$ and $60$~mm. Setting the simulated
range to the measured value reproduces the laboratory outcome and all runs
converge. The implication generalizes beyond this platform: for a reactive agent
the relevant passage width is not measured against its body but against its
sensing range, and a gap that is geometrically ample can be functionally
impassable.

The scalability scenario enlarges the arena to $3.8 \times 2.8$~m with ten
agents, preserving the obstacle size and gap width ratios of the physical
configurations. Increasing the number of agents changes the congregation problem
qualitatively and not only quantitatively: the ring radius of (\ref{eq:ring})
grows with $n$, so the target region itself becomes larger, while the passage
width remains fixed. This separates the contribution of swarm size from the
contribution of topology.

\begin{figure*}
    \centering
    \includegraphics[width=0.95\linewidth]{trajectories_sim.pdf}
    \caption{Trajectories of the ten simulated agents during the aggregation
    phase, with the three repetitions of each configuration superimposed. Boxes
    are the obstacles and the cross marks the meeting point. Trajectories are
    split wherever the localization of an agent is lost, so that no straight
    segment crosses an obstacle that the agent did not traverse.}
    \label{fig:sim_trajectories}
\end{figure*}
```

---

## 5. Scenario topology — área del obstáculo

**REEMPLAZAR en el párrafo.** Decía `1.48` en el párrafo y `2.12` en la lista. Confirmado 8×23 cm, o sea **2.12**.

```latex
and $2.12 \cdot a_{robot}$, area larger than the robot size.
```

---

## 6. Scenario topology — huecos construidos

**REEMPLAZAR la frase de los gaps.** Los huecos que se armaron no son los nominales; hay que reportar lo medido.

```latex
The passage gap widths between obstacles were nominally $2 \cdot d_{robot}$
(d=diameter) and $4 \cdot d_{robot}$. Since the physical obstacles were placed by
hand, the gaps actually built were measured from the overhead video and are
reported with the results: $2.3$ and $2.7 \cdot d_{robot}$ for the narrow
configurations and $4.9$ and $5.0 \cdot d_{robot}$ for the wide ones, that is,
somewhat wider than nominal but with the two levels cleanly separated by a factor
of two. The simulated scenarios, generated programmatically, match the
nominal values exactly. The obstacles total area was between 1,2\% and 1,5\% of the total scenario area, this ratio was keep constant among the different configurations.
```

---

## 7. Scenario topology — figura

**REEMPLAZAR el bloque `figure` del escenario.** La figura anterior mostraba el diseño planeado, no el montaje. Sube `scenario_configurations_lab.pdf` y `scenario_configurations_sim.pdf`.

```latex
\begin{figure}
    \centering
    \includegraphics[width=0.7\linewidth]{escenario.JPG} \\
    (a) \\
    \includegraphics[width=0.95\linewidth]{scenario_configurations_lab.pdf} \\
    (b) \\
    \includegraphics[width=0.95\linewidth]{scenario_configurations_sim.pdf} \\
    (c)
    \caption{Experimental scenario configurations. (a) Controlled environment
    used for physical swarm validation. (b) Obstacle layouts of the physical
    arena, drawn to scale; positions were recovered from the overhead video and
    obstacles are drawn at their nominal size. (c) Obstacle layouts of the
    simulated arena, where grey blocks close the barrier against the walls. The
    star marks the meeting point in both cases.}
    \label{fig:escenario}
\end{figure}
```

---

## 8. Scenario topology — definición de las métricas

**REEMPLAZAR los dos `\item` de tiempo y compactación.** El de la distancia queda igual: falta cotejarlo con Juan Carlos.

```latex
    \item Aggregation time: measured from the instant the aggregation command is
    issued, and finalized when all agents but one have reached the aggregation
    zone and remain inside it. The random walk that precedes the command, and the
    interval the operator takes to issue it, are excluded: they belong to the
    protocol and not to the scenario. The tolerance of one lagging agent is used
    at both swarm sizes, since a literal $90\%$ criterion is not equivalent
    across them --- with four agents it demands all four, and with ten it admits
    one. The aggregation zone is the ring the behavior targets, $k \cdot
    R_{ring}$ of (\ref{eq:ring}), dilated by a tolerance that absorbs the arrival
    threshold, the localization noise and the redistribution of slots near the
    wall; the resulting radii are $610$~mm for four agents and $900$~mm for ten.
    
    \item Swarm compactness: measures how disperse the swarm is at each instant of
    the experiment. Calculated as the root mean square of the euclidean distances
    from each robot to the swarm centroid, as defined in (\ref{eq:compaction}),
    (\ref{eq:centroid}) and (\ref{eq:avg_xy}), and normalized by the robot
    diameter. Averaging inside the root, rather than summing, is what makes the
    value comparable between swarms of different size: a plain sum grows with
    $\sqrt{N}$ and would make the ten-agent swarm appear less compact for no
    other reason than having more agents.
```

---

## 9. Results and analysis

**REEMPLAZAR la sección entera.** Sube `results_sim.pdf` y `trajectories_sim.pdf`.

```latex
\section{Results and analysis}

Group differences are assessed with the Kruskal--Wallis test rather than with a
one-way ANOVA. The design has three repetitions per configuration, the
distributions are skewed, and the dispersion is markedly uneven across
configurations --- a run in which one agent fails to cross produces a value far
outside the range of the others. Under those conditions the equal-variance and
normality assumptions of ANOVA do not hold, whereas a rank-based test remains
valid. Effects are reported as medians throughout, for the same reason.

\subsection{Simulation with ten agents}

The ten-agent simulation was run over the same five configurations with three
repetitions each, and every run converged. Table~\ref{tab:sim} and
Figure~\ref{fig:sim_boxplots} report the outcome.

\begin{table}[htbp]
\caption{Simulation results, ten agents. Aggregation time is the median over
three repetitions; compactness follows (\ref{eq:compaction}).}
\label{tab:sim}
\centering
\begin{tabular}{lccc}
\hline
\textbf{Configuration} & \textbf{Passage} & \textbf{Time (s)} & \textbf{Compactness} \\
\hline
No obstacles   & ---            & 17.1 & 5.56 \\
$2.1a$--$4d$   & $3.92\,d$      & 23.2 & 5.61 \\
$0.7a$--$4d$   & $3.81\,d$      & 29.3 & 5.82 \\
$0.7a$--$2d$   & $1.81\,d$      & 34.0 & 5.76 \\
$2.1a$--$2d$   & $2.00\,d$      & 49.2 & 6.13 \\
\hline
\end{tabular}
\end{table}

The clearest effect appears in the distance travelled, which has the advantage of
requiring no threshold and therefore of being independent of how the aggregation
zone is defined ($H = 34.5$, $p < 0.0001$ across the five configurations). It
separates into two contrasts. The presence of the barrier lengthens the paths,
from a median of $1.22$ times the straight-line distance in the obstacle-free
control to $1.53$ with obstacles ($p = 0.0012$). Narrowing the passage lengthens
them further, to $1.60$ for $2 \cdot d_{robot}$ gaps against $1.34$ for
$4 \cdot d_{robot}$ ($p < 0.0001$), while the obstacle size produces no
distinguishable difference.

\begin{figure}
    \centering
    \includegraphics[width=0.72\linewidth]{results_sim.pdf}
    \caption{Aggregation results of the ten-agent simulation, with the same three
    metrics and the same configuration ordering as Figure~\ref{fig:boxplots}.
    Top: distance travelled by each robot, normalized by the straight-line
    distance to its slot. Middle: aggregation time. Bottom: swarm compactness at
    the end of the episode, following (\ref{eq:compaction}).}
    \label{fig:sim_boxplots}
\end{figure}


The aggregation time shows the same ordering, growing monotonically as the
passage narrows ($H = 13.1$, $p = 0.011$), with the gap width again carrying the
effect ($43.6$~s median for $2 \cdot d_{robot}$ against $27.1$~s for
$4 \cdot d_{robot}$, $p = 0.0050$) and the obstacle size carrying none. This
result must be read with the
caveat that it depends on the radius chosen for the aggregation zone: a radius
too small discards runs in which one agent settles slightly outside, and a
radius too large is satisfied so early that the configurations become
indistinguishable. Over the interval in which every run converges and no run
converges trivially, the effect is significant, but the metric is not
threshold-free in the way the travelled distance is, and it is reported here as
supporting rather than primary evidence.

Swarm compactness at the end of the episode is the least discriminating of the
three metrics ($H = 8.9$, $p = 0.064$). This is expected: the behavior drives every agent
to an assigned slot on a ring whose radius is fixed by the swarm size, so once
the agents arrive the final geometry is imposed by the controller and not by the
scenario. Compactness is informative as a time series --- how quickly the swarm
tightens --- rather than as a final value.

\subsection{Physical experiments}

The same five configurations were run in the physical arena with four agents and
three repetitions each, the repetitions differing in the initial arrangement of
the swarm.

\textbf{What the two settings agree on is that obstacles matter, and both say so
significantly.} The presence of the barrier raises the median aggregation time
from $11.5$ to $32.1$~s in the laboratory ($p = 0.043$) and from $17.1$ to
$30.1$~s in simulation ($p = 0.0093$), and it lengthens the paths from $1.28$ to
$1.66$ times the straight-line distance in the laboratory ($p = 0.041$) and from
$1.22$ to $1.53$ in simulation ($p = 0.0012$). Four independent tests, two
metrics in two settings, all in the same direction.

\textbf{What the physical experiments do not resolve is the width of the
passage.} Narrowing the gaps changes the median aggregation time from $35.0$ to
$32.1$~s, which the test cannot separate from noise ($p = 1.00$), and the
travelled distance from $1.82$ to $1.48$, that is, in the direction opposite to
the simulation and again without significance ($p = 0.069$). The most likely
reason is the sample size: three repetitions per configuration leave the
between-run variability of a physical swarm --- marker occlusions, unit-to-unit
differences, the pose at which the random walk ends --- larger than the effect
being sought. A single run in which one agent never crossed contributes
normalized distances of $5.4$, $8.5$ and $13.4$, an order of magnitude above the
rest; removing it does not reverse the contrast, which indicates that the
absence of an effect is not the doing of one outlier.

Beyond the barrier contrast, the point estimates agree with the simulation on
features the simulation identifies as topological. The obstacle-free control is
the fastest condition in both settings, and the slowest configuration is the same
one in both, namely large obstacles with narrow gaps, which is also the
configuration with the smallest effective passage. The ordering of the five
configurations correlates between the two settings, with a rank correlation of
$\rho = 0.70$ for the aggregation time and $\rho = 0.60$ for the final
compactness; compactness reproduces both extremes, the obstacle-free control
yielding the tightest swarm and the large-obstacle narrow-gap configuration the
loosest.

\subsection{Effect of swarm size}

Taken together, the dependence on topology is not an unconditional property of
the aggregation behavior. It is statistically evident with ten agents and not
detectable with four, under the same passage widths, the same obstacle areas and
the same control software.

A congestion argument accounts for this. The passage acts as a shared resource,
and the cost it imposes grows with the number of agents that must traverse it
within the same episode: four agents crossing a $2.4$~m barrier queue very
little, while ten do. Under this reading the swarm size is not a scaling
parameter of the phenomenon but a precondition for it, and the quantity that
governs aggregation is not the passage width alone but its ratio to the demand
placed on it. The present design cannot separate this explanation from a simple
lack of statistical power at three repetitions per configuration, and doing so
--- by increasing either the number of repetitions or the size of the physical
swarm --- is the immediate continuation of this work.

\begin{figure}
    \centering
    \includegraphics[width=0.8\linewidth]{Fig1_Normalized_Distance.pdf} \\
    (a) \\
    \includegraphics[width=0.8\linewidth, trim= 48 0 0 0, clip ]{Fig2_Aggregation_Time.pdf} \\
    (b) \\
    \includegraphics[width=0.8\linewidth]{Fig3_Final_Compactness.pdf} \\
    (c)
    \caption{Validation results with physical robot swarm. (a) Normalized distance traveled by robots on each scenario configuration. (b) Final aggregation time for the robot swarm. (c) Swarm Compactness metric during validations.}
    \label{fig:boxplots}
\end{figure}
```

---
