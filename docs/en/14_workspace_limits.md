# Workspace limits

The SDK clamps joint-space commands ([`JointPositionCommand`](08_cpp_api_reference/types_command.md#jointpositioncommand),
[`JointImpedanceCommand`](08_cpp_api_reference/types_command.md#jointimpedancecommand)) into the reachable
workspace of each finger before IK. This document covers that range and how the clamp works.

## Contents

&nbsp;&nbsp;[**1. Joint limits**](#1-joint-limits)<br>
&nbsp;&nbsp;[**2. Coupled abduction–flexion workspace**](#2-coupled-abductionflexion-workspace)<br>
&nbsp;&nbsp;[**3. Clamping behavior**](#3-clamping-behavior)

## 1. Joint limits

The tables below are in degrees, while a joint command's `target` is in radians. Convert with
`deg * M_PI / 180.0` before comparing.

Long fingers (index / middle / ring / baby).

| Joint | Axis | Allowed range |
| :--- | :--- | :--- |
| `joint1` | abduction | [−27.5°, 27.5°] (coupled) |
| `joint2` | flexion | [0°, 95.46°] (coupled) |
| `joint3` | distal | [0°, 90°] |

Thumb. The CMC has 3 axes, so `joint0`, `joint1` and `joint2` each take a different one.

| Joint | Axis | Allowed range |
| :--- | :--- | :--- |
| `thumb_joint0` | CMC rotation | [0°, 110°] |
| `thumb_joint1` | CMC flexion | [0°, 76.23°] (coupled) |
| `thumb_joint2` | CMC abduction | [−45.1°, 45.1°] (coupled) |
| `thumb_joint3` | distal | [0°, 70°] |

The thumb's axis order is the reverse of the long fingers, where `joint1` is abduction and
`joint2` is
flexion.

`joint0` and `joint3` are not part of the coupled workspace. Both keep the ranges in the tables
regardless of the other axes.

The abduction and flexion marked `(coupled)` share one workspace. The values in the tables are that
pair's maximum reach; the effective maximum shrinks with the angle of the other axis.

## 2. Coupled abduction–flexion workspace

The workspace is the reachable region in the (flexion, abduction) plane. The sections below define
the boundary.

![Reachable workspace and clamp projection](../assets/workspace_clamp.webp)

| Mark | Meaning |
|---|---|
| Blue line | Workspace boundary |
| Blue dot | Segment endpoint |
| Grey dot | Measurement data |
| Orange ✕ | Commanded target outside the workspace |
| Orange ○ | The value that the SDK sends to the robot hand after the projection |

The 4 long fingers share the same boundary and the thumb has its own. The boundary is symmetric in
the sign of abduction.

The maximum allowed abduction at flexion $x$ is written $y_{\max}(x)$, with both $x$ and $y$ in
degrees. $y_{\max}$ consists of several segments, each defined between two points

$$
\mathbf p_0 = (x_0,\ y_0), \qquad \mathbf p_1 = (x_1,\ y_1)
$$

as a line or an arc. Each segment is therefore valid over $x \in [x_0,\ x_1]$.

A line segment is the linear interpolation between the two points; an arc segment is defined by its
circle center $\mathbf c = (x_c,\ y_c)$ and a signed radius $R$.

$$
y_{\max}(x) =
\begin{cases}
y_0 + \dfrac{x - x_0}{x_1 - x_0}\,(y_1 - y_0) & \text{Line} \\[2ex]
y_c + \operatorname{sgn}(R)\sqrt{R^2 - (x - x_c)^2} & \text{Arc}
\end{cases}
$$

$|R|$ is the circle radius, and the sign of $R$ selects the arc branch: $R > 0$ gives
$y_{\max}(x) \ge y_c$, and $R < 0$ gives $y_{\max}(x) \le y_c$.

The long finger $y_{\max}$ consists of the following 4 segments.

| Type | $\mathbf p_0$ | $\mathbf p_1$ | $\mathbf c$ | $R$ |
| :--- | ---: | ---: | ---: | ---: |
| Arc | (0, 0.4) | (10.6, 27.5) | (−117.5448848530, 62.0000287617) | −132.7078125000 |
| Line | (10.6, 27.5) | (54.6, 27.5) | — | — |
| Arc | (54.6, 27.5) | (92.68, 13.25) | (−71.1198326410, −366.4639071557) | 413.5372250000 |
| Arc | (92.68, 13.25) | (95.46, 0) | (143.6789848190, 17.0335266262) | −51.1391388889 |

The thumb $y_{\max}$ consists of the following 4 arc segments.

| Type | $\mathbf p_0$ | $\mathbf p_1$ | $\mathbf c$ | $R$ |
| :--- | ---: | ---: | ---: | ---: |
| Arc | (0, 0.6) | (9, 29.6) | (−425.8806321979, 148.6664030959) | −450.8857644759 |
| Arc | (9, 29.6) | (50, 45.1) | (−55.2563274548, 261.5441564934) | −240.6802180268 |
| Arc | (50, 45.1) | (65, 37.3) | (30.1694515845, −11.3587469529) | 59.8401266539 |
| Arc | (65, 37.3) | (76.23, 0) | (251.7767940919, 73.1928136100) | −190.1942819332 |

## 3. Clamping behavior

`joint0` and `joint3` are each clamped to the fixed ranges in the section 1 tables; the flexion
coupling is not applied to them.

For a `(coupled)` pair, what happens depends on whether the (flexion, |abduction|) combination
lies inside the workspace.

- Inside → passes through unchanged
- Outside → replaced by the nearest reachable pose in Euclidean distance (the orange ✕ moving to
  the orange ○ in the section 2 figure)

Abduction is symmetric, so the SDK checks against |abduction| and then restores the sign.
