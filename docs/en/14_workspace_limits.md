# Workspace limits

The SDK clamps joint-space commands ([`JointPositionCommand`](08_cpp_api_reference/types_command.md#jointpositioncommand),
[`JointImpedanceCommand`](08_cpp_api_reference/types_command.md#jointimpedancecommand)) into the reachable
workspace of each finger before IK. The SDK applies the range limits and the projection to the
nearest reachable pose on its own, so sending a command needs nothing more. This document covers
those ranges and how the clamp works.

To compute the same limits yourself before sending a command, for example to settle a target in a
higher-level controller, implement them from the formulas and tables in this document.

## Contents

&nbsp;&nbsp;[**1. Joint limits**](#1-joint-limits)<br>
&nbsp;&nbsp;[**2. Thumb**](#2-thumb)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.1 joint0](#21-joint0)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.2 joint1 and joint2](#22-joint1-and-joint2)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.3 joint3](#23-joint3)<br>
&nbsp;&nbsp;[**3. Long finger**](#3-long-finger)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.1 joint1 and joint2](#31-joint1-and-joint2)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[3.2 joint3](#32-joint3)

## 1. Joint limits

The allowed range of each joint, finger by finger.

Long fingers (index / middle / ring / baby).

| Joint | Axis | Allowed range |
| :--- | :--- | :--- |
| `joint1` | MCP abduction/adduction | [−31°, 31°] (coupled) |
| `joint2` | MCP flexion/extension | [0°, 96.2°] (coupled) |
| `joint3` | PIP flexion/extension | [0°, 80°] (lower bound depends on `joint1` and `joint2`) |
| `joint4` | DIP flexion/extension | [0°, 90.6°] (passive) |

Thumb. The CMC has 3 axes, so `joint0`, `joint1` and `joint2` each take a different one; unlike the
long fingers, `joint1` is flexion and `joint2` is abduction.

| Joint | Axis | Allowed range |
| :--- | :--- | :--- |
| `joint0` | CMC rotation | [0°, 108.8°] |
| `joint1` | CMC flexion/extension | [0°, 59°] (coupled) |
| `joint2` | CMC abduction/adduction | [−44.5°, 44.5°] (coupled) |
| `joint3` | MCP flexion/extension | [0°, 75°] |
| `joint4` | IP flexion/extension | [0°, 84.8°] (passive) |

The notes in parentheses mean the following.

- **(coupled)**: The two joints (abduction, flexion) share one workspace. The value in the table is
  that pair's maximum reach; the effective maximum shrinks with the angle of the other joint. The
  boundaries are in [2.2 joint1 and joint2](#22-joint1-and-joint2) and
  [3.1 joint1 and joint2](#31-joint1-and-joint2).
- **(lower bound depends on `joint1` and `joint2`)**: The lower bound of the long finger `joint3`
  varies between 0° and 5.7° with `joint1` and `joint2`; the upper bound of 80° does not depend on
  the pose. The formula is in [3.2 joint3](#32-joint3).
- **(passive)**: A joint command's `target` (16 entries) has no slot for `joint4`, so no command sets
  it; its angle depends on `joint3`. Read its current angle from `position_rad` in
  [`JointState`](08_cpp_api_reference/types_state.md#jointstate). Its range in the tables is the
  `joint4` angle with `joint3` at 0° and at its upper bound (80° for the long fingers, 75° for the
  thumb).

The thumb `joint0` and `joint3`, which carry no note, keep the ranges in the table regardless of
the other joints.

## 2. Thumb

### 2.1 joint0

`joint0` (CMC rotation) ranges over [0°, 108.8°], independent of the other joints. The SDK
clamps a command outside the range into it.

### 2.2 joint1 and joint2

This section writes the `joint1` (CMC flexion/extension) angle as $x$ and the
`joint2` (CMC abduction/adduction) angle as $y$. The two joints share one workspace. $x$ ranges over [0°, 59°], and $y$ over
$[-y_{\max}(x),\ y_{\max}(x)]$. The boundary is symmetric in the sign of $y$, so the figure shows only
$y$ of 0 and above.

![Thumb workspace and clamp projection](../assets/workspace_clamp_thumb.webp)

$y_{\max}$ consists of several segments, each defined between two points

```math
\mathbf p_0 = (x_0,\ y_0), \qquad \mathbf p_1 = (x_1,\ y_1)
```

as a line or an arc. Each segment is therefore valid over $x \in [x_0,\ x_1]$.

A line segment is the linear interpolation between the two points; an arc segment is defined by its
circle center $\mathbf c = (x_c,\ y_c)$ and a signed radius $R$.

```math
y_{\max}(x) =
\begin{cases}
y_0 + \dfrac{x - x_0}{x_1 - x_0}\,(y_1 - y_0) & \text{Line} \\[2ex]
y_c + R\sqrt{1 - \left(\dfrac{x - x_c}{R}\right)^2} & \text{Arc}
\end{cases}
```

$\mathbf c$ follows from the two points and $R$; the table below gives it to four decimal places.

The thumb $y_{\max}$ consists of the following 4 segments.

| Type | $\mathbf p_0$ | $\mathbf p_1$ | $\mathbf c$ | $R$ |
| :--- | ---: | ---: | ---: | ---: |
| Arc | (0, 0) | (9.4, 29.5) | (−423.8055, 151.2907) | −450 |
| Arc | (9.4, 29.5) | (48, 44.5) | (−61.5422, 269.2232) | −250 |
| Arc | (48, 44.5) | (58.4, 39.43) | (27.0304, −11.7163) | 60 |
| Line | (58.4, 39.43) | (59, 0) | — | — |

The SDK clamps a negative $x$ to 0°. When $(x, |y|)$ lies inside the workspace, it leaves the command
as it is. Otherwise it moves the command to the nearest point on the boundary in Euclidean distance
(the red arrows in the figure) and restores the sign of $y$. Both $x$ and $y$ can change.

### 2.3 joint3

`joint3` (MCP flexion/extension) ranges over [0°, 75°], independent of the other joints. The
SDK clamps a command outside the range into it.

## 3. Long finger

### 3.1 joint1 and joint2

This section writes the `joint2` (MCP flexion/extension) angle as $x$ and the `joint1`
(MCP abduction/adduction) angle as $y$. The two joints share one workspace. $x$ ranges over
[0°, 96.2°], and $y$ over $[-y_{\max}(x),\ y_{\max}(x)]$. The boundary is symmetric in the sign of
$y$, so the figure shows only $y$ of 0 and above.

![Long finger workspace and clamp projection](../assets/workspace_clamp_long.webp)

$y_{\max}$ consists of several segments, each defined between two points

```math
\mathbf p_0 = (x_0,\ y_0), \qquad \mathbf p_1 = (x_1,\ y_1)
```

as a line or an arc. Each segment is therefore valid over $x \in [x_0,\ x_1]$.

A line segment is the linear interpolation between the two points; an arc segment is defined by its
circle center $\mathbf c = (x_c,\ y_c)$ and a signed radius $R$.

```math
y_{\max}(x) =
\begin{cases}
y_0 + \dfrac{x - x_0}{x_1 - x_0}\,(y_1 - y_0) & \text{Line} \\[2ex]
y_c + R\sqrt{1 - \left(\dfrac{x - x_c}{R}\right)^2} & \text{Arc}
\end{cases}
```

$\mathbf c$ follows from the two points and $R$; the table below gives it to four decimal places.

The long finger $y_{\max}$ consists of the following 5 segments.

| Type | $\mathbf p_0$ | $\mathbf p_1$ | $\mathbf c$ | $R$ |
| :--- | ---: | ---: | ---: | ---: |
| Arc | (0, 0) | (11.84, 31) | (−142.7432, 72.2798) | −160 |
| Line | (11.84, 31) | (48.5, 31) | — | — |
| Arc | (48.5, 31) | (87.5, 18.5) | (104.0890, 137.3478) | −120 |
| Arc | (87.5, 18.5) | (93.2, 12.4) | (76.0587, 2.0958) | 20 |
| Arc | (93.2, 12.4) | (96.2, 0) | (121.1992, 12.6111) | −28 |

The SDK clamps a negative $x$ to 0°. When $(x, |y|)$ lies inside the workspace, it leaves the command
as it is. Otherwise it moves the command to the nearest point on the boundary in Euclidean distance
(the red arrows in the figure) and restores the sign of $y$. Both $x$ and $y$ can change.

### 3.2 joint3

This section writes the `joint3` (PIP flexion/extension) angle as $z$. `joint3` ranges over
$[z_{\min}(x, y),\ 80°]$. The upper bound of 80° does not depend on the pose, and the lower bound
$z_{\min}$ is a function of the $x$ and $y$ of section 3.1. The SDK clamps a command outside the range
into it, taking $z_{\min}$ at $(x, y)$ after the clamp to the section 3.1 boundary.

All angles in the formulas below are in radians.

```math
z_{\min}(x, y) = \max\bigl(0,\ \varphi(x, y)\bigr)
```

$\varphi(x, y)$ is defined as follows.

```math
\begin{aligned}
\beta &= a_0 - x \\[1ex]
\lambda &= a_1 + x + \mathrm{atan2}\bigl(\sin\beta + a_2\cos y,\ \cos\beta + a_3\bigr)
  - \arccos\frac{a_4 + a_5\sin\beta\cos y + a_6\cos\beta}
               {\sqrt{a_7 + a_8\cos^2 y + a_9\sin\beta\cos y + a_{10}\cos\beta}} \\[1ex]
\varphi &= b_0 - \mathrm{atan2}\bigl(\sin\lambda + b_1,\ \cos\lambda + b_2\bigr)
  + \arccos\frac{b_3 + b_4\cos\lambda + b_5\sin\lambda}
               {\sqrt{b_6 + b_7\cos\lambda + b_8\sin\lambda}}
\end{aligned}
```

The constants are below.

| $i$ | $a_i$ | $b_i$ |
| ---: | ---: | ---: |
| 0 | 0.819728 | −3.26693 |
| 1 | 1.79827 | −2.91243 |
| 2 | 0.287368 | −1.56229 |
| 3 | 0.142105 | −2.13874 |
| 4 | 0.371072 | 2.10835 |
| 5 | 0.34125 | 3.93041 |
| 6 | 0.16875 | 11.923 |
| 7 | 1.02019 | −3.12458 |
| 8 | 0.0825804 | −5.82487 |
| 9 | 0.574736 | — |
| 10 | 0.28421 | — |

Inside the workspace $z_{\min}$ ranges from 0 to 0.0995 rad (0° to 5.7°). At $y = 0$,
$\varphi \le 0$ from $x$ = 1.3044 rad (74.7°) on, and the lower bound is 0.

![Long finger joint3 lower bound](../assets/joint3_lower_bound.webp)
