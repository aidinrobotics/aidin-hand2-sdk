# Workspace limits

joint 공간 명령([`JointPositionCommand`](08_cpp_api_reference/types_command.md#jointpositioncommand), [`JointImpedanceCommand`](08_cpp_api_reference/types_command.md#jointimpedancecommand))은 IK 전에 각 finger의 도달 가능
workspace 안으로 투영됩니다. 그 범위와 투영 방식을 아래에서 다룹니다.

## Contents

&nbsp;&nbsp;[**1. Joint limits**](#1-joint-limits)<br>
&nbsp;&nbsp;[**2. Coupled abduction–flexion workspace**](#2-coupled-abductionflexion-workspace)<br>
&nbsp;&nbsp;[**3. Clamping behavior**](#3-clamping-behavior)

## 1. Joint limits

아래 표의 값은 degree입니다. joint command의 `target` 필드는 rad이므로 `deg * M_PI / 180.0`으로
바꿔 비교하십시오.

long finger(index / middle / ring / baby)입니다.

| Joint | Axis | Allowed range |
| :--- | :--- | :--- |
| `joint1` | abduction | [−27.5°, 27.5°] (coupled) |
| `joint2` | flexion | [0°, 95.46°] (coupled) |
| `joint3` | distal | [0°, 90°] |

thumb입니다. CMC(손목손허리관절)가 3축이라 `joint0`·`joint1`·`joint2`가 각각 다른 축을 맡습니다.

| Joint | Axis | Allowed range |
| :--- | :--- | :--- |
| `thumb_joint0` | CMC rotation | [0°, 110°] |
| `thumb_joint1` | CMC flexion | [0°, 76.23°] (coupled) |
| `thumb_joint2` | CMC abduction | [−45.1°, 45.1°] (coupled) |
| `thumb_joint3` | distal | [0°, 70°] |

thumb은 long finger와 축 순서가 반대입니다 — long finger는 `joint1`이 abduction, `joint2`가
flexion입니다.

`joint0`과 `joint3`은 coupled workspace에 속하지 않습니다. 두 joint는 상대 축과 무관하게 표의
범위를 그대로 씁니다.

`(coupled)`로 표시된 abduction과 flexion은 하나의 workspace를 공유합니다. 표의 값은 그 쌍의 최대
도달값이고, 실제로 허용되는 최대값은 상대 축의 각도에 따라 작아집니다.

## 2. Coupled abduction–flexion workspace

workspace는 (flexion, abduction) 평면의 도달 가능 영역입니다. 아래에서 경계를 정의합니다.

![도달 가능 workspace와 clamp 투영](../assets/workspace_clamp.webp)

| Mark | Meaning |
|---|---|
| 파란 선 | workspace 경계 |
| 파란 점 | segment의 끝점 |
| 회색 점 | 측정 데이터 |
| 주황 ✕ | workspace 밖의 명령 목표 |
| 주황 ○ | 투영 후 손에 전달되는 값 |

long finger 4개는 같은 경계를 공유하고 thumb은 별도입니다. 경계는 abduction 부호에 대해
대칭입니다.

flexion $x$에 따른 최대 허용 abduction을 $y_{\max}(x)$로 표기합니다. $x$와 $y$의 단위는 모두
degree입니다. $y_{\max}$는 여러 개의 segment로 구성되며, 각 segment는 두 점

$$
\mathbf p_0 = (x_0,\ y_0), \qquad \mathbf p_1 = (x_1,\ y_1)
$$

사이에서 정의되는 line 또는 arc입니다. 따라서 각 segment의 유효 범위는 $x \in [x_0,\ x_1]$입니다.

line segment는 두 점 사이의 선형 보간이고, arc segment는 원의 중심 $\mathbf c = (x_c,\ y_c)$와
signed radius $R$로 정의됩니다.

$$
y_{\max}(x) =
\begin{cases}
y_0 + \dfrac{x - x_0}{x_1 - x_0}\,(y_1 - y_0) & \text{Line} \\[2ex]
y_c + \operatorname{sgn}(R)\sqrt{R^2 - (x - x_c)^2} & \text{Arc}
\end{cases}
$$

$|R|$은 원의 반지름이고, $R$의 부호가 사용할 arc branch를 결정합니다. $R > 0$이면
$y_{\max}(x) \ge y_c$, $R < 0$이면 $y_{\max}(x) \le y_c$입니다.

long finger의 $y_{\max}$는 다음 4개 segment로 구성됩니다.

| Type | $\mathbf p_0$ | $\mathbf p_1$ | $\mathbf c$ | $R$ |
| :--- | ---: | ---: | ---: | ---: |
| Arc | (0, 0.4) | (10.6, 27.5) | (−117.5448848530, 62.0000287617) | −132.7078125000 |
| Line | (10.6, 27.5) | (54.6, 27.5) | — | — |
| Arc | (54.6, 27.5) | (92.68, 13.25) | (−71.1198326410, −366.4639071557) | 413.5372250000 |
| Arc | (92.68, 13.25) | (95.46, 0) | (143.6789848190, 17.0335266262) | −51.1391388889 |

thumb의 $y_{\max}$는 다음 4개 arc segment로 구성됩니다.

| Type | $\mathbf p_0$ | $\mathbf p_1$ | $\mathbf c$ | $R$ |
| :--- | ---: | ---: | ---: | ---: |
| Arc | (0, 0.6) | (9, 29.6) | (−425.8806321979, 148.6664030959) | −450.8857644759 |
| Arc | (9, 29.6) | (50, 45.1) | (−55.2563274548, 261.5441564934) | −240.6802180268 |
| Arc | (50, 45.1) | (65, 37.3) | (30.1694515845, −11.3587469529) | 59.8401266539 |
| Arc | (65, 37.3) | (76.23, 0) | (251.7767940919, 73.1928136100) | −190.1942819332 |

## 3. Clamping behavior

`joint0`과 `joint3`은 1절 표의 고정 범위로 각각 투영됩니다. flexion 연동은 반영하지 않습니다.

`(coupled)` 쌍은 (flexion, |abduction|) 조합이 workspace 안에 있는지에 따라 갈립니다.

- 안 → 그대로 통과
- 밖 → 유클리드 거리 기준으로 가장 가까운 도달 가능 자세로 대체 (2절 그림의 주황 ✕ → ○)

abduction은 대칭이므로 |abduction|으로 투영해 검사한 뒤 부호를 복원합니다.
