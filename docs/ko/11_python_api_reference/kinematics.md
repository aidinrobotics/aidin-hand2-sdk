[← Python API Reference](../11_python_api_reference.md)

# Kinematics

[`Hand`](hand.md#hand) 없이 쓰는 변환 함수입니다. 각도는 rad, actuator는 encoder count입니다.

| Symbol | Description |
|---|---|
| [`fk_actuator_to_joint()`](#fk_actuator_to_joint) | actuator encoder 16개 → joint 21개 |
| [`ik_joint_to_actuator()`](#ik_joint_to_actuator) | active joint 16개 → actuator encoder 16개 |

---

### `fk_actuator_to_joint()`

```python
def fk_actuator_to_joint(encoder: ArrayLike) -> numpy.ndarray: ...
```

encoder 값에서 joint 각도를 계산합니다. finger마다 4절 링크로 딸려 움직이는 passive joint `q4`가
하나씩 있고, 이 값도 결과에 포함됩니다.

**Parameters**     ｜ `encoder` — actuator encoder count 16개. 정수<br>
**Returns**        ｜ joint 각도 21개 [rad]. `float64` `(21,)`<br>
**Raises**         ｜ `ValueError` — 길이가 16이 아닐 때 · `TypeError` — 정수가 아닌 값<br>
**Notes**          ｜ `HandState.actuators.position_count`와 `ik_joint_to_actuator()`의 결과를 그대로
넘길 수 있습니다

---

### `ik_joint_to_actuator()`

```python
def ik_joint_to_actuator(active_joint: ArrayLike) -> numpy.ndarray: ...
```

active joint 각도에서 encoder 값을 계산합니다.

**Parameters**     ｜ `active_joint` — active joint 각도 16개 [rad]<br>
**Returns**        ｜ actuator encoder count 16개. `int32` `(16,)`<br>
**Raises**         ｜ `ValueError` — 길이가 16이 아닐 때<br>
**Notes**          ｜ joint command의 `target`을 그대로 넘길 수 있습니다. 결과를
[`ActuatorPositionCommand`](command.md#actuatorpositioncommand)에 넘기면 같은 자세가 됩니다
