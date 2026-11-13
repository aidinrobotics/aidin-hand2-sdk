[← C++ API Reference](../08_cpp_api_reference.md)

# `hand/hand_kinematics.hpp`

[`Hand`](hand.md#hand) 없이 쓰는 변환 함수입니다. 각도는 rad, actuator는 encoder count입니다. 첫 인자
`hand_model`에는 [`HandConfig`](types_config.md#handconfig)에 넣은 것과 같은 값을 넘깁니다.

| Symbol | Description |
|---|---|
| [`fk_actuator_to_joint()`](#fk_actuator_to_joint) | actuator encoder 16개 → joint 21개 |
| [`ik_joint_to_actuator()`](#ik_joint_to_actuator) | active joint 16개 → actuator encoder 16개 |

---

### `fk_actuator_to_joint()`

```cpp
std::array<double, kJointCount> fk_actuator_to_joint(
    HandModel hand_model, const std::array<int, kActuatorCount>& encoder);
```

encoder 값에서 joint 각도를 계산합니다. finger마다 4절 링크로 딸려 움직이는 passive joint `q4`가
하나씩 있고, 이 값도 결과에 포함됩니다.

**Parameters**     ｜ `hand_model` — 로봇 핸드 모델 · `encoder` — actuator encoder count 16개<br>
**Returns**        ｜ joint 각도 21개 [rad]

---

### `ik_joint_to_actuator()`

```cpp
std::array<int, kActuatorCount> ik_joint_to_actuator(
    HandModel hand_model, const std::array<double, kActiveJointCount>& active_joint);
```

active joint 각도에서 encoder 값을 계산합니다.

**Parameters**     ｜ `hand_model` — 로봇 핸드 모델 · `active_joint` — active joint 각도 16개 [rad]<br>
**Returns**        ｜ actuator encoder count 16개
