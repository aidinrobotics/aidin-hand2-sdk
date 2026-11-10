[← C++ API Reference](../08_cpp_api_reference.md)

# `hand/hand_kinematics.hpp`

[`Hand`](hand.md#hand) 없이 쓰는 변환 함수입니다. 각도는 rad, actuator는 encoder count입니다.

| Symbol | Description |
|---|---|
| [`fk_actuator_to_joint()`](#fk_actuator_to_joint) | actuator encoder 16개 → joint 21개 |
| [`ik_joint_to_actuator()`](#ik_joint_to_actuator) | active joint 16개 → actuator encoder 16개 |
| [`compute_joint_vel_eff()`](#compute_joint_vel_eff) | `HandState` 하나 → active joint 16개의 속도와 effort |

---

### `fk_actuator_to_joint()`

```cpp
std::array<double, kJointCount> fk_actuator_to_joint(
    const std::array<int, kActuatorCount>& encoder);
```

encoder 값에서 joint 각도를 계산합니다. finger마다 4절 링크로 딸려 움직이는 passive joint `q4`가
하나씩 있고, 이 값도 결과에 포함됩니다.

**Parameters**     ｜ `encoder` — actuator encoder count 16개<br>
**Returns**        ｜ joint 각도 21개 [rad]

---

### `ik_joint_to_actuator()`

```cpp
std::array<int, kActuatorCount> ik_joint_to_actuator(
    const std::array<double, kActiveJointCount>& active_joint);
```

active joint 각도에서 encoder 값을 계산합니다.

**Parameters**     ｜ `active_joint` — active joint 각도 16개 [rad]<br>
**Returns**        ｜ actuator encoder count 16개

---

### `compute_joint_vel_eff()`

```cpp
HandStateJointVelEff compute_joint_vel_eff(const HandState& state);
```

`HandState`의 actuator 속도·전류·effort setpoint를 그 자세의 kinematics로 active joint 16개의 값으로
바꿉니다. effort는 모터 torque 상수 3.62 mN·m/A에 전류를 곱한 값이고 감속기 손실은 포함하지
않습니다.

**Parameters**     ｜ `state` — [`Hand::get_state()`](hand.md#handget_state)의 결과<br>
**Returns**        ｜ [`HandStateJointVelEff`](types_state.md#handstatejointveleff). hand type C에서는 모두 NaN
