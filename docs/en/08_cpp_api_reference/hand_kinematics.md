[← C++ API Reference](../08_cpp_api_reference.md)

# `hand/hand_kinematics.hpp`

Conversion functions that work without a [`Hand`](hand.md#hand). Angles are in rad, actuators in encoder counts.

| Symbol | Description |
|---|---|
| [`fk_actuator_to_joint()`](#fk_actuator_to_joint) | 16 actuator encoders → 21 joints |
| [`ik_joint_to_actuator()`](#ik_joint_to_actuator) | 16 active joints → 16 actuator encoders |
| [`compute_joint_vel_eff()`](#compute_joint_vel_eff) | one `HandState` → velocity and effort of 16 active joints |

### `fk_actuator_to_joint()`

```cpp
std::array<double, kJointCount> fk_actuator_to_joint(
    const std::array<int, kActuatorCount>& encoder);
```

Computes joint angles from encoder values. Each finger has one passive joint `q4` carried by its
four-bar linkage, and the result includes it.

**Parameters**     ｜ `encoder` — 16 actuator encoder counts<br>
**Returns**        ｜ 21 joint angles [rad]

### `ik_joint_to_actuator()`

```cpp
std::array<int, kActuatorCount> ik_joint_to_actuator(
    const std::array<double, kActiveJointCount>& active_joint);
```

Computes encoder values from active joint angles.

**Parameters**     ｜ `active_joint` — 16 active joint angles [rad]<br>
**Returns**        ｜ 16 actuator encoder counts

### `compute_joint_vel_eff()`

```cpp
HandStateJointVelEff compute_joint_vel_eff(const HandState& state);
```

Converts the actuator velocity, current and effort setpoint in a `HandState` into values of the 16
active joints through the kinematics at that posture. The effort is the motor torque constant,
3.62 mN·m/A, times the current, and it does not include the gear losses.

**Parameters**     ｜ `state` — what [`Hand::get_state()`](hand.md#handget_state) returned<br>
**Returns**        ｜ [`HandStateJointVelEff`](types_state.md#handstatejointveleff). All NaN on hand type C

---
