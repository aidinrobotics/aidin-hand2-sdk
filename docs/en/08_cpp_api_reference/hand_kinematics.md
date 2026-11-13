[← C++ API Reference](../08_cpp_api_reference.md)

# `hand/hand_kinematics.hpp`

Conversion functions that work without a [`Hand`](hand.md#hand). Angles are in rad, actuators in encoder counts.
The first argument, `hand_model`, is the value you put into [`HandConfig`](types_config.md#handconfig).

| Symbol | Description |
|---|---|
| [`fk_actuator_to_joint()`](#fk_actuator_to_joint) | 16 actuator encoders → 21 joints |
| [`ik_joint_to_actuator()`](#ik_joint_to_actuator) | 16 active joints → 16 actuator encoders |

### `fk_actuator_to_joint()`

```cpp
std::array<double, kJointCount> fk_actuator_to_joint(
    HandModel hand_model, const std::array<int, kActuatorCount>& encoder);
```

Computes joint angles from encoder values. Each finger has one passive joint `q4` carried by its
four-bar linkage, and the result includes it.

**Parameters**     ｜ `hand_model` — the robot hand's model · `encoder` — 16 actuator encoder counts<br>
**Returns**        ｜ 21 joint angles [rad]

### `ik_joint_to_actuator()`

```cpp
std::array<int, kActuatorCount> ik_joint_to_actuator(
    HandModel hand_model, const std::array<double, kActiveJointCount>& active_joint);
```

Computes encoder values from active joint angles.

**Parameters**     ｜ `hand_model` — the robot hand's model · `active_joint` — 16 active joint angles [rad]<br>
**Returns**        ｜ 16 actuator encoder counts
