[← C++ API Reference](../08_cpp_api_reference.md)

# `types/state.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [`HandLifecycle`](#enum-handlifecycle) | enum | The 5 states of a [`Hand`](hand.md#hand) |
| [`HomingState`](#enum-homingstate) | enum | Origin state |
| [`ActuatorFault`](#enum-actuatorfault) | enum | Drive error code |
| [`CommandSource`](#enum-commandsource) | enum | What the SDK used this cycle |
| [`HandState`](#handstate) | struct | What [`get_state()`](hand.md#handget_state) returns |
| [`ActuatorState`](#actuatorstate) | struct | Position, velocity and current of the 16 actuators |
| [`JointState`](#jointstate) | struct | Angle, velocity and estimated torque of the 21 joints |
| [`TactileState`](#tactilestate) | struct | Tactile sensor values |
| [`CommandedState`](#commandedstate) | struct | Command input and output of this cycle |
| [`ActuatorHealth`](#actuatorhealth) | struct | Per-actuator enable and fault |
| [`ControllerOutput`](#type-aliases) | alias | Setpoint variant |

## enum `HandLifecycle`

```cpp
enum class HandLifecycle {
  Disconnected,   // CAN socket not open
  Connected,      // receiving frames, but no torque and no TX
  Running,        // under control
  Stopped,        // halted by stop()
  Faulted,        // halted by a communication error or a control and communication loop exception
};

constexpr const char* to_string(HandLifecycle lifecycle) noexcept;
```

[`Diagnostics::lifecycle`](types_diagnostics.md#diagnostics) is the observed value, not the value that you requested.
Transitions and calling rules are in the [C++ guide](../07_cpp_usage_guide.md#22-state-transition-calls).

## enum `HomingState`

```cpp
enum class HomingState {
  NotRun     = 0,  // not done yet: right after create, and after reconnect
  Succeeded  = 1,  // origin established. Only here do motion commands pass
  InProgress = 2,  // running
  Failed     = 3,  // failed
};

constexpr const char* to_string(HomingState state) noexcept;
```

Read through `Diagnostics::homing_state`.

## enum `ActuatorFault`

```cpp
enum class ActuatorFault : std::uint16_t {
  None                     = 0x0000,
  OverCurrentError         = 0x2310,
  OverVoltageError         = 0x3210,
  UnderVoltageError        = 0x3220,
  OverTemperatureError     = 0x4210,
  CurrentDetectionError    = 0x5210,
  SpeedError               = 0x7310,
  CommunicationError       = 0x7500,
  FollowingError           = 0x8611,
  HallSensorError          = 0xFF01,
  OverLoadError            = 0xFF02,
  PositiveLimitSwitchError = 0xFF03,
  NegativeLimitSwitchError = 0xFF04,
  EmergencySwitchError     = 0xFF05,
  Sto1Error                = 0xFF06,
  Sto2Error                = 0xFF07,
  SerialEncoderChannelAError             = 0xFF24,
  SerialEncoderChannelADisconnectedError = 0xFF25,
  SerialEncoderChannelBError             = 0xFF26,
  SerialEncoderChannelBDisconnectedError = 0xFF27,
};

constexpr const char* to_string(ActuatorFault fault) noexcept;
```

The drive's `0x603F` error code. The SDK does not stop by itself when a fault appears, and indices
listed in `disabled_actuators` are never reported. The log line and what to do when a fault is
reported are in [E504 in Error messages](../15_error_messages.md#6-e5xx-hardwarefault).

## enum `CommandSource`

```cpp
enum class CommandSource {
  None,        // no output. Receiving only, nothing transmitted
  Controller,  // controller output
  QuickStop,   // quick stop
  Homing,      // homing
};
```

## `HandState`

| Field | Type | Description |
|---|---|---|
| `timestamp` | `std::int64_t` | The time of that cycle |
| `actuators` | [`ActuatorState`](#actuatorstate) | Position, velocity and current of the 16 actuators |
| `joints` | [`JointState`](#jointstate) | Angle, velocity and estimated torque of the 21 joints |
| `tactile` | [`TactileState`](#tactilestate) | Tactile sensor values |
| `commanded` | [`CommandedState`](#commandedstate) | Command input and output of this cycle |

`timestamp` is in `CLOCK_REALTIME` epoch ns, and `0` means there is no value yet. It is for lining
up with rosbag or a ROS `header.stamp`. Use `Diagnostics::last_period_ms` to measure the period.

## `ActuatorState`

| Field | Type | Unit |
|---|---|---|
| `position_count` | `std::array<std::int32_t, kActuatorCount>` | encoder count |
| `velocity_rpm` | `std::array<std::int32_t, kActuatorCount>` | rpm |
| `current_mA` | `std::array<std::int16_t, kActuatorCount>` | mA |

## `JointState`

| Field | Type | Unit | Description |
|---|---|---|---|
| `position_rad` | `std::array<double, kJointCount>` | rad | Angles of the 21 joints |
| `velocity_rad_s` | `std::array<double, kJointCount>` | rad/s | Velocities of the 21 joints, the passive joint4 included |
| `effort_Nm` | `std::array<double, kJointCount>` | N·m | Estimated torque of the active joints. NaN on joint4 |

`effort_Nm` is the joint torque estimated from the actuator current (`current_mA`) with the motor
torque constant. It leaves out transmission losses and friction, so it can differ from the actual
joint torque. The passive joint4 has no effort and reads NaN.

## `TactileState`

| Field | Type | Description |
|---|---|---|
| `fingers` | `std::array<std::array<std::int32_t, kTactileTaxelsPerFinger>, kFingerCount>` | Outer index is [`Finger`](types_description.md#enum-finger) |
| `palm` | `std::array<std::int32_t, kPalmTactileCount>` | palm1 upper 20 + lower 20 + palm2 18 |

The values are the raw values of the sensors. After a bias is set with
[`Hand::set_tactile_bias()`](hand.md#handset_tactile_bias), the values are the difference from
the bias and can be negative.
[`Hand::get_tactile_bias()`](hand.md#handget_tactile_bias) returns the bias in this type too.

## `CommandedState`

| Field | Type | Description |
|---|---|---|
| `controller_input` | [`ControllerCommand`](types_command.md#type-aliases) | The command currently in effect |
| `controller_output` | [`ControllerOutput`](#type-aliases) | The actuator position or effort setpoint past the limits and the fault mask |
| `selected_source` | [`CommandSource`](#enum-commandsource) | What was actually used this cycle |
| `max_effort_pct` | `std::array<double, kActuatorCount>` | The per-actuator ceiling currently in effect |

`controller_output` alone does not tell you the drives received it. How to judge that is in the
[C++ guide](../07_cpp_usage_guide.md#72-applied-command). Read it together with `selected_source`.

### `ActuatorPositionSetpoint` / `ActuatorEffortSetpoint`

| Owning type | Field | Type | Unit |
|---|---|---|---|
| `ActuatorPositionSetpoint` | `target_position_cnt` | `std::array<double, kActuatorCount>` | encoder count |
| `ActuatorEffortSetpoint` | `target_effort_pct` | `std::array<double, kActuatorCount>` | % of rated current |

## `ActuatorHealth`

| Field | Type | Description |
|---|---|---|
| `enabled` | `std::array<bool, kActuatorCount>` | Whether the drive is enabled |
| `fault` | `std::array<ActuatorFault, kActuatorCount>` | Drive error code |

## Type aliases

```cpp
using ControllerOutput = std::variant<std::monostate, ActuatorPositionSetpoint, ActuatorEffortSetpoint>;
```

`std::monostate` means there is no setpoint because the hand is outside a control session
(`Running`, `Stopped`, `Faulted`).

---
