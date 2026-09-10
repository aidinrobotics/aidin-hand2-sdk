[← C++ API Reference](../08_cpp_api_reference.md)

# `types/state.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [`HandLifecycle`](#enum-handlifecycle) | enum | The 5 states of a [`Hand`](hand.md#hand) |
| [`HomingState`](#enum-homingstate) | enum | Origin state |
| [`ActuatorFault`](#enum-actuatorfault) | enum | Drive error code |
| [`CommandSource`](#enum-commandsource) | enum | What the SDK used this cycle |
| [`HandState`](#handstate) | struct | What [`get_state()`](hand.md#handget_state) returns |
| [`ActuatorState`](#actuatorstate) | struct | Actuator observation |
| [`JointState`](#jointstate) | struct | FK output |
| [`TactileState`](#tactilestate) | struct | Tactile |
| [`CommandedState`](#commandedstate) | struct | This cycle's command input and output |
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
  None                     = 0x0000,  // healthy
  OverCurrentError         = 0x2310,  // electrical
  OverVoltageError         = 0x3210,  // electrical
  UnderVoltageError        = 0x3220,  // electrical
  OverTemperatureError     = 0x4210,  // thermal
  CurrentDetectionError    = 0x5210,  // electrical
  SpeedError               = 0x7310,  // control
  CommunicationError       = 0x7500,  // communication
  FollowingError           = 0x8611,  // control, tracking failed
  HallSensorError          = 0xFF01,  // hall sensor
  OverLoadError            = 0xFF02,  // mechanical load
  PositiveLimitSwitchError = 0xFF03,  // limit
  NegativeLimitSwitchError = 0xFF04,  // limit
  EmergencySwitchError     = 0xFF05,  // safety
  Sto1Error                = 0xFF06,  // safety (STO = Safe Torque Off)
  Sto2Error                = 0xFF07,  // safety (STO = Safe Torque Off)
  SerialEncoderChannelAError             = 0xFF24,  // encoder
  SerialEncoderChannelADisconnectedError = 0xFF25,  // encoder
  SerialEncoderChannelBError             = 0xFF26,  // encoder
  SerialEncoderChannelBDisconnectedError = 0xFF27,  // encoder
};

constexpr const char* to_string(ActuatorFault fault) noexcept;
```

The drive's `0x603F` error code. The SDK does not stop by itself when a fault appears, and indices
listed in `disabled_actuators` are never reported. What to check for each code is in
[Error messages](../15_error_messages.md#11-actuator-faults).

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
| `timestamp` | `std::int64_t` | That cycle's `CLOCK_REALTIME` epoch ns. `0` means there is no value yet |
| `actuators` | [`ActuatorState`](#actuatorstate) | Actuator observation |
| `joints` | [`JointState`](#jointstate) | FK output |
| `tactile` | [`TactileState`](#tactilestate) | Tactile |
| `commanded` | [`CommandedState`](#commandedstate) | This cycle's command input and output |

`timestamp` is for lining up with rosbag or a ROS `header.stamp`. Use
`Diagnostics::last_period_ms` to measure the period.

## `ActuatorState`

| Field | Type | Unit |
|---|---|---|
| `position_count` | `std::array<double, kActuatorCount>` | encoder count |
| `velocity_rpm` | `std::array<double, kActuatorCount>` | rpm |
| `current_mA` | `std::array<double, kActuatorCount>` | mA |

## `JointState`

| Field | Type | Unit | Status |
|---|---|---|---|
| `position_rad` | `std::array<double, kJointCount>` | rad | FK output |
| `velocity_rad_s` | `std::array<double, kJointCount>` | rad/s | `0` placeholder |
| `effort_Nm` | `std::array<double, kJointCount>` | N·m | `0` placeholder |

The `0` in velocity and effort is not a measurement.

## `TactileState`

| Field | Type | Description |
|---|---|---|
| `fingers` | `std::array<std::array<double, kTactileTaxelsPerFinger>, kFingerCount>` | Outer index is [`Finger`](types_description.md#enum-finger). Values are raw 16-bit counts |
| `palm` | `std::array<double, kPalmTactileCount>` | palm1 upper 20 + lower 20 + palm2 18 |

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
