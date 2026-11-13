[← C++ API Reference](../08_cpp_api_reference.md)

# `types/state.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [`HandLifecycle`](#enum-handlifecycle) | enum | [`Hand`](hand.md#hand)의 상태 5개 |
| [`HomingState`](#enum-homingstate) | enum | 원점 상태 |
| [`ActuatorFault`](#enum-actuatorfault) | enum | actuator error code |
| [`CommandSource`](#enum-commandsource) | enum | 이번 cycle에 쓴 출처 |
| [`HandState`](#handstate) | struct | [`get_state()`](hand.md#handget_state)가 반환하는 값 |
| [`ActuatorState`](#actuatorstate) | struct | actuator 16개의 위치·속도·전류 |
| [`JointState`](#jointstate) | struct | joint 21개의 각도·속도·추정 토크 |
| [`TactileState`](#tactilestate) | struct | 촉각 센서 값 |
| [`CommandedState`](#commandedstate) | struct | 이번 cycle의 command 입력과 출력 |
| [`ActuatorHealth`](#actuatorhealth) | struct | actuator별 enable·fault |
| [`ControllerOutput`](#type-aliases) | alias | setpoint variant |

---

## enum `HandLifecycle`

```cpp
enum class HandLifecycle {
  Disconnected,   // CAN에 연결하지 않은 상태
  Connected,      // frame을 받고 있으나 토크도 TX도 없는 상태
  Running,        // actuator enable을 확인하고 제어 중
  Stopped,        // actuator가 quick stop에 도달한 것을 확인한 상태
  Faulted,        // 통신 오류나 제어·통신 루프 예외로 멈춘 상태
};

constexpr const char* to_string(HandLifecycle lifecycle) noexcept;
```

[`Diagnostics::lifecycle`](types_diagnostics.md#diagnostics)은 요구한 값이 아니라 관측된 값입니다.
전이와 호출 규칙은 [C++ guide](../07_cpp_usage_guide.md#22-state-transition-calls)에 있습니다.

---

## enum `HomingState`

```cpp
enum class HomingState {
  NotRun     = 0,  // 아직 안 함. create 직후, reconnect 후
  Succeeded  = 1,  // 원점 확정. Succeeded에서만 motion command가 통과
  InProgress = 2,  // 진행 중
  Failed     = 3,  // 실패
};

constexpr const char* to_string(HomingState state) noexcept;
```

`Diagnostics::homing_state`로 읽습니다.

---

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

actuator가 보고하는 `0x603F` error code입니다. fault가 떠도 SDK는 스스로 멈추지 않습니다. 대신
제어·통신 루프가 그 actuator에 fault reset을 반복 시도하고, 해당 actuator는 command 적용 대상에서
제외됩니다. `disabled_actuators` 필드에 지정한 index는 보고되지 않습니다. fault가 보고됐을 때 남는 log와 조치는
[Error messages의 E504](../15_error_messages.md#6-e5xx-hardwarefault)에 있습니다.

---

## enum `CommandSource`

```cpp
enum class CommandSource {
  None,        // 출력 없음. 수신만 하고 TX를 안 함
  Controller,  // controller 출력
  QuickStop,   // quick stop
  Homing,      // homing
};
```

---

## `HandState`

| Field | Type | Description |
|---|---|---|
| `timestamp` | `std::int64_t` | 그 cycle의 시각 |
| `actuators` | [`ActuatorState`](#actuatorstate) | actuator 16개의 위치·속도·전류 |
| `joints` | [`JointState`](#jointstate) | joint 21개의 각도·속도·추정 토크 |
| `tactile` | [`TactileState`](#tactilestate) | 촉각 센서 값 |
| `commanded` | [`CommandedState`](#commandedstate) | 이번 cycle의 command 입력과 출력 |

`timestamp` 필드는 `CLOCK_REALTIME` 기준 epoch ns이며, `0`이면 아직 값이 없다는 뜻입니다. rosbag이나
ROS `header.stamp`와 맞추는 용도이고, 주기 측정에는 `Diagnostics::last_period_ms` 필드를 사용하십시오.

---

## `ActuatorState`

| Field | Type | Unit |
|---|---|---|
| `position_count` | `std::array<std::int32_t, kActuatorCount>` | encoder count |
| `velocity_rpm` | `std::array<std::int32_t, kActuatorCount>` | rpm |
| `current_mA` | `std::array<std::int16_t, kActuatorCount>` | mA |

---

## `JointState`

| Field | Type | Unit | Description |
|---|---|---|---|
| `position_rad` | `std::array<double, kJointCount>` | rad | joint 21개의 각도 |
| `velocity_rad_s` | `std::array<double, kJointCount>` | rad/s | joint 21개의 속도. passive joint4 포함 |
| `effort_Nm` | `std::array<double, kJointCount>` | N·m | active joint의 추정 토크. joint4는 NaN |

`effort_Nm` 필드는 actuator 전류(`current_mA`)에서 모터 토크 상수를 고려하여 추정된 joint 토크입니다.
구동부의 손실과 마찰을 반영하지 않으므로 실제 joint 토크와 다를 수 있습니다. passive joint인 joint4에는
effort가 없어 NaN입니다.

---

## `TactileState`

| Field | Type | Description |
|---|---|---|
| `fingers` | `std::array<std::array<std::int32_t, kTactileTaxelsPerFinger>, kFingerCount>` | 바깥 index가 [`Finger`](types_description.md#enum-finger) |
| `palm` | `std::array<std::int32_t, kPalmTactileCount>` | palm1 upper 20 + lower 20 + palm2 18 |

값은 센서의 raw value입니다. [`Hand::set_tactile_bias()`](hand.md#handset_tactile_bias)로 bias를
설정하면 값은 bias와의 차이이며 음수일 수 있습니다.
[`Hand::get_tactile_bias()`](hand.md#handget_tactile_bias)도 이 type으로 bias를 반환합니다.

---

## `CommandedState`

| Field | Type | Description |
|---|---|---|
| `controller_input` | [`ControllerCommand`](types_command.md#type-aliases) | 현재 적용 중인 command |
| `controller_output` | [`ControllerOutput`](#type-aliases) | limit과 fault mask를 지난 actuator position 또는 effort setpoint |
| `selected_source` | [`CommandSource`](#enum-commandsource) | 이번 cycle에 실제로 쓴 출처 |
| `max_effort_pct` | `std::array<double, kActuatorCount>` | 적용 중인 actuator별 상한 |

`controller_output` 필드만으로는 actuator가 값을 적용했는지 알 수 없습니다. 판단 방법은
[C++ guide](../07_cpp_usage_guide.md#72-applied-command)에 있습니다. `selected_source` 필드와 함께
확인하십시오.

---

### `ActuatorPositionSetpoint` / `ActuatorEffortSetpoint`

| Owning type | Field | Type | Unit |
|---|---|---|---|
| `ActuatorPositionSetpoint` | `target_position_cnt` | `std::array<double, kActuatorCount>` | encoder count |
| `ActuatorEffortSetpoint` | `target_effort_pct` | `std::array<double, kActuatorCount>` | 정격 전류의 0.1% |

---

## `ActuatorHealth`

| Field | Type | Description |
|---|---|---|
| `enabled` | `std::array<bool, kActuatorCount>` | actuator enable 여부 |
| `fault` | `std::array<ActuatorFault, kActuatorCount>` | actuator error code |

---

## Type aliases

```cpp
using ControllerOutput = std::variant<std::monostate, ActuatorPositionSetpoint, ActuatorEffortSetpoint>;
```

`std::monostate`는 control session(`Running`·`Stopped`·`Faulted`) 밖이라 setpoint가 없다는
뜻입니다.
