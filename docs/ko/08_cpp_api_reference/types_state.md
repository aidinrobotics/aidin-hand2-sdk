[← C++ API Reference](../08_cpp_api_reference.md)

# `types/state.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [`HandLifecycle`](#enum-handlifecycle) | enum | [`Hand`](hand.md#hand)의 상태 5개 |
| [`HomingState`](#enum-homingstate) | enum | 원점 상태 |
| [`ActuatorFault`](#enum-actuatorfault) | enum | actuator error code |
| [`CommandSource`](#enum-commandsource) | enum | 이번 cycle에 쓴 출처 |
| [`HandState`](#handstate) | struct | [`get_state()`](hand.md#handget_state)가 돌려주는 값 |
| [`ActuatorState`](#actuatorstate) | struct | actuator 관측 |
| [`JointState`](#jointstate) | struct | FK 결과 |
| [`TactileState`](#tactilestate) | struct | 촉각 |
| [`CommandedState`](#commandedstate) | struct | 이번 cycle의 command 입력·출력 |
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
  None                     = 0x0000,  // 정상
  OverCurrentError         = 0x2310,  // 전기
  OverVoltageError         = 0x3210,  // 전기
  UnderVoltageError        = 0x3220,  // 전기
  OverTemperatureError     = 0x4210,  // 열
  CurrentDetectionError    = 0x5210,  // 전기
  SpeedError               = 0x7310,  // 제어
  CommunicationError       = 0x7500,  // 통신
  FollowingError           = 0x8611,  // 제어, 추종 실패
  HallSensorError          = 0xFF01,  // 홀센서
  OverLoadError            = 0xFF02,  // 기구 부하
  PositiveLimitSwitchError = 0xFF03,  // 리밋
  NegativeLimitSwitchError = 0xFF04,  // 리밋
  EmergencySwitchError     = 0xFF05,  // 안전
  Sto1Error                = 0xFF06,  // 안전 (STO = Safe Torque Off)
  Sto2Error                = 0xFF07,  // 안전 (STO = Safe Torque Off)
  SerialEncoderChannelAError             = 0xFF24,  // 엔코더
  SerialEncoderChannelADisconnectedError = 0xFF25,  // 엔코더
  SerialEncoderChannelBError             = 0xFF26,  // 엔코더
  SerialEncoderChannelBDisconnectedError = 0xFF27,  // 엔코더
};

constexpr const char* to_string(ActuatorFault fault) noexcept;
```

actuator가 보고하는 `0x603F` error code입니다. fault가 떠도 SDK는 스스로 멈추지 않습니다. 대신
제어·통신 루프가 그 actuator에 fault reset을 반복 시도하고, 해당 actuator는 command 적용 대상에서
제외됩니다. `disabled_actuators` 필드에 지정한 index는 보고되지 않습니다. code별 점검 항목은
[Error messages](../15_error_messages.md#11-actuator-fault)에 있습니다.

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
| `timestamp` | `std::int64_t` | 그 cycle의 `CLOCK_REALTIME` epoch ns. `0`은 아직 값이 없다는 뜻 |
| `actuators` | [`ActuatorState`](#actuatorstate) | actuator 관측 |
| `joints` | [`JointState`](#jointstate) | FK 결과 |
| `tactile` | [`TactileState`](#tactilestate) | 촉각 |
| `commanded` | [`CommandedState`](#commandedstate) | 이번 cycle의 command 입력·출력 |

`timestamp` 필드는 rosbag이나 ROS `header.stamp`와 맞추는 용도입니다. 주기 측정에는
`Diagnostics::last_period_ms` 필드를 사용하십시오.

---

## `ActuatorState`

| Field | Type | Unit |
|---|---|---|
| `position_count` | `std::array<double, kActuatorCount>` | encoder count |
| `velocity_rpm` | `std::array<double, kActuatorCount>` | rpm |
| `current_mA` | `std::array<double, kActuatorCount>` | mA |

---

## `JointState`

| Field | Type | Unit | Status |
|---|---|---|---|
| `position_rad` | `std::array<double, kJointCount>` | rad | FK 결과 |
| `velocity_rad_s` | `std::array<double, kJointCount>` | rad/s | `0` placeholder |
| `effort_Nm` | `std::array<double, kJointCount>` | N·m | `0` placeholder |

velocity와 effort의 `0`은 측정값이 아닙니다.

---

## `TactileState`

| Field | Type | Description |
|---|---|---|
| `fingers` | `std::array<std::array<double, kTactileTaxelsPerFinger>, kFingerCount>` | 바깥 index가 [`Finger`](types_description.md#enum-finger). 값은 16-bit 원시 count |
| `palm` | `std::array<double, kPalmTactileCount>` | palm1 upper 20 + lower 20 + palm2 18 |

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
