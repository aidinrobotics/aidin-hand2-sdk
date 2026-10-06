[← Python API Reference](../11_python_api_reference.md)

# State

| Symbol | Kind | Description |
|---|---|---|
| [`HandLifecycle`](#enum-handlifecycle) | `enum.Enum` | [`Hand`](hand.md#hand)의 상태 5개 |
| [`HomingState`](#enum-homingstate) | `enum.Enum` | 원점 상태 |
| [`ActuatorFault`](#enum-actuatorfault) | `enum.IntEnum` | actuator error code |
| [`CommandSource`](#enum-commandsource) | `enum.Enum` | 이번 cycle에 쓴 출처 |
| [`HandState`](#handstate) | class | [`get_state()`](hand.md#handget_state)가 돌려주는 값 |
| [`ActuatorState`](#actuatorstate) | class | actuator 관측 |
| [`JointState`](#jointstate) | class | FK 결과 |
| [`TactileState`](#tactilestate) | class | 촉각 |
| [`CommandedState`](#commandedstate) | class | 이번 cycle의 command 입력·출력 |
| [`ActuatorHealth`](#actuatorhealth) | class | actuator별 enable·fault |

`HandState`와 그 안의 객체는 [`get_state()`](hand.md#handget_state)를 호출한 시점의 사본입니다. 안의
numpy 배열은 읽기 전용이므로, 값을 바꿔 쓰려면 `.copy()`로 복사하십시오.

---

## enum `HandLifecycle`

```python
class HandLifecycle(enum.Enum):
    DISCONNECTED = 0   # CAN에 연결하지 않은 상태
    CONNECTED = 1      # frame을 받고 있으나 토크도 TX도 없는 상태
    RUNNING = 2        # actuator enable을 확인하고 제어 중
    STOPPED = 3        # actuator가 quick stop에 도달한 것을 확인한 상태
    FAULTED = 4        # 통신 오류나 제어·통신 루프 예외로 멈춘 상태
```

[`Diagnostics.lifecycle`](diagnostics.md#diagnostics)은 요구한 값이 아니라 관측된 값입니다. 전이와
호출 규칙은 [Python guide](../10_python_usage_guide.md#22-state-transition-calls)에 있습니다.

---

## enum `HomingState`

```python
class HomingState(enum.Enum):
    NOT_RUN = 0        # 아직 안 함. create 직후, reconnect 후
    SUCCEEDED = 1      # 원점 확정. SUCCEEDED에서만 motion command가 통과
    IN_PROGRESS = 2    # 진행 중
    FAILED = 3         # 실패
```

`Diagnostics.homing_state`로 읽습니다.

---

## enum `ActuatorFault`

```python
class ActuatorFault(enum.IntEnum):
    NONE = 0x0000                                          # 정상
    OVER_CURRENT_ERROR = 0x2310                            # 전기
    OVER_VOLTAGE_ERROR = 0x3210                            # 전기
    UNDER_VOLTAGE_ERROR = 0x3220                           # 전기
    OVER_TEMPERATURE_ERROR = 0x4210                        # 열
    CURRENT_DETECTION_ERROR = 0x5210                       # 전기
    SPEED_ERROR = 0x7310                                   # 제어
    COMMUNICATION_ERROR = 0x7500                           # 통신
    FOLLOWING_ERROR = 0x8611                               # 제어, 추종 실패
    HALL_SENSOR_ERROR = 0xFF01                             # 홀센서
    OVER_LOAD_ERROR = 0xFF02                               # 기구 부하
    POSITIVE_LIMIT_SWITCH_ERROR = 0xFF03                   # 리밋
    NEGATIVE_LIMIT_SWITCH_ERROR = 0xFF04                   # 리밋
    EMERGENCY_SWITCH_ERROR = 0xFF05                        # 안전
    STO1_ERROR = 0xFF06                                    # 안전 (STO = Safe Torque Off)
    STO2_ERROR = 0xFF07                                    # 안전 (STO = Safe Torque Off)
    SERIAL_ENCODER_CHANNEL_A_ERROR = 0xFF24                # 엔코더
    SERIAL_ENCODER_CHANNEL_A_DISCONNECTED_ERROR = 0xFF25   # 엔코더
    SERIAL_ENCODER_CHANNEL_B_ERROR = 0xFF26                # 엔코더
    SERIAL_ENCODER_CHANNEL_B_DISCONNECTED_ERROR = 0xFF27   # 엔코더
```

actuator가 보고하는 `0x603F` error code이며, 값이 곧 그 code입니다. 정수와 비교할 수 있고
`f"0x{fault:04X}"`로 code를 출력할 수 있습니다. 위에 정의되지 않은 code를 보고하면
[`ActuatorHealth.fault`](#actuatorhealth)에는 정수 그대로 담깁니다.

fault가 떠도 SDK는 스스로 멈추지 않습니다. 대신 제어·통신 루프가 그 actuator에 fault reset을 반복
시도하고, 해당 actuator는 command 적용 대상에서 제외됩니다. `disabled_actuators` 필드에 지정한
index는 보고되지 않습니다. code별 점검 항목은
[Error messages](../15_error_messages.md#11-actuator-fault)에 있습니다.

---

## enum `CommandSource`

```python
class CommandSource(enum.Enum):
    NONE = 0           # 출력 없음. 수신만 하고 TX를 안 함
    CONTROLLER = 1     # controller 출력
    QUICK_STOP = 2     # quick stop
    HOMING = 3         # homing
```

---

## `HandState`

| Field | Type | Description |
|---|---|---|
| `timestamp` | `int` | 그 cycle의 `CLOCK_REALTIME` epoch ns. `0`은 아직 값이 없다는 뜻 |
| `actuators` | [`ActuatorState`](#actuatorstate) | actuator 관측 |
| `joints` | [`JointState`](#jointstate) | FK 결과 |
| `tactile` | [`TactileState`](#tactilestate) | 촉각 |
| `commanded` | [`CommandedState`](#commandedstate) | 이번 cycle의 command 입력·출력 |

`timestamp` 필드는 `time.time_ns()`나 ROS `header.stamp`와 맞추는 용도입니다. 주기 측정에는
`Diagnostics.last_period_ms` 필드를 사용하십시오.

---

## `ActuatorState`

| Field | Type | Unit |
|---|---|---|
| `position_count` | `int32` `(16,)` | encoder count |
| `velocity_rpm` | `int32` `(16,)` | rpm |
| `current_mA` | `int16` `(16,)` | mA |

---

## `JointState`

| Field | Type | Unit |
|---|---|---|
| `position_rad` | `float64` `(21,)` | rad |

---

## `TactileState`

| Field | Type | Description |
|---|---|---|
| `fingers` | `uint16` `(5, 17)` | 첫 번째 index가 [`Finger`](description.md#enum-finger) |
| `palm` | `uint16` `(58,)` | palm1 upper `[0:20]`, lower `[20:40]`, palm2 `[40:58]` |

값은 단위와 정규화가 없는 16-bit raw value입니다. baseline과의 차이를 계산할 때는
`astype(numpy.int32)`로 바꾼 뒤 빼십시오. `uint16`끼리 빼면 음수가 큰 양수로 바뀝니다.

---

## `CommandedState`

| Field | Type | Description |
|---|---|---|
| `controller_input` | command 5종 중 하나 | 현재 적용 중인 command |
| `controller_output` | `ActuatorPositionSetpoint` · `ActuatorEffortSetpoint` · `None` | limit과 fault mask를 지난 actuator position 또는 effort setpoint |
| `selected_source` | [`CommandSource`](#enum-commandsource) | 이번 cycle에 실제로 쓴 출처 |
| `max_effort_pct` | `float64` `(16,)` | 적용 중인 actuator별 상한 |

`controller_output` 필드가 `None`이면 control session(`RUNNING`·`STOPPED`·`FAULTED`) 밖이라 setpoint가
없다는 뜻입니다. `controller_output` 필드만으로는 actuator가 값을 적용했는지 알 수 없으므로
`selected_source` 필드와 함께 확인하십시오. 판단 방법은
[Python guide](../10_python_usage_guide.md#72-applied-command)에 있습니다.

---

### `ActuatorPositionSetpoint` / `ActuatorEffortSetpoint`

| Owning type | Field | Type | Unit |
|---|---|---|---|
| `ActuatorPositionSetpoint` | `target_position_cnt` | `float64` `(16,)` | encoder count |
| `ActuatorEffortSetpoint` | `target_effort_pct` | `float64` `(16,)` | 정격 전류의 0.1% |

어느 쪽인지는 `isinstance(output, ActuatorPositionSetpoint)`로 구분합니다.

---

## `ActuatorHealth`

| Field | Type | Description |
|---|---|---|
| `enabled` | `bool` `(16,)` | actuator enable 여부 |
| `fault` | `list[ActuatorFault \| int]` | actuator error code. 정의되지 않은 code는 `int` |
