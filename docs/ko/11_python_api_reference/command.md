[← Python API Reference](../11_python_api_reference.md)

# Command

| Symbol | Kind | Description |
|---|---|---|
| [`CommandMode`](#enum-commandmode) | `enum.Enum` | 활성 controller |
| [`Idle`](#idle) | class | 모든 actuator의 effort를 `0`으로 |
| [`JointPositionCommand`](#jointpositioncommand) | class | joint 위치 목표 |
| [`JointImpedanceCommand`](#jointimpedancecommand) | class | joint impedance 목표 |
| [`ActuatorPositionCommand`](#actuatorpositioncommand) | class | actuator 위치 목표 |
| [`ActuatorEffortCommand`](#actuatoreffortcommand) | class | actuator effort 목표 |

command 5종은 [`Hand.set_command()`](hand.md#handset_command)에 넘깁니다. `Idle`을 뺀 넷은 길이 16인
`target` 필드 하나를 갖고, 생성자에 `target`을 넘기거나 생략해 전부 `0`으로 만듭니다.

```python
command = JointPositionCommand(target)    # target: 길이 16인 list·tuple·numpy 배열
command = JointPositionCommand()          # target이 전부 0
```

`target` 필드는 쓰기 가능한 `float64` `(16,)` numpy 배열입니다. `command.target[5] = 0.3`처럼 일부만
바꾸거나 길이 16인 배열을 통째로 대입할 수 있습니다. 길이가 16이 아니면 생성자와 대입 모두
`ValueError`를 던집니다. 바꾼 값은 `set_command()`를 다시 호출해야 적용됩니다.

---

## enum `CommandMode`

```python
class CommandMode(enum.Enum):
    IDLE = 0
    JOINT_POSITION = 1
    JOINT_IMPEDANCE = 2
    ACTUATOR_POSITION = 3
    ACTUATOR_EFFORT = 4
```

`set_command()`에 넘긴 command의 class와 1:1이며 [`get_command_mode()`](hand.md#handget_command_mode)가
돌려줍니다.

---

## `Idle`

```python
class Idle:
    def __init__(self) -> None: ...
```

필드가 없습니다. 모든 actuator의 effort를 `0`으로 보냅니다. drive는 켜진 채로 남으므로 직접 관절을
움직이면 약간 저항이 느껴집니다. 힘을 완전히 빼려면 [`stop()`](hand.md#handstop)을 쓰십시오.

---

## `JointPositionCommand`

| Field | Type | Unit | Description |
|---|---|---|---|
| `target` | `float64` `(16,)` | rad | active joint 위치 목표 16개 |

---

### `JointPositionCommand.clamp()`

```python
def clamp(self) -> None: ...
```

`target` 필드를 finger별 도달 범위 안으로 투영합니다. `target` 필드의 값을 그 자리에서 바꿉니다.

**Notes**          ｜ [`set_command()`](hand.md#handset_command)가 joint command를 받을 때 같은 투영을
수행하므로, 보내기 전에 결과를 미리 확인할 때만 직접 호출하십시오. 범위 모델은
[Workspace limits](../14_workspace_limits.md)에 있습니다

---

## `JointImpedanceCommand`

| Field | Type | Unit | Description |
|---|---|---|---|
| `target` | `float64` `(16,)` | rad | active joint 위치 목표 16개 |

gain은 command가 아니라
[`ControllerConfig.JointImpedanceController`](config.md#controllerconfigjointimpedancecontroller)에
있습니다.

---

### `JointImpedanceCommand.clamp()`

```python
def clamp(self) -> None: ...
```

[`JointPositionCommand.clamp()`](#jointpositioncommandclamp)와 같습니다.

---

## `ActuatorPositionCommand`

| Field | Type | Unit | Description |
|---|---|---|---|
| `target` | `float64` `(16,)` | encoder count | 모터 위치 목표 16개 |

---

## `ActuatorEffortCommand`

| Field | Type | Unit | Description |
|---|---|---|---|
| `target` | `float64` `(16,)` | 정격 전류의 0.1% | effort 목표 16개 |

부호가 방향이고, 크기는 max effort 값까지만 나갑니다. 정격 전류는 모든 모터가 400 mA이므로
`1000`(100%)이 400 mA에 해당합니다.

---

## Validation

아래 중 하나라도 걸리면 command 전체가 버려지고 직전 command가 그대로 유지됩니다.

| Condition | Result |
|---|---|
| `target` 필드에 NaN 또는 Inf | 예외 없음. warning log, `Diagnostics.nan_command_count` 증가 |
| `ActuatorPositionCommand`의 `target`이 `int32` 범위 밖 | 예외 없음. warning log, `Diagnostics.nan_command_count` 증가 |
| `lifecycle` 값이 `RUNNING`이 아님 | `Error`(`WRONG_CALL_ORDER`) |
| `homing_state` 값이 `SUCCEEDED`가 아님 ([`Idle`](#idle) 제외) | `Error`(`WRONG_CALL_ORDER`) |
