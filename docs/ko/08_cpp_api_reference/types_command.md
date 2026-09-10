[← C++ API Reference](../08_cpp_api_reference.md)

# `types/command.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [`CommandMode`](#enum-commandmode) | enum | 활성 controller |
| [`Idle`](#idle) | struct | 모든 actuator의 effort를 `0`으로 |
| [`JointPositionCommand`](#jointpositioncommand) | struct | joint 위치 목표 |
| [`JointImpedanceCommand`](#jointimpedancecommand) | struct | joint impedance 목표 |
| [`ActuatorPositionCommand`](#actuatorpositioncommand) | struct | actuator 위치 목표 |
| [`ActuatorEffortCommand`](#actuatoreffortcommand) | struct | actuator effort 목표 |
| [`ControllerCommand`](#type-aliases) | alias | command 하나를 담는 variant |
| [`to_command_mode()`](#to_command_mode) | function | variant에서 mode를 꺼냄 |

---

## enum `CommandMode`

```cpp
enum class CommandMode { Idle, JointPosition, JointImpedance, ActuatorPosition, ActuatorEffort };
```

[`set_command()`](hand.md#handset_command) 인자 type과 1:1이며 [`get_command_mode()`](hand.md#handget_command_mode)가 돌려줍니다.

---

## `Idle`

```cpp
struct Idle {};
```

멤버가 없습니다. 모든 actuator의 effort를 `0`으로 보냅니다. drive는 켜진 채로 남으므로 직접
관절을 움직이면 약간 저항이 느껴집니다. 힘을 완전히 빼려면 [`stop()`](hand.md#handstop)을 쓰십시오.

---

## `JointPositionCommand`

| Field | Type | Unit | Description |
|---|---|---|---|
| `target` | `std::array<double, kActiveJointCount>` | rad | active joint 위치 목표 16개 |

---

### `JointPositionCommand::clamp()`

```cpp
void clamp();
```

`target` 필드를 finger별 도달 범위 안으로 투영합니다.

**Notes**          ｜ [`set_command()`](hand.md#handset_command)가 joint command를 받을 때 같은 투영을
수행하므로, 보내기 전에 결과를 미리 확인할 때만 직접 호출하십시오. 범위 모델은
[Workspace limits](../14_workspace_limits.md)에 있습니다.

---

## `JointImpedanceCommand`

| Field | Type | Unit | Description |
|---|---|---|---|
| `target` | `std::array<double, kActiveJointCount>` | rad | active joint 위치 목표 16개 |

gain은 command가 아니라
[`ControllerConfig::JointImpedanceController`](types_config.md#controllerconfigjointimpedancecontroller)에 있습니다.

---

### `JointImpedanceCommand::clamp()`

```cpp
void clamp();
```

[`JointPositionCommand::clamp()`](#jointpositioncommandclamp)와 같습니다.

---

## `ActuatorPositionCommand`

| Field | Type | Unit | Description |
|---|---|---|---|
| `target` | `std::array<double, kActuatorCount>` | encoder count | 모터 위치 목표 16개 |

---

## `ActuatorEffortCommand`

| Field | Type | Unit | Description |
|---|---|---|---|
| `target` | `std::array<double, kActuatorCount>` | 정격 전류의 0.1% | effort 목표 16개 |

부호가 방향이고, 크기는 `max_effort`까지만 나갑니다. 정격 전류는 모든 모터가 400 mA이므로
`1000`(100%)이 400 mA에 해당합니다.

---

## Type aliases

```cpp
using ControllerCommand = std::variant<Idle, JointPositionCommand, JointImpedanceCommand,
                                       ActuatorPositionCommand, ActuatorEffortCommand>;
```

---

## `to_command_mode()`

```cpp
CommandMode to_command_mode(const ControllerCommand& command);
```

**Parameters**     ｜ `command` — command variant<br>
**Returns**        ｜ 담긴 type에 대응하는 [`CommandMode`](#enum-commandmode)

---

## Validation

아래 중 하나라도 걸리면 command 전체가 버려지고 직전 command가 그대로 유지됩니다.

| Condition | Result |
|---|---|
| `target` 필드에 NaN 또는 Inf | 예외 없음. warning log, `Diagnostics::nan_command_count` 증가 |
| `ActuatorPositionCommand::target`이 `int32` 범위 밖 | 예외 없음. warning log, `Diagnostics::nan_command_count` 증가 |
| `homing_state != Succeeded` ([`Idle`](#idle) 제외) | `WrongCallOrder` |
