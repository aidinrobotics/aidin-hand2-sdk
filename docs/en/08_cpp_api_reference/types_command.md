[← C++ API Reference](../08_cpp_api_reference.md)

# `types/command.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [`CommandMode`](#enum-commandmode) | enum | The active controller |
| [`Idle`](#idle) | struct | Torque-free |
| [`JointPositionCommand`](#jointpositioncommand) | struct | Joint position target |
| [`JointImpedanceCommand`](#jointimpedancecommand) | struct | Joint impedance target |
| [`ActuatorPositionCommand`](#actuatorpositioncommand) | struct | Actuator position target |
| [`ActuatorEffortCommand`](#actuatoreffortcommand) | struct | Actuator effort target |
| [`ControllerCommand`](#type-aliases) | alias | Variant holding one command |
| [`to_command_mode()`](#to_command_mode) | function | Reads the mode out of the variant |

## enum `CommandMode`

```cpp
enum class CommandMode { Idle, JointPosition, JointImpedance, ActuatorPosition, ActuatorEffort };
```

One to one with the [`set_command()`](hand.md#handset_command) argument types, and returned by
[`get_command_mode()`](hand.md#handget_command_mode).

## `Idle`

```cpp
struct Idle {};
```

No members. It sends an effort of `0` to every actuator. The drives stay on, so moving a joint by
hand meets slight resistance. To remove the torque completely, use [`stop()`](hand.md#handstop).

## `JointPositionCommand`

| Field | Type | Unit | Description |
|---|---|---|---|
| `target` | `std::array<double, kActiveJointCount>` | rad | 16 active joint position targets |

### `JointPositionCommand::clamp()`

```cpp
void clamp();
```

Projects `target` into the reachable range of each finger.

**Notes**          ｜ [`set_command()`](hand.md#handset_command) applies the same projection to a joint
command, so call this yourself only to inspect the result before sending. The range model is in
[Workspace limits](../14_workspace_limits.md).

## `JointImpedanceCommand`

| Field | Type | Unit | Description |
|---|---|---|---|
| `target` | `std::array<double, kActiveJointCount>` | rad | 16 active joint position targets |

The gains are not in the command; they live in
[`ControllerConfig::JointImpedanceController`](types_config.md#controllerconfigjointimpedancecontroller).

### `JointImpedanceCommand::clamp()`

```cpp
void clamp();
```

Same as [`JointPositionCommand::clamp()`](#jointpositioncommandclamp).

## `ActuatorPositionCommand`

| Field | Type | Unit | Description |
|---|---|---|---|
| `target` | `std::array<double, kActuatorCount>` | encoder count | 16 motor position targets |

## `ActuatorEffortCommand`

| Field | Type | Unit | Description |
|---|---|---|---|
| `target` | `std::array<double, kActuatorCount>` | % of rated current (`1000`=100 %) | 16 effort targets |

The sign is direction, and the magnitude only goes out as far as `max_effort`.

## Type aliases

```cpp
using ControllerCommand = std::variant<Idle, JointPositionCommand, JointImpedanceCommand,
                                       ActuatorPositionCommand, ActuatorEffortCommand>;
```

## `to_command_mode()`

```cpp
CommandMode to_command_mode(const ControllerCommand& command);
```

**Parameters**     ｜ `command` — the command variant<br>
**Returns**        ｜ The [`CommandMode`](#enum-commandmode) matching the type it holds

## Validation

Any of these drops the whole command and leaves the previous one in effect.

| Condition | Result |
|---|---|
| NaN or Inf in `target` | No exception. Warning log, `Diagnostics::nan_command_count` increments |
| `ActuatorPositionCommand::target` outside the `int32` range | No exception. Warning log, `Diagnostics::nan_command_count` increments |
| `homing_state != Succeeded` ([`Idle`](#idle) exempt) | `WrongCallOrder` |

---
