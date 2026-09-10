# C++ API Reference

The public symbols, organized by header file. Functions carry their declaration and contract;
structs carry a field table. For the order in which you use them see the
[C++ guide](07_cpp_usage_guide.md); for what to do about a specific message see
[Error messages](15_error_messages.md).

```cpp
#include <aidin_hand2/aidin_hand2.hpp>   // every public symbol, namespace aidin_hand2
namespace ah2 = aidin_hand2;             // the alias used in this document
```

Function entries are written as **Parameters → Returns → Throws → Preconditions →
Postconditions → Notes**, skipping whatever does not apply.

## Contents

&nbsp;&nbsp;[**`hand/hand_manager.hpp`**](08_cpp_api_reference/hand_manager.md) — `HandManager`<br>
&nbsp;&nbsp;[**`hand/hand.hpp`**](08_cpp_api_reference/hand.md) — `Hand`<br>
&nbsp;&nbsp;[**`hand/hand_kinematics.hpp`**](08_cpp_api_reference/hand_kinematics.md) — FK / IK<br>
&nbsp;&nbsp;[**`types/description.hpp`**](08_cpp_api_reference/types_description.md) — constants, `HandSide`, `Finger`, array conventions<br>
&nbsp;&nbsp;[**`types/config.hpp`**](08_cpp_api_reference/types_config.md) — `HandConfig`, `ControllerConfig`<br>
&nbsp;&nbsp;[**`types/command.hpp`**](08_cpp_api_reference/types_command.md) — `CommandMode` and the 5 commands<br>
&nbsp;&nbsp;[**`types/state.hpp`**](08_cpp_api_reference/types_state.md) — `HandLifecycle`, `HomingState`, `HandState`, `ActuatorFault`<br>
&nbsp;&nbsp;[**`types/diagnostics.hpp`**](08_cpp_api_reference/types_diagnostics.md) — `Diagnostics`<br>
&nbsp;&nbsp;[**`types/error.hpp`**](08_cpp_api_reference/types_error.md) — `ErrorCode`, `Exception`<br>
&nbsp;&nbsp;[**`logging/logging.hpp`**](08_cpp_api_reference/logging.md) — `LogLevel` and the logging functions<br>
&nbsp;&nbsp;[**`version.hpp`**](08_cpp_api_reference/version.md) — version constants and `find_package`

---

## Function index

Every public function, grouped by the file that declares it. The names are in normalized form: one
with `::` is a member of that class or struct, and one without `::` is a free function at the
`aidin_hand2` namespace level.

**[`hand/hand_manager.hpp`](08_cpp_api_reference/hand_manager.md)**

| Function | Role |
|---|---|
| [`HandManager::HandManager()`](08_cpp_api_reference/hand_manager.md#handmanagerhandmanager) | Create an empty manager, or move from another one |
| [`HandManager::~HandManager()`](08_cpp_api_reference/hand_manager.md#handmanagerhandmanager-1) | Quick stop every hand it owns, then shut them down |
| [`HandManager::operator=`](08_cpp_api_reference/hand_manager.md#handmanageroperator) | Move assignment |
| [`HandManager::create()`](08_cpp_api_reference/hand_manager.md#handmanagercreate) | Validate a config and create a `Hand` |
| [`HandManager::destroy()`](08_cpp_api_reference/hand_manager.md#handmanagerdestroy) | Shut one `Hand` down and invalidate its handles |
| [`HandManager::destroy_all()`](08_cpp_api_reference/hand_manager.md#handmanagerdestroy_all) | Shut down every `Hand` it owns |

**[`hand/hand.hpp`](08_cpp_api_reference/hand.md)**

| Function | Role |
|---|---|
| [`Hand::connect()`](08_cpp_api_reference/hand.md#handconnect) | Open the CAN connection and start receiving state |
| [`Hand::disconnect()`](08_cpp_api_reference/hand.md#handdisconnect) | Quick stop the actuators, then close the CAN connection |
| [`Hand::reconnect()`](08_cpp_api_reference/hand.md#handreconnect) | Reconnect CAN after a fault |
| [`Hand::run()`](08_cpp_api_reference/hand.md#handrun) | Confirm the actuator enable and start control |
| [`Hand::stop()`](08_cpp_api_reference/hand.md#handstop) | Quick stop the actuators |
| [`Hand::home()`](08_cpp_api_reference/hand.md#handhome) | Wait until homing completes (blocking) |
| [`Hand::set_command(Idle)`](08_cpp_api_reference/hand.md#handset_command) | Zero output |
| [`Hand::set_command(JointPositionCommand)`](08_cpp_api_reference/hand.md#handset_command) | Joint position target |
| [`Hand::set_command(JointImpedanceCommand)`](08_cpp_api_reference/hand.md#handset_command) | Joint impedance target |
| [`Hand::set_command(ActuatorPositionCommand)`](08_cpp_api_reference/hand.md#handset_command) | Actuator position target |
| [`Hand::set_command(ActuatorEffortCommand)`](08_cpp_api_reference/hand.md#handset_command) | Actuator effort target |
| [`Hand::set_max_effort(double)`](08_cpp_api_reference/hand.md#handset_max_effort) | One effort ceiling for every actuator |
| [`Hand::set_max_effort(array<double, 16>)`](08_cpp_api_reference/hand.md#handset_max_effort) | Per-actuator effort ceiling |
| [`Hand::set_controller_config()`](08_cpp_api_reference/hand.md#handset_controller_config) | Position filter and impedance gains |
| [`Hand::get_state()`](08_cpp_api_reference/hand.md#handget_state) | The latest [`HandState`](08_cpp_api_reference/types_state.md#handstate) |
| [`Hand::get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics) | The latest [`Diagnostics`](08_cpp_api_reference/types_diagnostics.md#diagnostics) |
| [`Hand::get_command_mode()`](08_cpp_api_reference/hand.md#handget_command_mode) | The [`CommandMode`](08_cpp_api_reference/types_command.md#enum-commandmode) that is active now |

**[`hand/hand_kinematics.hpp`](08_cpp_api_reference/hand_kinematics.md)**

| Function | Role |
|---|---|
| [`fk_actuator_to_joint()`](08_cpp_api_reference/hand_kinematics.md#fk_actuator_to_joint) | 16 actuator encoders → 21 joints |
| [`ik_joint_to_actuator()`](08_cpp_api_reference/hand_kinematics.md#ik_joint_to_actuator) | 16 active joints → 16 actuator encoders |

**[`types/config.hpp`](08_cpp_api_reference/types_config.md)**

| Function | Role |
|---|---|
| [`HandConfig::HandConfig()`](08_cpp_api_reference/types_config.md#handconfighandconfig) | A config filled in with the defaults |

**[`types/command.hpp`](08_cpp_api_reference/types_command.md)**

| Function | Role |
|---|---|
| [`JointPositionCommand::clamp()`](08_cpp_api_reference/types_command.md#jointpositioncommandclamp) | Bring the target inside the joint limits |
| [`JointImpedanceCommand::clamp()`](08_cpp_api_reference/types_command.md#jointimpedancecommandclamp) | Bring the target inside the joint limits |
| [`to_command_mode()`](08_cpp_api_reference/types_command.md#to_command_mode) | Take the `CommandMode` out of a variant |

**[`types/state.hpp`](08_cpp_api_reference/types_state.md)**

| Function | Role |
|---|---|
| [`to_string(HandLifecycle)`](08_cpp_api_reference/types_state.md#enum-handlifecycle) | The lifecycle name |
| [`to_string(HomingState)`](08_cpp_api_reference/types_state.md#enum-homingstate) | The homing state name |
| [`to_string(ActuatorFault)`](08_cpp_api_reference/types_state.md#enum-actuatorfault) | The actuator error code name |

**[`types/error.hpp`](08_cpp_api_reference/types_error.md)**

| Function | Role |
|---|---|
| [`to_string(ErrorCode)`](08_cpp_api_reference/types_error.md#enum-errorcode) | The error code name |
| [`Exception::Exception()`](08_cpp_api_reference/types_error.md#exceptionexception) | Build an exception from a code and a message |
| [`Exception::code()`](08_cpp_api_reference/types_error.md#exceptioncode) | Returns the `ErrorCode` |

**[`logging/logging.hpp`](08_cpp_api_reference/logging.md)**

| Function | Role |
|---|---|
| [`set_log_level()`](08_cpp_api_reference/logging.md#set_log_level) | The console sink level |
| [`set_log_to_console()`](08_cpp_api_reference/logging.md#set_log_to_console) | Turn the console sink on or off |
| [`set_log_to_file()`](08_cpp_api_reference/logging.md#set_log_to_file) | The rotating file sink |
| [`set_log_callback()`](08_cpp_api_reference/logging.md#set_log_callback) | Relay to an application logger |
| [`flush_log()`](08_cpp_api_reference/logging.md#flush_log) | Send out the log that remains |

## Type index

Every enum, struct, class and alias, grouped by the file that declares it. A nested type carries
`::`.

**[`hand/hand_manager.hpp`](08_cpp_api_reference/hand_manager.md)**

| Type | Kind | Role |
|---|---|---|
| [`HandManager`](08_cpp_api_reference/hand_manager.md#handmanager) | class | Move-only type that creates and owns a hand's resources |

**[`hand/hand.hpp`](08_cpp_api_reference/hand.md)**

| Type | Kind | Role |
|---|---|---|
| [`Hand`](08_cpp_api_reference/hand.md#hand) | class | The handle that connects, controls and queries |

**[`types/description.hpp`](08_cpp_api_reference/types_description.md)**

| Type | Kind | Role |
|---|---|---|
| [constants](08_cpp_api_reference/types_description.md#constants) | `constexpr` | The actuator, joint and tactile counts |
| [`HandSide`](08_cpp_api_reference/types_description.md#enum-handside) | enum | Left or right |
| [`Finger`](08_cpp_api_reference/types_description.md#enum-finger) | enum | The tactile finger index |

**[`types/config.hpp`](08_cpp_api_reference/types_config.md)**

| Type | Kind | Role |
|---|---|---|
| [`HandConfig`](08_cpp_api_reference/types_config.md#handconfig) | struct | The settings that you pass to [`HandManager::create()`](08_cpp_api_reference/hand_manager.md#handmanagercreate) |
| [`ControllerConfig`](08_cpp_api_reference/types_config.md#controllerconfig) | struct | The tuning that [`Hand::set_controller_config()`](08_cpp_api_reference/hand.md#handset_controller_config) changes |
| [`ControllerConfig::JointPositionController`](08_cpp_api_reference/types_config.md#controllerconfigjointpositioncontroller) | struct | The position filter settings |
| [`ControllerConfig::JointImpedanceController`](08_cpp_api_reference/types_config.md#controllerconfigjointimpedancecontroller) | struct | The impedance gain settings |

**[`types/command.hpp`](08_cpp_api_reference/types_command.md)**

| Type | Kind | Role |
|---|---|---|
| [`CommandMode`](08_cpp_api_reference/types_command.md#enum-commandmode) | enum | The active controller |
| [`Idle`](08_cpp_api_reference/types_command.md#idle) | struct | No torque |
| [`JointPositionCommand`](08_cpp_api_reference/types_command.md#jointpositioncommand) | struct | Joint position target |
| [`JointImpedanceCommand`](08_cpp_api_reference/types_command.md#jointimpedancecommand) | struct | Joint impedance target |
| [`ActuatorPositionCommand`](08_cpp_api_reference/types_command.md#actuatorpositioncommand) | struct | Actuator position target |
| [`ActuatorEffortCommand`](08_cpp_api_reference/types_command.md#actuatoreffortcommand) | struct | Actuator effort target |
| [`ControllerCommand`](08_cpp_api_reference/types_command.md#type-aliases) | alias | The variant that holds one command |

**[`types/state.hpp`](08_cpp_api_reference/types_state.md)**

| Type | Kind | Role |
|---|---|---|
| [`HandLifecycle`](08_cpp_api_reference/types_state.md#enum-handlifecycle) | enum | The 5 states of a `Hand` |
| [`HomingState`](08_cpp_api_reference/types_state.md#enum-homingstate) | enum | The origin state |
| [`ActuatorFault`](08_cpp_api_reference/types_state.md#enum-actuatorfault) | enum | The actuator error code |
| [`CommandSource`](08_cpp_api_reference/types_state.md#enum-commandsource) | enum | The source that this cycle used |
| [`HandState`](08_cpp_api_reference/types_state.md#handstate) | struct | What [`Hand::get_state()`](08_cpp_api_reference/hand.md#handget_state) returns |
| [`ActuatorState`](08_cpp_api_reference/types_state.md#actuatorstate) | struct | The actuator observations |
| [`JointState`](08_cpp_api_reference/types_state.md#jointstate) | struct | The FK result |
| [`TactileState`](08_cpp_api_reference/types_state.md#tactilestate) | struct | The tactile readings |
| [`CommandedState`](08_cpp_api_reference/types_state.md#commandedstate) | struct | The command input and output of this cycle |
| [`ActuatorPositionSetpoint` / `ActuatorEffortSetpoint`](08_cpp_api_reference/types_state.md#actuatorpositionsetpoint--actuatoreffortsetpoint) | struct | The controller output setpoint |
| [`ActuatorHealth`](08_cpp_api_reference/types_state.md#actuatorhealth) | struct | The per-actuator enable and fault |
| [`ControllerOutput`](08_cpp_api_reference/types_state.md#type-aliases) | alias | The setpoint variant |

**[`types/diagnostics.hpp`](08_cpp_api_reference/types_diagnostics.md)**

| Type | Kind | Role |
|---|---|---|
| [`Diagnostics`](08_cpp_api_reference/types_diagnostics.md#diagnostics) | struct | What [`Hand::get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics) returns |

**[`types/error.hpp`](08_cpp_api_reference/types_error.md)**

| Type | Kind | Role |
|---|---|---|
| [`ErrorCode`](08_cpp_api_reference/types_error.md#enum-errorcode) | enum | The failure classification |
| [`Exception`](08_cpp_api_reference/types_error.md#exception--stdruntime_error) | class | The exception that a public call throws |

**[`logging/logging.hpp`](08_cpp_api_reference/logging.md)**

| Type | Kind | Role |
|---|---|---|
| [`LogLevel`](08_cpp_api_reference/logging.md#enum-loglevel) | enum | The log level |
| [`LogCallback`](08_cpp_api_reference/logging.md#type-aliases) | alias | The callback shape |

**[`version.hpp`](08_cpp_api_reference/version.md)**

| Type | Kind | Role |
|---|---|---|
| version constants | `constexpr` | `kVersionMajor` · `kVersionMinor` · `kVersionPatch` · `kVersionString` |
