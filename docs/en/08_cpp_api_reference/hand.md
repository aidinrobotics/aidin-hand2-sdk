[← C++ API Reference](../08_cpp_api_reference.md)

# `hand/hand.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [`Hand`](#hand) | class | The handle used to connect, control, and query |

## `Hand`

You never create or destroy one directly. Get it from
[`HandManager::create()`](hand_manager.md#handmanagercreate) and release it with
[`HandManager::destroy()`](hand_manager.md#handmanagerdestroy). A copy refers to the same hand.

Every method throws [`Exception`](types_error.md#exception--stdruntime_error) on failure. The
[C++ guide](../07_cpp_usage_guide.md#2-lifecycle) tabulates which state accepts which call.

| Group | Member | Description |
|---|---|---|
| Connection | [`connect()`](#handconnect) | Open the CAN socket and start receiving |
| | [`disconnect()`](#handdisconnect) | Stop it and close |
| | [`reconnect()`](#handreconnect) | Rebuild the link after a fault |
| Operation | [`run()`](#handrun) | Enable the drives and start control |
| | [`stop()`](#handstop) | Quick stop |
| | [`home()`](#handhome) | Home, waiting for the end |
| Command | [`set_command()`](#handset_command) | Latch a command and pick the controller |
| Config | [`set_max_effort()`](#handset_max_effort) | Actuator current ceiling |
| | [`set_controller_config()`](#handset_controller_config) | Filter and impedance gains |
| Observation | [`get_state()`](#handget_state) | What the SDK read from the robot hand |
| | [`get_diagnostics()`](#handget_diagnostics) | SDK and control and communication loop state |
| | [`get_command_mode()`](#handget_command_mode) | The active mode |

### `Hand::connect()`

```cpp
void connect();
```

Opens the CAN socket and starts receiving state. It applies no torque.

**Throws**         ｜ `InterfaceUnavailable` · `CommunicationLost` ·
`ControlLoopFault` · `WrongCallOrder`<br>
**Preconditions**  ｜ `Disconnected` · `Connected` (no-op)<br>
**Postconditions** ｜ `Connected`<br>
**Notes**          ｜ Waits up to 300 ms for the first frame

### `Hand::disconnect()`

```cpp
void disconnect();
```

Requests a quick stop from the actuators and then closes the CAN connection. If control was ever
requested, it confirms that the actuators reached the quick stop state.

**Throws**         ｜ `CommunicationLost` · `HardwareFault` ·
`ControlLoopFault` · `WrongCallOrder`<br>
**Preconditions**  ｜ `Disconnected` (no-op) · `Connected` · `Running` · `Stopped`<br>
**Postconditions** ｜ `Disconnected`<br>
**Notes**          ｜ Waits up to 500 ms for the confirmation. If it cannot confirm, it throws
and does not close the connection, so call [`destroy()`](hand_manager.md#handmanagerdestroy) when you must
disconnect anyway. If a fault occurs while it confirms, it does not wait for the time limit and
returns the cause of that fault as an exception

### `Hand::reconnect()`

```cpp
void reconnect();
```

Reconnects after a fault. It resets `homing_state` and the stored command.

- `homing_state` → `NotRun`
- `max_effort`, [`ControllerConfig`](types_config.md#controllerconfig) → kept
- The stored command → [`Idle`](types_command.md#idle), the same as automatic reconnection

**Throws**         ｜ `InterfaceUnavailable` · `CommunicationLost` ·
`ControlLoopFault` · `WrongCallOrder`<br>
**Preconditions**  ｜ `Faulted`<br>
**Postconditions** ｜ `Connected`<br>
**Notes**          ｜ Tries the reconnection once and waits up to 300 ms for the first state. On
failure it stays `Faulted`

### `Hand::run()`

```cpp
void run();
```

Enables the actuators and returns after confirming that every actuator without a fault reached
Operation Enabled. It excludes an actuator that has a fault from the confirmation and keeps
retrying the fault reset on it. With [`auto_home`](types_config.md#handconfig)`=true` and [`homing_state`](types_state.md#enum-homingstate) other than
`Succeeded`, it homes first.

**Throws**         ｜ `CommunicationLost` · `HardwareFault` ·
`ControlLoopFault` · `WrongCallOrder`<br>
**Preconditions**  ｜ `Connected` · `Running` (no-op) · `Stopped`<br>
**Postconditions** ｜ `Running`<br>
**Notes**          ｜ Waits up to 4000 ms for the actuator enable, and on the path that homes it
waits for homing to finish. If it cannot confirm, it clears the command to [`Idle`](types_command.md#idle),
requests a quick stop from the actuators and throws, so the hand becomes `Stopped` once that quick
stop is confirmed. Resuming from `Stopped` inserts one command that holds the pose at the moment of
the resume

### `Hand::stop()`

```cpp
void stop();
```

Requests a quick stop and confirms that the actuators reached the quick stop state. The hand does
not become `Stopped` before that confirmation. It resets the last command to [`Idle`](types_command.md#idle).

**Throws**         ｜ `CommunicationLost` · `HardwareFault` ·
`ControlLoopFault` · `WrongCallOrder`<br>
**Preconditions**  ｜ `Running` · `Stopped` (no-op)<br>
**Postconditions** ｜ `Stopped`<br>
**Notes**          ｜ Waits up to 500 ms for the confirmation. If it cannot confirm, it throws and
stays `Running`, so the actuators may still hold the last command. A second call judges again
against the state at the moment of that call. If a fault occurs while it confirms, it does not wait
for the time limit and returns the cause of that fault as an exception

### `Hand::home()`

```cpp
void home();
```

Starts homing and waits for it to end. It enables the drives internally, so there is no need to
call [`run()`](#handrun) first.

**Throws**         ｜ `HardwareFault` (naming the actuators that failed) · `CommunicationLost` ·
`ControlLoopFault` ·
`WrongCallOrder`<br>
**Preconditions**  ｜ `Connected` · `Running` · `Stopped`<br>
**Postconditions** ｜ `Running`, `homing_state = Succeeded`<br>
**Notes**          ｜ The fingers travel to their hard stops. The SDK clears the stored command
to [`Idle`](types_command.md#idle)

### `Hand::set_command()`

```cpp
void set_command(const Idle& command);
void set_command(const JointPositionCommand& command);
void set_command(const JointImpedanceCommand& command);
void set_command(const ActuatorPositionCommand& command);
void set_command(const ActuatorEffortCommand& command);
```

The argument type picks the controller, and what you set stays in effect until the next call. A
joint command goes through the workspace projection automatically. The range model is in
[Workspace limits](../14_workspace_limits.md).

**Parameters**     ｜ `command` — one of the five command types<br>
**Throws**         ｜ `CommunicationLost` · `ControlLoopFault` ·
`WrongCallOrder` (not `Running`, or `homing_state != Succeeded`, from which [`Idle`](types_command.md#idle) is exempt)<br>
**Preconditions**  ｜ `Running` — the actuator enable must be confirmed, so this comes after
[`run()`](#handrun) succeeds<br>
**Notes**          ｜ A bad value does not throw. The previous command keeps going out,
`Diagnostics::nan_command_count` increments and the SDK logs a warning. See [Validation](types_command.md#validation)

### `Hand::set_max_effort()`

```cpp
void set_max_effort(double limit);
void set_max_effort(const std::array<double, kActuatorCount>& limit);
```

Sets the actuator current ceiling. The `double` overload applies to every actuator, the array
overload sets them individually.

**Parameters**     ｜ `limit` — percent of rated current (`1000`=100 %). Values outside
`[0, 2000]` are
clipped<br>
**Throws**         ｜ `InvalidArgument` (NaN or Inf) · `WrongCallOrder` (destroyed `HandCore` or
invalidated handle)<br>
**Notes**          ｜ Survives [`reconnect()`](#handreconnect)

### `Hand::set_controller_config()`

```cpp
void set_controller_config(const ControllerConfig& config);
```

Sets the filter and the impedance gains. Callable at any time, applied from the next cycle.

**Parameters**     ｜ `config` — [`ControllerConfig`](types_config.md#controllerconfig)<br>
**Throws**         ｜ `WrongCallOrder` (destroyed `HandCore` or invalidated handle) ·
`InvalidArgument` — one of `cutoff_freq`, `deadband`, `stiffness`, `damping` is
non-finite or negative<br>
**Notes**          ｜ On failure the SDK keeps the previous settings. There is no getter, so
the application keeps
the values it applied

### `Hand::get_state()`

```cpp
[[nodiscard]] HandState get_state() const;
```

Returns the latest values that the SDK read from the robot hand.

**Returns**        ｜ [`HandState`](types_state.md#handstate)<br>
**Throws**         ｜ `WrongCallOrder` (destroyed `HandCore` or invalidated handle)<br>
**Notes**          ｜ The values from one call all come from the same cycle. Reuse the result
of a single call
when comparing them. It is not one transaction with
[`get_diagnostics()`](#handget_diagnostics)

### `Hand::get_diagnostics()`

```cpp
[[nodiscard]] Diagnostics get_diagnostics() const;
```

Returns the latest state of the SDK and the control loop.

**Returns**        ｜ [`Diagnostics`](types_diagnostics.md#diagnostics)<br>
**Throws**         ｜ `WrongCallOrder` (destroyed `HandCore` or invalidated handle)<br>
**Notes**          ｜ One call mixes two moments — `lifecycle`, `homing_state` and
`nan_command_count` are read
at call time, the rest come from the last cycle the control loop published

### `Hand::get_command_mode()`

```cpp
[[nodiscard]] CommandMode get_command_mode() const;
```

Returns the controller mode currently active.

**Returns**        ｜ [`CommandMode`](types_command.md#enum-commandmode)<br>
**Throws**         ｜ `WrongCallOrder` (invalidated handle)

---
