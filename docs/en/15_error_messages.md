# Error messages and remedies

Look up the error code that the SDK puts on an exception or a log line (`E401` and so on), and find the
cause and what to do about it. For call order and background see the
[C++ guide](07_cpp_usage_guide.md); for symbols and fields see the
[API reference](08_cpp_api_reference.md). For how to record and send the log, see
[Logging](09_cpp_logging.md).

## Contents

&nbsp;&nbsp;[**1. Reading a message**](#1-reading-a-message)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.1 Message format](#11-message-format)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.2 Code ranges](#12-code-ranges)<br>
&nbsp;&nbsp;[**2. E1xx `InvalidArgument`**](#2-e1xx-invalidargument)<br>
&nbsp;&nbsp;[**3. E2xx `WrongCallOrder`**](#3-e2xx-wrongcallorder)<br>
&nbsp;&nbsp;[**4. E3xx `InterfaceUnavailable`**](#4-e3xx-interfaceunavailable)<br>
&nbsp;&nbsp;[**5. E4xx `CommunicationLost`**](#5-e4xx-communicationlost)<br>
&nbsp;&nbsp;[**6. E5xx `HardwareFault`**](#6-e5xx-hardwarefault)<br>
&nbsp;&nbsp;[**7. E6xx `HandFaulted`**](#7-e6xx-handfaulted)<br>
&nbsp;&nbsp;[**8. E7xx `ControlLoopFault`**](#8-e7xx-controlloopfault)<br>
&nbsp;&nbsp;[**9. E9xx `UnexpectedError`**](#9-e9xx-unexpectederror)<br>
&nbsp;&nbsp;[**10. Actuator fault values**](#10-actuator-fault-values)

## 1. Reading a message

### 1.1 Message format

The `what()` of [`Exception`](08_cpp_api_reference/types_error.md#exception--stdruntime_error) and the
error lines of the log share one shape.

```text
[<side>] [E<code>] cannot <action>: <reason> (<detail>)    the what() of an exception
[<side>] [E<code>] <event> (<detail>)                      an event that appears in the log only
```

The messages under each code below leave out `[<side>]` and start from the code. Each code lists its
cause, state and remedy, split per function, as in "If `stop()` failed", where they differ. The E403
and E503 of `destroy()` write the same message to the log instead of throwing it.

### 1.2 Code ranges

| [`ErrorKind`](08_cpp_api_reference/types_error.md#enum-errorkind) | What it means | Code |
|---|---|---|
| `InvalidArgument` | Invalid argument or config value | E1xx |
| `WrongCallOrder` | Call not allowed in the current lifecycle | E2xx |
| `InterfaceUnavailable` | The CAN interface could not be opened | E3xx |
| `CommunicationLost` | CAN communication failure (no RX or TX failure) | E4xx |
| `HardwareFault` | A drive did not reach the requested state, or reported a fault | E5xx |
| `HandFaulted` | Call rejected because the lifecycle is `Faulted` | E6xx |
| `ControlLoopFault` | An SDK thread failed to start, or exited on an exception | E7xx |
| `UnexpectedError` | Unexpected exception or internal state error | E9xx |

If the remedy of a code does not solve the problem, or the same error keeps coming back, contact us
with the SDK log and the `what()` the application recorded. For a code marked as an SDK internal error,
contact us right away. The issue tracker to report through is in
[SDK build & install](06_sdk_build_and_install.md).

## 2. E1xx `InvalidArgument`

A call that throws a code in this range changes nothing. Fix the value and call again. The
[`HandConfig`](08_cpp_api_reference/types_config.md#handconfig) constructor throws E101 to E103, and these
three do not appear in the log. A command with invalid values does not throw; the SDK does not apply it
and keeps sending the previous command ([C++ guide](07_cpp_usage_guide.md#522-rejected-by-value)).

```text
[E101] cannot create HandConfig: interface_name must not be empty
```

- Cause: the first constructor argument is an empty string

```text
[E102] cannot create HandConfig: hand_side must be HandSide::Left or HandSide::Right
```

- Cause: the `hand_side` value is outside the enum

```text
[E103] cannot create HandConfig: hand_model must be HandModel::Gen2 or HandModel::Gen2B
```

- Cause: the `hand_model` value is outside the enum
- Remedy: convert a string with [`parse_hand_model()`](08_cpp_api_reference/types_description.md#parse_hand_model)

```text
[E104] cannot set max effort: limit contains NaN/Inf
```

- Cause: a NaN or Inf was passed to [`set_max_effort()`](08_cpp_api_reference/hand.md#handset_max_effort). The SDK does not reject values outside `[0, 2000]` but clamps them to that range

```text
[E105] cannot set controller config: joint position cutoff_freq and deadband must be finite and >= 0
```

- Cause: one of the two is non-finite or negative

```text
[E106] cannot set controller config: joint impedance stiffness must be finite and in [0, 1]
```

- Cause: a value in the `stiffness` array is non-finite or outside `[0, 1]`

```text
[E107] cannot set controller config: joint impedance cutoff_freq must be finite and >= 0
```

- Cause: `cutoff_freq` is non-finite or negative

## 3. E2xx `WrongCallOrder`

The call is not allowed in the current state. Except for E203, the call changes nothing. Read the
current lifecycle from [`get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics)`.lifecycle`,
and see the table in the [C++ guide](07_cpp_usage_guide.md#22-state-transition-calls) for which call goes
through in which state.

```text
[E201] cannot connect hand: already connected
```

- Cause: [`connect()`](08_cpp_api_reference/hand.md#handconnect) was called in `Running` or `Stopped`. In `Connected` it does nothing and succeeds
- Remedy: to connect again, call [`disconnect()`](08_cpp_api_reference/hand.md#handdisconnect), then `connect()`

```text
[E202] cannot <action>: not connected
```

- Cause: [`run()`](08_cpp_api_reference/hand.md#handrun), [`stop()`](08_cpp_api_reference/hand.md#handstop), [`home()`](08_cpp_api_reference/hand.md#handhome), [`reconnect()`](08_cpp_api_reference/hand.md#handreconnect), [`set_command()`](08_cpp_api_reference/hand.md#handset_command), [`set_tactile_bias()`](08_cpp_api_reference/hand.md#handset_tactile_bias) or [`reset_tactile_bias()`](08_cpp_api_reference/hand.md#handreset_tactile_bias) was called in `Disconnected`
- Remedy: call `connect()` first. Use `reconnect()` only to recover from `Faulted`

```text
[E203] cannot home hand: interrupted during homing
```

- Cause: another thread called `stop()` or `disconnect()` during the wait of `home()` for homing. When `auto_home` is `true`, `run()` also throws this message
- Remedy: to continue control, call `home()` again

```text
[E204] cannot <action>: not running
```

- Cause: `stop()` was called in `Connected`, or `set_command()` was called in `Connected` or `Stopped`, or before the enable was confirmed
- If `stop()` was rejected: there is nothing to stop, so leave it
- If `set_command()` was rejected: send it again after `run()` succeeds

```text
[E205] cannot set command: not homed
```

- Cause: a command other than [`Idle`](08_cpp_api_reference/types_command.md#idle) was sent before homing succeeded
- Remedy: send it again after `home()` succeeds

```text
[E206] cannot reconnect hand: not faulted
```

- Cause: `reconnect()` was called in `Connected`, `Running` or `Stopped`
- Remedy: call `reconnect()` only in `Faulted`. To connect again, call `disconnect()`, then `connect()`

```text
[E207] cannot use hand: hand is destroyed
```

- Cause: a destroyed hand was called. [`destroy()`](08_cpp_api_reference/hand_manager.md#handmanagerdestroy) or `destroy_all()` has already finished or is running on another thread, or the manager was destroyed before the handle
- Remedy: do not use a destroyed handle. Create a new one with [`create()`](08_cpp_api_reference/hand_manager.md#handmanagercreate) if you need one, and destroy the manager after its handles

## 4. E3xx `InterfaceUnavailable`

These come when `connect()` or `reconnect()` could not open the CAN interface. After a failed `connect()`
the `lifecycle` value stays `Disconnected`; after a failed `reconnect()` it stays `Faulted`. Fix the cause
and call the failed function again.

```text
[E301] cannot connect hand: CAN interface '<name>' not found
```

- Cause: no network interface has that name. The name is wrong, or the CAN adapter is not connected
- Remedy: check the name with `ip link` and match the `interface_name` of [`HandConfig`](08_cpp_api_reference/types_config.md#handconfig) to it

```text
[E302] cannot connect hand: permission denied opening CAN socket on '<name>'
```

- Cause: the OS refused to open a CAN socket. Opening a CAN socket does not need root, so the environment running the process (a container, or a security policy such as AppArmor or SELinux) is blocking it
- Remedy: check that the run environment allows CAN sockets

```text
[E303] cannot connect hand: CAN not supported by the kernel
```

- Cause: the kernel could not load its CAN modules (`can`, `can_raw`)
- Remedy: run `sudo modprobe can can_raw`. For a kernel you built yourself, check `CONFIG_CAN` and `CONFIG_CAN_RAW` in [1.5 Configure the kernel](04_real_time_kernel_setup.md#15-configure-the-kernel)

```text
[E304] cannot connect hand: kernel or interface '<name>' is not configured for CAN-FD (CAN_RAW_FD_FRAMES)
```

- Cause: the kernel does not accept the CAN-FD option of the CAN socket (`CAN_RAW_FD_FRAMES`)
- Remedy: with the command in [1.4 Verify the link](05_can_fd_setup.md#14-verify-the-link), check that the interface is up as CAN-FD (`<FD>`)

```text
[E305] cannot connect hand: CAN interface '<name>' is down
```

- Cause: the interface is DOWN
- Remedy: bring the interface up as in [1.3 Bring up that interface](05_can_fd_setup.md#13-bring-up-that-interface)

```text
[E306] cannot connect hand: CAN interface '<name>' could not be opened (<reason>)
```

- Cause: the interface could not be opened for a reason other than the ones above. `<reason>` is the error description that the OS returned
- Remedy: bring the interface up again as in [CAN-FD setup](05_can_fd_setup.md#1-bring-up-the-interface)

```text
[E307] cannot connect hand: another master is commanding this hand on <name>
```

- Cause: another program is sending commands to this robot hand. It can be a program on this computer or another computer on the same CAN bus
- Remedy: stop the other program and call the failed function again

## 5. E4xx `CommunicationLost`

A code in this range means that communication between the host and the robot hand is gone. Check the
following first.

1. The robot hand is powered on
2. The CAN cable is connected
3. The interface is up as CAN-FD (`<FD>`), checked with the command in [1.4 Verify the link](05_can_fd_setup.md#14-verify-the-link). If not, bring it up again as in [1.3 Bring up that interface](05_can_fd_setup.md#13-bring-up-that-interface)
4. `interface_name` and `hand_side` match the robot hand actually connected. The SDK receives frames only from the side that `hand_side` names

After you restore communication, make the call listed under each code again.

> [!WARNING]
> The SDK cannot send a quick stop when communication is gone, so the drives may hold the last
> command.

```text
[E401] cannot connect hand: no data received from <name> within 300 ms
```

- Cause: `connect()` or `reconnect()` received no frame from this robot hand within 300 ms. `reconnect()` throws the same message
- If `connect()` failed: the `lifecycle` value stays `Disconnected`. Call `connect()` again
- If `reconnect()` failed: the `lifecycle` value stays `Faulted`. Call `reconnect()` again

```text
[E402] cannot <action>: CAN communication failed
```

- Cause: communication was gone at the call of [`run()`](08_cpp_api_reference/hand.md#handrun), [`home()`](08_cpp_api_reference/hand.md#handhome) or [`set_tactile_bias()`](08_cpp_api_reference/hand.md#handset_tactile_bias), or during the wait of `run()` for the enable
- If `run()` failed during its wait for the enable: the SDK requests a quick stop
- If `set_tactile_bias()` failed: the bias stays as it was
- Remedy: if the `lifecycle` value becomes `Faulted`, call `reconnect()`; otherwise call the failed function again

```text
[E403] cannot <action>: quick stop not confirmed (CAN communication failed)
```

- Cause: the SDK requested a quick stop but sending failed, so it could not confirm the stop within 500 ms
- If `stop()` failed: the `lifecycle` value does not change. Call `stop()` again, and power the robot hand off if you cannot restore communication right away
- If `disconnect()` failed: the connection stays open. Call `disconnect()` again, or call `destroy()` if you must disconnect anyway
- For `destroy()`: it finishes destroying the hand without an exception, and writes the same message, starting with `cannot stop hand`, to the log

```text
[E404] communication lost (no RX)
```

- Cause: the control and communication loop received no frame from this robot hand for 100 ms and set the `lifecycle` value to `Faulted`. The log line appears one time at that moment
- State: `Faulted`, with `404` in the `fault_code` field. From then on the SDK sends nothing to the robot hand, and later calls are refused with [E601](#7-e6xx-handfaulted)
- Remedy: restore communication, then call `reconnect()`. When `auto_reconnect` is `true`, the SDK retries on its own

When `, last RX error: conflicting command` follows `no RX` in the log line, another program sent
commands to the same robot hand at some point after the connection. There is no guarantee that it
caused this loss. Stop the other program, then call `reconnect()`.

```text
[E405] CAN TX failed (errno <number>: <text>)
```

- Cause: sending failed in `Running`, and the control loop set the `lifecycle` value to `Faulted`. `(errno …)` is the error that the OS returned, and it can be missing
- State: `Faulted`, with `405` in the `fault_code` field. From then on the SDK sends nothing to the robot hand, and later calls are refused with [E601](#7-e6xx-handfaulted)
- Remedy: restore communication, then call `reconnect()`

```text
[E406] auto-reconnect timed out
```

- Cause: `auto_reconnect` is `true`, and communication did not come back within `auto_reconnect_timeout_ms`
- State: stays `Faulted`, and the automatic retry stops
- Remedy: restore communication, then call `reconnect()` yourself

## 6. E5xx `HardwareFault`

These come when communication is working but the drives could not finish the request. First check
whether the `actuator_health.fault` field of
[`get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics) reports a fault on any actuator
([10. Actuator fault values](#10-actuator-fault-values)), then make the call listed under each code again.

```text
[E501] cannot run hand: drives did not reach operation enabled within 4000 ms
```

- Cause: not every actuator without a fault was enabled within 4000 ms, or every actuator is in fault
- State: the SDK requests a quick stop, and the hand becomes `Stopped` after the quick stop is confirmed
- Remedy: call [`run()`](08_cpp_api_reference/hand.md#handrun) again

```text
[E502] homing failed at step (<k>/3) <step>: <reason> (actuators <list>)
[E502] cannot home hand: homing failed (actuators <list>)
```

- Cause: homing could not finish one of its steps. The log line names the step and the reason (table below). `<list>` is the failed actuator indices joined by commas. When no actuator failed, the exception shows `none` and the log line leaves out the parentheses as well
- State: the `homing_state` value becomes `Failed`. When `auto_home` is `true`, `run()` also throws this message
- Remedy: call [`home()`](08_cpp_api_reference/hand.md#handhome) again

| Step | `<reason>` | Meaning |
|---|---|---|
| `(1/3) preparing actuators` | `no actuator could be brought under control` | No actuator was enabled within the time limit, or every actuator is listed in the `disabled_actuators` field |
| `(1/3) preparing actuators` | `some actuators could not be prepared for homing` | The preparation for homing did not finish within the time limit |
| `(3/3) setting origin` | `some actuators reported a homing error` | A drive reported a homing error |
| `(3/3) setting origin` | `some actuators did not finish homing` | A drive did not report completion within the time limit after homing started |

When only some actuators respond in the enable step, the SDK leaves the others out of homing, and they
do not appear in `<list>` either.

```text
[E503] cannot <action>: drives did not reach quick stop within 500 ms
```

- Cause: communication is working, but the drives did not reach quick stop within 500 ms
- If `stop()` failed: the `lifecycle` value does not change. Call `stop()` again or power the robot hand off
- If `disconnect()` failed: the connection stays open. Power the robot hand off, or call `destroy()` if you must disconnect anyway
- For `destroy()`: it finishes destroying the hand without an exception, and writes the same message, starting with `cannot stop hand`, to the log

> [!WARNING]
> When a stop ends without confirmation, the drives may hold the last command.

```text
[E504] actuator <index> fault set (<name> 0x<code>)
```

- Cause: a drive reported an actuator fault. `<name>` is the name of the [`ActuatorFault`](08_cpp_api_reference/types_state.md#enum-actuatorfault) value, and `0x<code>` is the code that the drive sent
- State: the `lifecycle` value does not change. The SDK excludes that actuator when it applies commands, and the control loop keeps trying to reset the fault. When the fault clears, `actuator <index> fault cleared` appears at the info level
- Remedy: the application decides how to respond ([C++ guide](07_cpp_usage_guide.md#81-actuator-faults))

## 7. E6xx `HandFaulted`

These mean the call was refused because the robot hand is `Faulted`. The cause that made it `Faulted`
is in the log line written before, and in the `fault_code` field of
[`Diagnostics`](08_cpp_api_reference/types_diagnostics.md#diagnostics).

```text
[E601] cannot <action>: hand faulted (E<code> <event>)
```

- Cause: a function other than `reconnect()` was called while the `lifecycle` value is `Faulted`. The parentheses hold the code of the event that made the hand `Faulted` and the words its log line uses. When no cause was recorded, the message ends with `hand faulted` and no parentheses, which is an SDK internal error
- State: stays `Faulted`
- Remedy: follow the remedy of the code in the parentheses ([E404](#5-e4xx-communicationlost), [E405](#5-e4xx-communicationlost), [E702](#8-e7xx-controlloopfault)), then call `reconnect()`

## 8. E7xx `ControlLoopFault`

```text
[E701] cannot connect hand: failed to start SDK threads (<reason>)
```

- Cause: the SDK could not create its threads. `<reason>` is the error that the OS returned
- If `connect()` failed: the `lifecycle` value stays `Disconnected`
- If `reconnect()` failed: the `lifecycle` value stays `Faulted`
- Remedy: check the thread and memory limits of the process, then call the failed function again

```text
[E702] control loop terminated, quick stop sent without confirmation (<reason>)
```

- Cause: the control and communication loop ended with an exception. This is an SDK internal error
- State: `Faulted`, with `702` in the `fault_code` field. The SDK sends a quick stop but cannot confirm it. Even when `auto_reconnect` is `true`, the SDK does not recover on its own, and later calls are refused with [E601](#7-e6xx-handfaulted)
- Remedy: call `reconnect()` yourself

```text
[E703] event logging thread terminated (<reason>)
```

- Cause: the thread that moves the events of the control loop into the log ended with an exception. This is an SDK internal error
- State: control goes on, but later events (communication loss, actuator faults, homing failures and so on) do not reach the log
- Remedy: watch the state through [`get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics)

## 9. E9xx `UnexpectedError`

```text
[E901] cannot <action>: unexpected failure (<detail>)
```

- Cause: an unclassified exception. This is an SDK internal error. `<action>` is the call that failed and `<detail>` is the content of the exception; when the content is unknown, it reads `unknown exception`

```text
[E902] cannot connect hand: CAN transport already open on '<name>'
```

- Cause: an SDK internal state is not what it expects. This is an SDK internal error. It also appears when two threads call `connect()` on the same hand at the same time
- Remedy: call a `Hand` and its copies from one thread only

## 10. Actuator fault values

The `actuator_health.fault` field of [`get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics)
is the per-actuator fault that the drives report. The value stays there even when nothing is thrown,
so record it as well. The value is an [`ActuatorFault`](08_cpp_api_reference/types_state.md#enum-actuatorfault)
enum, and `None` means that actuator has no fault. When a fault appears, an [E504](#6-e5xx-hardwarefault)
log line is written too. An example that scans the whole array is in the
[C++ guide](07_cpp_usage_guide.md#81-actuator-faults).

| Value | Code |
|---|---|
| `OverVoltageError` · `UnderVoltageError` | `0x3210` · `0x3220` |
| `OverCurrentError` · `OverLoadError` | `0x2310` · `0xFF02` |
| `OverTemperatureError` | `0x4210` |
| `CurrentDetectionError` | `0x5210` |
| `FollowingError` · `SpeedError` | `0x8611` · `0x7310` |
| `CommunicationError` | `0x7500` |
| `HallSensorError` | `0xFF01` |
| `PositiveLimitSwitchError` · `NegativeLimitSwitchError` | `0xFF03` · `0xFF04` |
| `EmergencySwitchError` · `Sto1Error` · `Sto2Error` | `0xFF05` · `0xFF06` · `0xFF07` |
| `SerialEncoderChannelAError` · `SerialEncoderChannelBError` | `0xFF24` · `0xFF26` |
| `SerialEncoderChannelADisconnectedError` · `SerialEncoderChannelBDisconnectedError` | `0xFF25` · `0xFF27` |
| A value not in the table | The drive's code comes through unchanged. `to_string()` returns `"Unknown"`, and the E504 log line shows `UnknownError` |
