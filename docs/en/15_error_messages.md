# Error messages and remedies

Look up a message the SDK threw and find what to do about it. For call order and background see the
[C++ guide](07_cpp_usage_guide.md); for symbols and fields see the
[API reference](08_cpp_api_reference.md). Where a remedy below asks you to keep the log, see
[Logging](09_cpp_logging.md).

## Contents

&nbsp;&nbsp;[**1. Reading a message**](#1-reading-a-message)<br>
&nbsp;&nbsp;[**2. ErrorCode summary**](#2-errorcode-summary)<br>
&nbsp;&nbsp;[**3. `InvalidArgument`**](#3-invalidargument)<br>
&nbsp;&nbsp;[**4. `WrongCallOrder`**](#4-wrongcallorder)<br>
&nbsp;&nbsp;[**5. `InterfaceUnavailable`**](#5-interfaceunavailable)<br>
&nbsp;&nbsp;[**6. `CommunicationLost`**](#6-communicationlost)<br>
&nbsp;&nbsp;[**7. `HardwareFault`**](#7-hardwarefault)<br>
&nbsp;&nbsp;[**8. `ControlLoopFault`**](#8-controlloopfault)<br>
&nbsp;&nbsp;[**9. `UnexpectedError`**](#9-unexpectederror)<br>
&nbsp;&nbsp;[**10. Faulted cause phrases**](#10-faulted-cause-phrases)<br>
&nbsp;&nbsp;[**11. Actuator faults**](#11-actuator-faults)

## 1. Reading a message

The `what()` of [`Exception`](08_cpp_api_reference/types_error.md#exception--stdruntime_error) carries the
message only; the error code is not attached. Record both — there is an
example in [9.1 Exception of the C++ guide](07_cpp_usage_guide.md#91-exception).

Most messages take the form `Cannot <action>: <reason>`, and the tables in this document are keyed
on the `<reason>` part. A `<...>` marks a place that takes a value at run time.

When `<reason>` is `hand faulted (<cause>)`, the `lifecycle` value is already `Faulted` — the cause
phrases are in [10. Faulted cause phrases](#10-faulted-cause-phrases). In that state the SDK
rejects every call that changes the lifecycle until
[`reconnect()`](08_cpp_api_reference/hand.md#handreconnect) runs, and it still allows the calls that the
[C++ guide](07_cpp_usage_guide.md#22-state-transition-calls) classifies as configuration and
observation.

The message of the exception that the rejected call throws carries the cause of the stop. Whether
the cause also reaches the log depends on the cause.

- A control and communication loop exception → it reaches the log directly as well
- A communication error → it can drop out of the log, so take the message of the exception as your
  evidence

When the SDK rejects a call in a state other than `Faulted`, the remedy differs per code, so follow
the Remedy column of the table in each section below.

## 2. ErrorCode summary

| [`ErrorCode`](08_cpp_api_reference/types_error.md#enum-errorcode) | What it means | Look at |
|---|---|---|
| `InvalidArgument` | A value broke a rule | The calling code |
| `WrongCallOrder` | Not callable in the current state | The calling code (lifecycle) |
| `InterfaceUnavailable` | The CAN interface could not be opened | Interface name, permissions, ownership |
| `CommunicationLost` | Communication is gone | Power, wiring, bus state |
| `HardwareFault` | The device could not finish the request | Drive faults, mechanics |
| `ControlLoopFault` | The control and communication loop terminated abnormally | Report to the SDK |
| `UnexpectedError` | An unclassified failure | Report to the SDK |

The issue tracker to report through is in [SDK build & install](06_sdk_build_and_install.md).

## 3. `InvalidArgument`

| Message | Cause | Remedy |
|---|---|---|
| `HandConfig: interface_name must not be empty` | First constructor argument is an empty string | Pass a CAN interface name |
| `HandConfig: hand_side must be HandSide::Left or HandSide::Right` | `hand_side` out of range | Use the enum values |
| `Cannot create hand: control_rate must be positive` | `control_rate` is `0` or less | Set a positive value. The default is `500` |
| `Cannot set max effort: limit contains NaN/Inf` | A NaN or Inf passed to [`set_max_effort()`](08_cpp_api_reference/hand.md#handset_max_effort) | Validate the values. The accepted range is `[0, 2000]` |
| `Cannot set controller config: joint position cutoff_freq and deadband must be finite and >= 0` | One of the two is non-finite or negative | The previous settings are kept. Fix the value and call again |
| `Cannot set controller config: joint impedance stiffness and damping must be finite and >= 0` | A non-finite or negative gain in the arrays | Same as above |

A rejected command is dropped whole and **the previous one keeps going out.**
[`set_command()`](08_cpp_api_reference/hand.md#handset_command) does not throw on a bad value. It leaves
a warning log and holds the previous command, so detection goes through
[`get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics)`.nan_command_count` — see the
[C++ guide](07_cpp_usage_guide.md#522-rejected-by-value).

## 4. `WrongCallOrder`

The state names in the Situation column are [`HandLifecycle`](08_cpp_api_reference/types_state.md#enum-handlifecycle)
values, and the [C++ guide](07_cpp_usage_guide.md#22-state-transition-calls) tabulates which call goes
through in which state.

These are the reasons appended to `Cannot <action>: `. Read the current lifecycle from
[`get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics)`.lifecycle`.

| Reason | Situation | Remedy |
|---|---|---|
| `already connected — call disconnect() first to rebuild the link` | [`connect()`](08_cpp_api_reference/hand.md#handconnect) in `Running` or `Stopped` | Call [`disconnect()`](08_cpp_api_reference/hand.md#handdisconnect), then `connect()`, to rebuild communication |
| `not connected — call connect() first` | [`run()`](08_cpp_api_reference/hand.md#handrun), [`stop()`](08_cpp_api_reference/hand.md#handstop), or [`home()`](08_cpp_api_reference/hand.md#handhome) in `Disconnected` | Call `connect()` first |
| `not connected yet — call connect()` | `reconnect()` in `Disconnected` | `reconnect()` is for recovery; the first connection is `connect()` |
| `not connected yet — call connect(), then run()` | [`set_command()`](08_cpp_api_reference/hand.md#handset_command) in `Disconnected` | Connect and run, then send again |
| `control not running — call run() first` | `stop()` in `Connected` | Leave it as is. Control is not running, so there is nothing to stop |
| `not faulted — reconnect() recovers from Faulted only` | `reconnect()` outside `Faulted` | `reconnect()` is for fault recovery; other states do not need it |
| `not running — call run() first` | `set_command()` in `Connected` or `Stopped`, or before the enable is confirmed | Send after [`run()`](08_cpp_api_reference/hand.md#handrun) succeeds |
| `not homed — call home() first` | A non-[`Idle`](08_cpp_api_reference/types_command.md#idle) command before the origin is found | Send again after `home()` succeeds |
| `hand is destroyed — create a hand` | A handle to a destroyed `HandCore` | Create a new one with [`create()`](08_cpp_api_reference/hand_manager.md#handmanagercreate) |
| `Hand is no longer valid (destroyed or manager gone)` | The manager was destroyed first | Outlive every handle with the manager |
| `Cannot home hand: interrupted during homing — call run() and home() again` | Left `Running` during homing | Check the cause, then repeat |
| `CAN transport already open: <name> — close it before reopening` | This handle already has it open | `disconnect()` before reopening |

A rejection in `Faulted` carries the reason the hand stopped instead of a `WrongCallOrder` reason.
See [10. Faulted cause phrases](#10-faulted-cause-phrases).

## 5. `InterfaceUnavailable`

| Message | Cause | Remedy |
|---|---|---|
| `CAN interface '<name>' not found (check the name with `ip link`)` | No such interface | Compare the name with `ip link` and match [`HandConfig`](08_cpp_api_reference/types_config.md#handconfig) |
| `CAN interface '<name>' is down and cannot be configured (needs CAP_NET_ADMIN). Bring it up manually: sudo ip link set <name> type can bitrate 1000000 sample-point 0.875 sjw 10 …` | The interface is `DOWN` and the SDK may not bring it up | Run the command in the message, or follow [CAN-FD setup](05_can_fd_setup.md) |
| `CAN interface '<name>' not found (check the name with `ip link`, and that it exists)` | The interface vanished before the bind | Check the adapter connection |
| `permission denied opening CAN socket on '<name>' (need root or CAP_NET_RAW)` | Not allowed to create the socket | Grant `CAP_NET_RAW` or run as root |
| `CAN not supported by the kernel (try `modprobe can can_raw`)` | The CAN modules are not loaded | Run `modprobe can can_raw` |
| `kernel or interface '<name>' is not configured for CAN-FD (CAN_RAW_FD_FRAMES); check with `ip -details link show <name>`` | No CAN-FD support | Check with the command in the message and use a CAN-FD adapter |
| `failed to open CAN interface '<name>': <reason>` | Any other socket open failure | Read the errno description in `<reason>` |
| `failed to bring up CAN interface '<name>': <reason>` | Link up failed | Check the adapter connection and its driver |
| `failed to configure CAN interface '<name>': <reason>` | Bit timing could not be set | Check CAN-FD support and the timing values |
| `failed to set CAN error mask on <name>` | Socket option failed | Check the kernel's SocketCAN support |
| `failed to set CAN RX filters on <name>` | Filter setup failed | Same as above |
| `another master is commanding this hand on <name>` | Another process is driving the same hand | Leave exactly one running |

## 6. `CommunicationLost`

| Message | Cause | Remedy |
|---|---|---|
| `Cannot <action>: no data received from <name> within 300 ms — check hand power and CAN wiring` | [`connect()`](08_cpp_api_reference/hand.md#handconnect) received no first frame | Check power, wiring, termination, and bit rate, then call `connect()` again. The state stays `Disconnected` |
| `Cannot stop hand: cannot confirm quick stop — no RX on <name>; drives may hold last command. Restore CAN link and hand power` | [`stop()`](08_cpp_api_reference/hand.md#handstop) could not confirm the stop | Restore communication and power, or power the robot hand off. The state stays `Stopped` |
| `Cannot disconnect hand: cannot confirm stop — no RX on <name>; drives may hold last command. Use reconnect() or destroy` | [`disconnect()`](08_cpp_api_reference/hand.md#handdisconnect) could not confirm the stop | Same as above. Handle it with [`reconnect()`](08_cpp_api_reference/hand.md#handreconnect) or [`destroy()`](08_cpp_api_reference/hand_manager.md#handmanagerdestroy) |
| `Cannot run hand: cannot confirm drive enable — no RX on <name>; restore CAN link and hand power, then reconnect()` | [`run()`](08_cpp_api_reference/hand.md#handrun) could not confirm the drive enable | Restore communication and power, then call `reconnect()` |
| `Cannot enable hand: interface <name> is gone — reconnect the CAN adapter, then recover with reconnect()` | The interface disappeared | Reconnect the adapter. The hand faults, and `reconnect()` recovers it from there |
| `Cannot enable hand: interface <name> was re-created — recover with reconnect() once the hand faults` | Re-created under the same name, so the old socket is stale | Reception is gone, so the hand faults shortly. Call `reconnect()` then |
| `Cannot enable hand: communication not restored (no RX on <name>) — restore CAN link and hand power, then recover with reconnect()` | Still nothing received | Restore communication and power. The hand faults, and `reconnect()` recovers it from there |

A call rejected in `Faulted` does not appear in this table; it arrives as `hand faulted (<cause>)` —
see [10. Faulted cause phrases](#10-faulted-cause-phrases).

The SDK treats about 100 ms without RX as a communication error. A transient full TX queue
(`EAGAIN`, `EWOULDBLOCK`, `ENOBUFS`) is not treated as one.

While `Stopped` the control and communication loop keeps sending a quick stop every cycle, so the
drives receive it once communication is back. Calling `stop()` again does nothing and returns
success, because the hand is already `Stopped` — it is not a confirmation.

> [!WARNING]
> When a stop ends without confirmation, the drives may hold the last command.

## 7. `HardwareFault`

| Message | Cause | Remedy |
|---|---|---|
| `Cannot home hand: no actuator could be brought under control (actuators <list>)` | Not one actuator reached the controlled state | Check power and drive faults |
| `Cannot home hand: some actuators could not be prepared for homing (actuators <list>)` | Some actuators did not pass the preparation stage | Check the faults and wiring for those indices |
| `Cannot home hand: some actuators reported a homing error (actuators <list>)` | A drive reported a homing error | Check for mechanical jams and the area around the hard stops |
| `Cannot home hand: some actuators did not finish homing (actuators <list>)` | Some actuators did not finish within the window | Check mechanical resistance and whether the hard stops were reached |
| `Cannot home hand: homing did not complete (actuators <list>)` | An ending that matches none of the above | Keep the log and report it to the SDK |
| `Cannot stop hand: drives did not reach quick stop within 500 ms — retry stop() or power off the hand` | Communication is alive but the drives did not reach the stop | Retry [`stop()`](08_cpp_api_reference/hand.md#handstop) or power the hand off |
| `Cannot disconnect hand: drives did not reach quick stop within 500 ms — power off the hand, or destroy the hand to disconnect anyway` | Same as above. The SDK does not close the connection because it could not confirm the quick stop | Power the robot hand off, or call [`destroy()`](08_cpp_api_reference/hand_manager.md#handmanagerdestroy) if you must disconnect anyway |
| `Cannot run hand: drives did not reach operation enabled within 4000 ms — retry run(), or check actuator faults via diagnostics()` | Communication is alive and no fault is present, but the drives did not reach Operation Enabled | Check the actuator faults and the power, then call [`run()`](08_cpp_api_reference/hand.md#handrun) again |

Every homing failure message ends with ` — retry home(), or check actuator faults via
diagnostics()`. `<list>` names the actuator indices that failed. Indices in `disabled_actuators`
are never reported. The `diagnostics()` in that text is
[`get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics), and what to check for each
value it returns is in [11. Actuator faults](#11-actuator-faults).

## 8. `ControlLoopFault`

| Message | Cause | Remedy |
|---|---|---|
| `Cannot connect hand: failed to start SDK threads (<reason>)` | Thread creation failed | Check resource limits and permissions |
| `hand faulted (control loop terminated at cycle <n>) — call reconnect()` | The control and communication loop ended on an exception | Recover with [`reconnect()`](08_cpp_api_reference/hand.md#handreconnect), and report the reproduction steps and logs to the SDK |

A call rejected in `Faulted` arrives as `hand faulted (<cause>)` —
see [10. Faulted cause phrases](#10-faulted-cause-phrases).

`auto_reconnect` does not apply here. Its conditions and behavior are in the
[C++ guide](07_cpp_usage_guide.md#94-recovering-from-faulted).

## 9. `UnexpectedError`

| Message | Cause | Remedy |
|---|---|---|
| `Unexpected failure in <operation>: <detail>` | An unclassified exception | Report the reproduction steps and full logs to the SDK |
| `Unexpected failure in <operation>: unknown exception` | An unknown exception | Same as above |

## 10. Faulted cause phrases

A call rejected in `Faulted` arrives as `Cannot <action>: hand faulted (<cause>)<hint>`, and the
error code follows the cause.

| `<cause>` | Meaning | [`ErrorCode`](08_cpp_api_reference/types_error.md#enum-errorcode) |
|---|---|---|
| `no RX since cycle <n>` | Nothing received since that cycle | `CommunicationLost` |
| `CAN TX failed at cycle <n> (errno <number>: <text>)` | A send failed with a non-transient error | `CommunicationLost` |
| `control loop terminated at cycle <n>` | The control and communication loop ended on an exception | `ControlLoopFault` |

The `<hint>` depends on the call.

| Call | `<hint>` |
|---|---|
| [`connect()`](08_cpp_api_reference/hand.md#handconnect) | ` — call reconnect() to rebuild` |
| [`disconnect()`](08_cpp_api_reference/hand.md#handdisconnect) | ` — use reconnect() or destroy` |
| [`run()`](08_cpp_api_reference/hand.md#handrun) | ` — call reconnect(), then run()` |
| [`home()`](08_cpp_api_reference/hand.md#handhome) | ` — call reconnect() first` |
| [`stop()`](08_cpp_api_reference/hand.md#handstop) · [`set_command()`](08_cpp_api_reference/hand.md#handset_command) | ` — call reconnect()` |

`no RX since cycle <n>` sometimes carries the last receive error with it. It does not when
reception simply stopped without an error.

| Suffix | Check first |
|---|---|
| `, last RX error: bus off` | Clear the physical bus cause, then confirm the kernel recovery with `restart-ms` and `ip -details link show` ([CAN-FD setup](05_can_fd_setup.md)) |
| `, last RX error: bus error (no ACK)` | Check whether power is off or the robot hand is unplugged |
| `, last RX error: bus error` | Check termination, noise, and bit timing |
| `, last RX error: malformed data` | Check the bit rate and CAN-FD support |
| `, last RX error: conflicting command` | Check whether another master is driving the same hand |
| `, last RX error: unknown data` | Check for other traffic on the bus |

## 11. Actuator faults

The `actuator_health.fault` array from [`get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics) is a separate
axis from [`ErrorCode`](08_cpp_api_reference/types_error.md#enum-errorcode). Values persist even when nothing is thrown, so record them alongside.
Each value is an [`ActuatorFault`](08_cpp_api_reference/types_state.md#enum-actuatorfault), and `None` means
that actuator has no fault. There is an example that walks the whole array in the
[C++ guide](07_cpp_usage_guide.md#81-actuator-faults).

| `to_string()` value | Check first |
|---|---|
| `OverVoltageError` · `UnderVoltageError` | Supply voltage, cable drop, regenerative conditions |
| `OverCurrentError` · `OverLoadError` | Mechanical jam, collision, gains, effort ceiling |
| `OverTemperatureError` | Duty cycle, cooling, ambient temperature |
| `CurrentDetectionError` | The drive's current sensing path |
| `FollowingError` · `SpeedError` | Target step size, load, encoder, controller tuning |
| `CommunicationError` | The bus and cabling on the drive side |
| `HallSensorError` | Hall sensor wiring and connector |
| `PositiveLimitSwitchError` · `NegativeLimitSwitchError` | Limit switches and mechanical position |
| `EmergencySwitchError` · `Sto1Error` · `Sto2Error` (STO = Safe Torque Off) | The emergency stop circuit and safety wiring |
| `SerialEncoderChannelAError` · `SerialEncoderChannelBError` | Encoder signal quality and connector |
| `SerialEncoderChannelADisconnectedError` · `SerialEncoderChannelBDisconnectedError` | A disconnected encoder cable |
| `Unknown` | A code with no name above |

`Unknown` appears in the SDK's RT log as `UnknownError 0x<code>`, so report that code with it.

Do not re-enable repeatedly before the cause is removed.
