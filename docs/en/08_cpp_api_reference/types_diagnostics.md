[← C++ API Reference](../08_cpp_api_reference.md)

# `types/diagnostics.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [`Diagnostics`](#diagnostics) | struct | What [`get_diagnostics()`](hand.md#handget_diagnostics) returns |

## `Diagnostics`

| Field | Type | Expected | Description |
|---|---|---|---|
| `lifecycle` | [`HandLifecycle`](types_state.md#enum-handlifecycle) | `Running` | The state that the SDK observed the [`Hand`](hand.md#hand) reach |
| `fault_code` | `std::uint16_t` | `0` | The [error code](../15_error_messages.md#12-code-ranges) of the event that made the `lifecycle` value `Faulted` (`404`, `405`, `702`). `0` when not `Faulted` |
| `homing_state` | [`HomingState`](types_state.md#enum-homingstate) | `Succeeded` | Origin state |
| `nan_command_count` | `std::uint64_t` | Does not grow | Commands rejected by [Validation](types_command.md#validation) |
| `control_cycles` | `std::uint64_t` | Keeps increasing | Cumulative cycles of the control and communication loop |
| `deadline_misses` | `std::uint64_t` | Does not grow | Cumulative cycles whose compute time overran the target period |
| `last_period_ms` | `double` | Near `2.0` | Gap between cycle starts [ms] |
| `last_compute_ms` | `double` | Below `last_period_ms` | Compute time of that cycle [ms] |
| `actuator_health` | [`ActuatorHealth`](types_state.md#actuatorhealth) | No faults | Per-actuator enable and fault |

The `lifecycle` field is an observation, not a request. If it stays `Running` after
[`stop()`](hand.md#handstop) throws, the SDK could not confirm that the actuators reached the quick stop,
so they may still hold the last command.

The `fault_code` field goes back to `0` when [`reconnect()`](hand.md#handreconnect) or the automatic
reconnect succeeds and the hand leaves `Faulted`, and keeps its value when it fails and the hand stays
`Faulted`. So a fault that the SDK already recovered with `auto_reconnect` set to `true` does not show
in this field, only in the log.

There is no error message, exception history, or reconnect count in here. To investigate a cause,
read the log and the application's own exception records alongside it.

---
