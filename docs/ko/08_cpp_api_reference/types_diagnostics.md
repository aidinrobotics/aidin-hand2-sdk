[← C++ API Reference](../08_cpp_api_reference.md)

# `types/diagnostics.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [`Diagnostics`](#diagnostics) | struct | [`get_diagnostics()`](hand.md#handget_diagnostics)가 돌려주는 값 |

---

## `Diagnostics`

| Field | Type | Normal | Description |
|---|---|---|---|
| `lifecycle` | [`HandLifecycle`](types_state.md#enum-handlifecycle) | `Running` | [`Hand`](hand.md#hand)가 도달한 것으로 관측된 상태 |
| `homing_state` | [`HomingState`](types_state.md#enum-homingstate) | `Succeeded` | 원점 상태 |
| `nan_command_count` | `std::uint64_t` | 늘지 않음 | [Validation](types_command.md#validation)에서 거부된 command 수 |
| `control_cycles` | `std::uint64_t` | 계속 증가 | 제어·통신 루프 누적 cycle |
| `deadline_misses` | `std::uint64_t` | 늘지 않음 | 계산 시간이 목표 주기를 넘긴 cycle 누적 |
| `last_period_ms` | `double` | `1000 / control_rate` 근처 | cycle 시작 사이 간격 [ms] |
| `last_compute_ms` | `double` | `last_period_ms` 값보다 작음 | 마지막 cycle의 계산 시간 [ms] |
| `actuator_health` | [`ActuatorHealth`](types_state.md#actuatorhealth) | fault 없음 | actuator별 enable·fault |

`lifecycle` 필드는 요구가 아니라 관측 결과입니다. [`stop()`](hand.md#handstop)이 예외를 던진 뒤에도
`Running`으로 남아 있으면 actuator가 quick stop에 도달했음을 확인하지 못한 것이므로, 마지막
command를 그대로 물고 있을 수 있습니다.

error message, 예외 이력, reconnect 횟수는 들어 있지 않습니다. 원인을 볼 때는 log와 application
쪽 예외 기록을 함께 보십시오.
