[← C++ API Reference](../08_cpp_api_reference.md)

# `hand/hand.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [`Hand`](#hand) | class | 연결·제어·조회를 하는 handle |

---

## `Hand`

직접 만들거나 없애지 않습니다. [`HandManager::create()`](hand_manager.md#handmanagercreate)로 얻고
[`HandManager::destroy()`](hand_manager.md#handmanagerdestroy)로 파기합니다. 복사해도 같은 hand를 나타냅니다.

모든 method는 실패하면 [`Exception`](types_error.md#exception--stdruntime_error)을 던집니다. 상태 전이 함수가
어느 상태에서 허용되는지는 [C++ guide](../07_cpp_usage_guide.md#22-state-transition-calls)의 표에 정리돼 있습니다.

| Group | Member | Description |
|---|---|---|
| Connection | [`connect()`](#handconnect) | CAN에 연결하고 수신 시작 |
| | [`disconnect()`](#handdisconnect) | actuator quick stop 후 연결 끊음 |
| | [`reconnect()`](#handreconnect) | fault 뒤 통신을 재연결 |
| Operation | [`run()`](#handrun) | actuator enable을 확인하고 제어 시작 |
| | [`stop()`](#handstop) | actuator quick stop, 도달까지 확인 |
| | [`home()`](#handhome) | homing, 완료까지 대기 |
| Command | [`set_command()`](#handset_command) | command를 latch하고 controller 선택 |
| Config | [`set_max_effort()`](#handset_max_effort) | actuator 전류 상한 |
| | [`set_controller_config()`](#handset_controller_config) | filter와 impedance gain |
| Observation | [`get_state()`](#handget_state) | 로봇 핸드에서 읽은 값 |
| | [`get_diagnostics()`](#handget_diagnostics) | SDK·제어·통신 루프 상태 |
| | [`get_command_mode()`](#handget_command_mode) | 지금 활성인 mode |

---

### `Hand::connect()`

```cpp
void connect();
```

CAN에 연결하고 state 수신을 시작합니다. 토크는 걸지 않습니다.

**Throws**         ｜ `InterfaceUnavailable` · `CommunicationLost` ·
`ControlLoopFault` · `WrongCallOrder`<br>
**Preconditions**  ｜ `Disconnected` · `Connected`(no-op)<br>
**Postconditions** ｜ `Connected`<br>
**Notes**          ｜ 첫 state가 올 때까지 최대 300 ms 기다립니다

---

### `Hand::disconnect()`

```cpp
void disconnect();
```

actuator에 quick stop을 요청한 뒤 CAN 연결을 끊습니다. 제어를 요구한 적이 있으면 actuator가
quick stop 상태에 도달한 것을 확인합니다.

**Throws**         ｜ `CommunicationLost` · `HardwareFault` ·
`ControlLoopFault` · `WrongCallOrder`<br>
**Preconditions**  ｜ `Disconnected`(no-op) · `Connected` · `Running` · `Stopped`<br>
**Postconditions** ｜ `Disconnected`<br>
**Notes**          ｜ 확인까지 최대 500 ms 기다립니다. 확인하지 못하면 예외를 던지고 연결을 끊지
않으므로, 그대로 끊어야 하면 [`destroy()`](hand_manager.md#handmanagerdestroy)를 호출하십시오. 도달을 확인하는 동안 fault가
발생하면 제한 시간을 기다리지 않고 그 fault의 원인을 예외로 돌려줍니다

---

### `Hand::reconnect()`

```cpp
void reconnect();
```

fault 뒤 통신을 재연결합니다. 되돌리는 것은 `homing_state` 필드와 직전 command입니다.

- `homing_state` → `NotRun`
- `max_effort`, [`ControllerConfig`](types_config.md#controllerconfig) → 유지
- 직전 command → [`Idle`](types_command.md#idle). 자동 재연결과 같습니다

**Throws**         ｜ `InterfaceUnavailable` · `CommunicationLost` ·
`ControlLoopFault` · `WrongCallOrder`<br>
**Preconditions**  ｜ `Faulted`<br>
**Postconditions** ｜ `Connected`<br>
**Notes**          ｜ 재연결을 한 번만 시도하고, 첫 state가 올 때까지 최대 300 ms 기다립니다. 실패하면
`Faulted`로 남습니다

---

### `Hand::run()`

```cpp
void run();
```

actuator를 enable하고, fault가 없는 actuator가 모두 Operation Enabled에 도달한 것을 확인한 뒤
반환합니다. fault가 발생한 actuator는 확인에서 제외되고 계속 fault reset을 시도합니다.
[`auto_home`](types_config.md#handconfig)`=true`이고 [`homing_state`](types_state.md#enum-homingstate)가
`Succeeded`가 아니면 homing부터 합니다.

**Throws**         ｜ `CommunicationLost` · `HardwareFault` ·
`ControlLoopFault` · `WrongCallOrder`<br>
**Preconditions**  ｜ `Connected` · `Running`(no-op) · `Stopped`<br>
**Postconditions** ｜ `Running`<br>
**Notes**          ｜ actuator enable 확인까지 최대 4000 ms 기다리고, homing을 하는 경로에서는 homing 완료까지
기다립니다. 확인하지 못하면 command를 [`Idle`](types_command.md#idle)로 지우고 actuator에 quick stop을
요구한 뒤 예외를 던지므로, 도달이 확인되면 `Stopped`가 됩니다. `Stopped`에서 재개하면 재개 시점의 자세를 유지하는
command를 한 번 넣습니다

---

### `Hand::stop()`

```cpp
void stop();
```

quick stop을 요청하고 actuator가 quick stop 상태에 도달한 것을 확인합니다. 확인 전에는 `Stopped`가
되지 않습니다. 직전 command는 [`Idle`](types_command.md#idle)로 초기화됩니다.

**Throws**         ｜ `CommunicationLost` · `HardwareFault` ·
`ControlLoopFault` · `WrongCallOrder`<br>
**Preconditions**  ｜ `Running` · `Stopped`(no-op)<br>
**Postconditions** ｜ `Stopped`<br>
**Notes**          ｜ 확인까지 최대 500 ms 기다립니다. 확인하지 못하면 예외를 던지고 `Running`으로 남으므로
actuator가 마지막 command를 유지하고 있을 수 있습니다. 다시 호출하면 재호출 시점의 상태를 기준으로 다시
판정합니다. 도달을 확인하는 동안 fault가 발생하면 제한 시간을 기다리지 않고 그 fault의 원인을
예외로 돌려줍니다

---

### `Hand::home()`

```cpp
void home();
```

homing을 시작하고 끝날 때까지 기다립니다. actuator enable을 안에서 처리하므로 [`run()`](#handrun)을 먼저 부를
필요가 없습니다.

**Throws**         ｜ `HardwareFault`(실패한 actuator index 포함) · `CommunicationLost` ·
`ControlLoopFault` · `WrongCallOrder`<br>
**Preconditions**  ｜ `Connected` · `Running` · `Stopped`<br>
**Postconditions** ｜ `Running`, `homing_state = Succeeded`<br>
**Notes**          ｜ finger가 hard stop까지 움직입니다. 직전 command는 [`Idle`](types_command.md#idle)로 지워집니다

---

### `Hand::set_command()`

```cpp
void set_command(const Idle& command);
void set_command(const JointPositionCommand& command);
void set_command(const JointImpedanceCommand& command);
void set_command(const ActuatorPositionCommand& command);
void set_command(const ActuatorEffortCommand& command);
```

controller는 인자 type에 따라 정해지고, 넣은 command는 다음 호출까지 유지됩니다. joint command는
workspace 안으로 자동 투영됩니다. 범위 모델은 [Workspace limits](../14_workspace_limits.md)에 있습니다.

**Parameters**     ｜ `command` — command 5종 중 하나<br>
**Throws**         ｜ `CommunicationLost` · `ControlLoopFault` ·
`WrongCallOrder`(`Running`이 아니거나 `homing_state != Succeeded`. [`Idle`](types_command.md#idle)은 후자에서 예외)<br>
**Preconditions**  ｜ `Running` — actuator enable이 확인된 상태여야 하므로 [`run()`](#handrun)이 성공한 뒤입니다<br>
**Notes**          ｜ 값이 잘못돼도 예외를 던지지 않습니다. 직전 command가 유지되고
`Diagnostics::nan_command_count`가 늘며 warning log가 남습니다. 조건은 [Validation](types_command.md#validation) 참조

---

### `Hand::set_max_effort()`

```cpp
void set_max_effort(double limit);
void set_max_effort(const std::array<double, kActuatorCount>& limit);
```

actuator 전류 상한을 결정합니다. `double`은 전체 공통, array는 actuator별입니다.

**Parameters**     ｜ `limit` — 정격 전류의 0.1% 단위. `[0, 2000]` 밖은 잘립니다<br>
**Throws**         ｜ `InvalidArgument`(NaN·Inf) ·
`WrongCallOrder`(파기된 `HandCore` 또는 무효가 된 handle)<br>
**Notes**          ｜ [`reconnect()`](#handreconnect)를 지나도 그대로 유지됩니다

---

### `Hand::set_controller_config()`

```cpp
void set_controller_config(const ControllerConfig& config);
```

filter와 impedance gain을 결정합니다. 언제든 호출할 수 있고 다음 cycle부터 적용됩니다.

**Parameters**     ｜ `config` — [`ControllerConfig`](types_config.md#controllerconfig)<br>
**Throws**         ｜ `WrongCallOrder`(파기된 `HandCore` 또는 무효가 된 handle) ·
`InvalidArgument` — `cutoff_freq`·`deadband`·`stiffness`·`damping` 필드 중 non-finite이거나
음수인 값이 있을 때<br>
**Notes**          ｜ 실패하면 기존 설정을 유지합니다. getter가 없으므로 적용값이 필요하면 application에서 직접
보관하십시오

---

### `Hand::get_state()`

```cpp
[[nodiscard]] HandState get_state() const;
```

로봇 핸드에서 읽은 최신 값을 돌려줍니다.

**Returns**        ｜ [`HandState`](types_state.md#handstate)<br>
**Throws**         ｜ `WrongCallOrder`(파기된 `HandCore` 또는 무효가 된 handle)<br>
**Notes**          ｜ 한 번의 호출로 얻은 값들은 같은 cycle에서 나온 것입니다. 값끼리 비교할 때는 한 번 읽은
결과를 재사용하십시오. [`get_diagnostics()`](#handget_diagnostics)와 함께 하나의 transaction을
이루지는 않습니다

---

### `Hand::get_diagnostics()`

```cpp
[[nodiscard]] Diagnostics get_diagnostics() const;
```

SDK와 제어·통신 루프의 최신 상태를 돌려줍니다.

**Returns**        ｜ [`Diagnostics`](types_diagnostics.md#diagnostics)<br>
**Throws**         ｜ `WrongCallOrder`(파기된 `HandCore` 또는 무효가 된 handle)<br>
**Notes**          ｜ 한 번의 호출이 두 시점을 섞습니다 — `lifecycle`·`homing_state`·`nan_command_count` 필드는
호출한 순간의 값이고 나머지는 제어·통신 루프가 마지막으로 발행한 cycle의 값입니다

---

### `Hand::get_command_mode()`

```cpp
[[nodiscard]] CommandMode get_command_mode() const;
```

지금 활성인 controller mode를 돌려줍니다.

**Returns**        ｜ [`CommandMode`](types_command.md#enum-commandmode)<br>
**Throws**         ｜ `WrongCallOrder`(무효가 된 handle)
