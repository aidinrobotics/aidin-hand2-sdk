[← Python API Reference](../11_python_api_reference.md)

# Hand

| Symbol | Kind | Description |
|---|---|---|
| [`Hand`](#hand) | class | 연결·제어·조회를 하는 handle |

---

## `Hand`

직접 만들지 않습니다. [`HandManager.create()`](hand_manager.md#handmanagercreate)로 얻습니다.
`Hand`는 자신을 만든 manager를 참조하므로, `Hand`가 살아 있는 동안 manager도 유지됩니다.

모든 method는 SDK의 실패를 [`Error`](error.md#error)로 던집니다. 정리된 hand를 가리키는 `Hand`의
method는 `WRONG_CALL_ORDER` 코드의 `Error`를 던집니다. 상태 전이 함수가 어느 상태에서 허용되는지는
[Python guide](../10_python_usage_guide.md#22-state-transition-calls)의 표에 정리돼 있습니다.

method가 대기하는 동안 다른 Python thread는 계속 실행됩니다. 대기 중에 누른 `Ctrl-C`는 method가
반환된 뒤 `KeyboardInterrupt`가 됩니다. 같은 `Hand`를 여러 thread에서 호출한다면 동기화는
application의 책임입니다.

| Group | Member | Description |
|---|---|---|
| Connection | [`connect()`](#handconnect) | CAN에 연결하고 수신 시작 |
| | [`disconnect()`](#handdisconnect) | actuator quick stop 후 연결 끊음 |
| | [`reconnect()`](#handreconnect) | fault 뒤 통신을 재연결 |
| Operation | [`run()`](#handrun) | actuator enable을 확인하고 제어 시작 |
| | [`stop()`](#handstop) | actuator quick stop, 도달까지 확인 |
| | [`home()`](#handhome) | homing, 완료까지 대기 |
| Command | [`set_command()`](#handset_command) | command를 적용하고 controller 선택 |
| Config | [`set_max_effort()`](#handset_max_effort) | actuator 전류 상한 |
| | [`set_controller_config()`](#handset_controller_config) | filter와 impedance gain |
| Observation | [`get_state()`](#handget_state) | 로봇 핸드에서 읽은 값 |
| | [`get_diagnostics()`](#handget_diagnostics) | SDK·제어·통신 루프 상태 |
| | [`get_command_mode()`](#handget_command_mode) | 지금 활성인 mode |

---

### `Hand.connect()`

```python
def connect(self) -> None: ...
```

CAN에 연결하고 state 수신을 시작합니다. 토크는 걸지 않습니다.

**Raises**         ｜ `Error` — `INTERFACE_UNAVAILABLE` · `COMMUNICATION_LOST` ·
`CONTROL_LOOP_FAULT` · `WRONG_CALL_ORDER`<br>
**Preconditions**  ｜ `DISCONNECTED` · `CONNECTED`(no-op)<br>
**Postconditions** ｜ `CONNECTED`<br>
**Notes**          ｜ 첫 state가 올 때까지 최대 300 ms 기다립니다

---

### `Hand.disconnect()`

```python
def disconnect(self) -> None: ...
```

actuator에 quick stop을 요청한 뒤 CAN 연결을 끊습니다. 제어를 요구한 적이 있으면 actuator가
quick stop 상태에 도달한 것을 확인합니다.

**Raises**         ｜ `Error` — `COMMUNICATION_LOST` · `HARDWARE_FAULT` ·
`CONTROL_LOOP_FAULT` · `WRONG_CALL_ORDER`<br>
**Preconditions**  ｜ `DISCONNECTED`(no-op) · `CONNECTED` · `RUNNING` · `STOPPED`<br>
**Postconditions** ｜ `DISCONNECTED`<br>
**Notes**          ｜ 확인까지 최대 500 ms 기다립니다. 확인하지 못하면 `Error`를 던지고 연결을 끊지
않으므로, 그대로 끊어야 하면 [`HandManager.destroy()`](hand_manager.md#handmanagerdestroy)를
호출하십시오. 도달을 확인하는 동안 fault가 발생하면 제한 시간을 기다리지 않고 그 fault의 원인을
`Error`로 돌려줍니다

---

### `Hand.reconnect()`

```python
def reconnect(self) -> None: ...
```

fault 뒤 통신을 재연결합니다. 되돌리는 것은 `homing_state` 필드와 직전 command입니다.

- `homing_state` → `NOT_RUN`
- max effort, [`ControllerConfig`](config.md#controllerconfig) → 유지
- 직전 command → [`Idle`](command.md#idle). 자동 재연결과 같습니다

**Raises**         ｜ `Error` — `INTERFACE_UNAVAILABLE` · `COMMUNICATION_LOST` ·
`CONTROL_LOOP_FAULT` · `WRONG_CALL_ORDER`<br>
**Preconditions**  ｜ `FAULTED`<br>
**Postconditions** ｜ `CONNECTED`<br>
**Notes**          ｜ 재연결을 한 번만 시도하고, 첫 state가 올 때까지 최대 300 ms 기다립니다. 실패하면
`FAULTED` 상태로 남습니다

---

### `Hand.run()`

```python
def run(self) -> None: ...
```

actuator를 enable하고, fault가 없는 actuator가 모두 Operation Enabled에 도달한 것을 확인한 뒤
반환합니다. fault가 발생한 actuator는 확인에서 제외되고 계속 fault reset을 시도합니다.
`auto_home=True`이고 `homing_state` 값이 `SUCCEEDED`가 아니면 homing부터 합니다.

**Raises**         ｜ `Error` — `COMMUNICATION_LOST` · `HARDWARE_FAULT` ·
`CONTROL_LOOP_FAULT` · `WRONG_CALL_ORDER`<br>
**Preconditions**  ｜ `CONNECTED` · `RUNNING`(no-op) · `STOPPED`<br>
**Postconditions** ｜ `RUNNING`<br>
**Notes**          ｜ actuator enable 확인까지 최대 4000 ms 기다리고, homing을 하는 경로에서는 homing
완료까지 기다립니다. 확인하지 못하면 command를 [`Idle`](command.md#idle)로 지우고 actuator에 quick
stop을 요구한 뒤 `Error`를 던지므로, 도달이 확인되면 `STOPPED` 상태가 됩니다. `STOPPED` 상태에서
재개하면 재개 시점의 자세를 유지하는 command를 한 번 넣습니다

---

### `Hand.stop()`

```python
def stop(self) -> None: ...
```

quick stop을 요청하고 actuator가 quick stop 상태에 도달한 것을 확인합니다. 확인 전에는 `STOPPED`가
되지 않습니다. 직전 command는 [`Idle`](command.md#idle)로 초기화됩니다.

**Raises**         ｜ `Error` — `COMMUNICATION_LOST` · `HARDWARE_FAULT` ·
`CONTROL_LOOP_FAULT` · `WRONG_CALL_ORDER`<br>
**Preconditions**  ｜ `RUNNING` · `STOPPED`(no-op)<br>
**Postconditions** ｜ `STOPPED`<br>
**Notes**          ｜ 확인까지 최대 500 ms 기다립니다. 확인하지 못하면 `Error`를 던지고 `RUNNING` 상태로
남으므로 actuator가 마지막 command를 유지하고 있을 수 있습니다. 다시 호출하면 재호출 시점의 상태를
기준으로 다시 판정합니다

---

### `Hand.home()`

```python
def home(self) -> None: ...
```

homing을 시작하고 끝날 때까지 기다립니다. actuator enable을 안에서 처리하므로 [`run()`](#handrun)을
먼저 부를 필요가 없습니다.

**Raises**         ｜ `Error` — `HARDWARE_FAULT`(실패한 actuator index 포함) · `COMMUNICATION_LOST` ·
`CONTROL_LOOP_FAULT` · `WRONG_CALL_ORDER`<br>
**Preconditions**  ｜ `CONNECTED` · `RUNNING` · `STOPPED`<br>
**Postconditions** ｜ `RUNNING`, `homing_state` 값이 `SUCCEEDED`<br>
**Notes**          ｜ finger가 hard stop까지 움직입니다. 500 Hz에서 최대 22 s 기다립니다. 직전 command는
[`Idle`](command.md#idle)로 지워집니다

---

### `Hand.set_command()`

```python
def set_command(
    self,
    command: Idle | JointPositionCommand | JointImpedanceCommand
    | ActuatorPositionCommand | ActuatorEffortCommand,
) -> None: ...
```

controller는 `command`의 class에 따라 정해지고, 넣은 command는 다음 호출까지 유지됩니다. joint
command는 도달 범위 안으로 자동 투영됩니다. 범위 모델은 [Workspace limits](../14_workspace_limits.md)에
있습니다. 호출한 시점의 `target` 값이 적용되므로, `target` 필드를 바꾼 뒤에는 다시 호출해야 합니다.

**Parameters**     ｜ `command` — command 5종 중 하나<br>
**Raises**         ｜ `Error` — `COMMUNICATION_LOST` · `CONTROL_LOOP_FAULT` ·
`WRONG_CALL_ORDER`(`RUNNING` 상태가 아니거나 `homing_state` 값이 `SUCCEEDED`가 아닐 때.
[`Idle`](command.md#idle)은 후자에서 예외) · `TypeError` — command가 아닌 값<br>
**Preconditions**  ｜ `RUNNING` — actuator enable이 확인된 상태여야 하므로 [`run()`](#handrun)이 성공한 뒤입니다<br>
**Notes**          ｜ 값이 잘못돼도 예외를 던지지 않습니다. 직전 command가 유지되고
`Diagnostics.nan_command_count`가 늘며 warning log가 남습니다. 조건은
[Validation](command.md#validation) 참조

---

### `Hand.set_max_effort()`

```python
def set_max_effort(self, limit: float | ArrayLike) -> None: ...
```

actuator 전류 상한을 결정합니다. 숫자 하나는 전체 공통, 길이 16인 배열은 actuator별입니다.

**Parameters**     ｜ `limit` — 정격 전류의 0.1% 단위. `[0, 2000]` 밖은 잘립니다<br>
**Raises**         ｜ `Error` — `INVALID_ARGUMENT`(NaN·Inf) · `WRONG_CALL_ORDER`(정리된 hand) ·
`ValueError` — 배열 길이가 16이 아닐 때<br>
**Notes**          ｜ [`reconnect()`](#handreconnect)를 지나도 그대로 유지됩니다

---

### `Hand.set_controller_config()`

```python
def set_controller_config(self, config: ControllerConfig) -> None: ...
```

filter와 impedance gain을 결정합니다. 언제든 호출할 수 있고 다음 cycle부터 적용됩니다.

**Parameters**     ｜ `config` — [`ControllerConfig`](config.md#controllerconfig)<br>
**Raises**         ｜ `Error` — `WRONG_CALL_ORDER`(정리된 hand) · `INVALID_ARGUMENT` —
`cutoff_freq`·`deadband`·`stiffness`·`damping` 필드 중 NaN·Inf이거나 음수인 값이 있을 때<br>
**Notes**          ｜ 실패하면 기존 설정을 유지합니다. getter가 없으므로 적용값이 필요하면 application에서
직접 보관하십시오. 호출한 뒤 `config`를 바꿔도 다시 호출하기 전에는 적용되지 않습니다

---

### `Hand.get_state()`

```python
def get_state(self) -> HandState: ...
```

로봇 핸드에서 읽은 최신 값을 돌려줍니다.

**Returns**        ｜ [`HandState`](state.md#handstate). 호출한 시점의 사본이며 안의 배열은 읽기 전용<br>
**Raises**         ｜ `Error` — `WRONG_CALL_ORDER`(정리된 hand)<br>
**Notes**          ｜ 한 번의 호출로 얻은 값들은 같은 cycle에서 나온 것입니다. 값끼리 비교할 때는 한 번
읽은 결과를 재사용하십시오. [`get_diagnostics()`](#handget_diagnostics)와 같은 cycle을 보장하지
않습니다. `FAULTED` 상태에서도 예외를 던지지 않습니다

---

### `Hand.get_diagnostics()`

```python
def get_diagnostics(self) -> Diagnostics: ...
```

SDK와 제어·통신 루프의 최신 상태를 돌려줍니다.

**Returns**        ｜ [`Diagnostics`](diagnostics.md#diagnostics). 호출한 시점의 사본<br>
**Raises**         ｜ `Error` — `WRONG_CALL_ORDER`(정리된 hand)<br>
**Notes**          ｜ 한 번의 호출이 두 시점을 섞습니다. `lifecycle`·`homing_state`·`nan_command_count`
필드는 호출한 순간의 값이고 나머지는 제어·통신 루프가 마지막으로 발행한 cycle의 값입니다.
`FAULTED` 상태에서도 예외를 던지지 않습니다

---

### `Hand.get_command_mode()`

```python
def get_command_mode(self) -> CommandMode: ...
```

지금 활성인 controller mode를 돌려줍니다.

**Returns**        ｜ [`CommandMode`](command.md#enum-commandmode)<br>
**Raises**         ｜ `Error` — `WRONG_CALL_ORDER`(정리된 hand)
