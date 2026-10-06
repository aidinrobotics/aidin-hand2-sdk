[← Python API Reference](../11_python_api_reference.md)

# Config

| Symbol | Kind | Description |
|---|---|---|
| [`HandConfig`](#handconfig) | class | [`create()`](hand_manager.md#handmanagercreate)에 넘기는 설정 |
| [`ControllerConfig`](#controllerconfig) | class | [`set_controller_config()`](hand.md#handset_controller_config)로 바꾸는 tuning |

---

## `HandConfig`

### `HandConfig.__init__()`

```python
def __init__(
    self,
    interface_name: str,
    hand_side: HandSide,
    *,
    control_rate: int = 500,
    auto_home: bool = True,
    rt_cpu_affinity: int = -1,
    auto_reconnect: bool = False,
    auto_reconnect_timeout_ms: int = 0,
    auto_reconnect_home: bool = False,
    disabled_actuators: Sequence[int] = (),
) -> None: ...
```

**Parameters**     ｜ `interface_name` — CAN interface 이름 · `hand_side` — `HandSide.LEFT` 또는
`HandSide.RIGHT` · 나머지는 아래 표의 필드와 같음<br>
**Raises**         ｜ `Error`(`INVALID_ARGUMENT`) — `interface_name` 값이 빈 문자열일 때 ·
`TypeError` — `hand_side`가 [`HandSide`](description.md#enum-handside)가 아닐 때

---

### Attributes

생성자 인자와 이름이 같은 속성이며, 만든 뒤에 바꿀 수 있습니다.

| Field | Type | Default | Description |
|---|---|---|---|
| `interface_name` | `str` | 필수 | CAN interface 이름 |
| `hand_side` | [`HandSide`](description.md#enum-handside) | 필수 | 로봇 핸드 좌우 구분 |
| `control_rate` | `int` | `500` | 제어 주기 [Hz]. `0` 이하면 [`create()`](hand_manager.md#handmanagercreate)가 `INVALID_ARGUMENT` |
| `auto_home` | `bool` | `True` | [`run()`](hand.md#handrun)의 자동 homing 여부. `homing_state` 값이 `SUCCEEDED`가 아닐 때만 수행 |
| `rt_cpu_affinity` | `int` | `-1` | 제어·통신 루프를 실행하는 thread를 고정할 CPU 번호. `-1`은 고정 안 함. 고정 실패는 알리지 않음 |
| `auto_reconnect` | `bool` | `False` | 통신 오류 후 자동 재연결 여부 |
| `auto_reconnect_timeout_ms` | `int` | `0` | 재연결 제한 시간 [ms]. `0`은 제한 없음. `auto_reconnect=False`면 무시 |
| `auto_reconnect_home` | `bool` | `False` | 자동 재연결에서 제어 재개 전의 homing 여부. 수동 [`reconnect()`](hand.md#handreconnect)에는 적용되지 않음 |
| `disabled_actuators` | `tuple[int, ...]` | `()` | 사용하지 않을 [actuator index](description.md#array-layout) |

`disabled_actuators` 필드는 읽으면 tuple입니다. 바꿀 때는 `config.disabled_actuators = [3, 7]`처럼
목록 전체를 대입하십시오. 지정한 actuator는 전원이 인가되지 않습니다. 자세한 동작은
[C++ guide](../07_cpp_usage_guide.md#12-handconfig)에 있습니다.

`create()`는 그 시점의 config 값을 복사해 씁니다. `create()` 뒤에 config를 바꿔도 이미 만든 hand에는
적용되지 않습니다.

> [!NOTE]
> ROS 2 bringup은 자체 기본값을 씁니다(`auto_reconnect: true` 등). 위 SDK 기본값과 다르므로
> ROS 2와 동작을 맞추려면 ROS 2 launch 문서를 함께 보십시오.

---

## `ControllerConfig`

[`HandConfig`](#handconfig)와 별개이며 [`set_controller_config()`](hand.md#handset_controller_config)로만
적용합니다. 한 번도 적용하지 않으면 아래 기본값이 쓰입니다.

```python
controller = ControllerConfig()                 # 모든 필드가 기본값
controller.joint_position_controller            # ControllerConfig.JointPositionController
controller.joint_impedance_controller           # ControllerConfig.JointImpedanceController
```

두 필드의 값을 바꾸면 `controller`에 바로 반영됩니다. 배열 필드도 일부만 바꿀 수 있습니다.

---

### `ControllerConfig.JointPositionController`

| Field | Type | Default | Description |
|---|---|---|---|
| `filter_enabled` | `bool` | `True` | deadband·저역통과 사용 여부. `False`면 둘 다 건너뜀 |
| `cutoff_freq` | `float` | `10.0` | 3차 저역통과의 section당 corner [Hz]. `0`이면 끔 |
| `deadband` | `float` | `0.000873` | 무시할 목표 변화의 크기 [rad]. `0`이면 끔 |

두 값을 고르는 기준은 [C++ guide](../07_cpp_usage_guide.md#421-joint-position-controller)에 있습니다.
둘 중 하나라도 NaN·Inf이거나 음수면 [`set_controller_config()`](hand.md#handset_controller_config)가
`INVALID_ARGUMENT` 코드의 `Error`를 던집니다.

---

### `ControllerConfig.JointImpedanceController`

| Field | Type | Default | Description |
|---|---|---|---|
| `stiffness` | `float64` `(16,)` | 아래 표 | 위치 오차에 곱하는 gain |
| `damping` | `float64` `(16,)` | 모든 actuator `1e-5` | 속도에 곱하는 gain |

두 필드는 쓰기 가능한 numpy 배열입니다. `stiffness[0:4] = 0.02`처럼 일부만 바꾸거나, 길이 16인
배열을 통째로 대입합니다. encoder 공간에서 다음과 같이 계산합니다.

```text
effort = stiffness × position_error − damping × velocity
```

| Actuator index | stiffness | damping |
|---|---:|---:|
| 0 – 3 (thumb) | `0.02` | `1e-5` |
| 4, 5 / 7, 8 / 10, 11 / 13, 14 (long finger a1·a2) | `0.01` | `1e-5` |
| 6 / 9 / 12 / 15 (long finger a3) | `0.02` | `1e-5` |

어느 actuator라도 NaN·Inf이거나 음수면 [`set_controller_config()`](hand.md#handset_controller_config)가
`INVALID_ARGUMENT` 코드의 `Error`를 던집니다. 값을 고르는 기준은
[C++ guide](../07_cpp_usage_guide.md#422-joint-impedance-controller)에 있습니다.
