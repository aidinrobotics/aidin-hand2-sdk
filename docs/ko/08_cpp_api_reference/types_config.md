[← C++ API Reference](../08_cpp_api_reference.md)

# `types/config.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [`HandConfig`](#handconfig) | struct | [`create()`](hand_manager.md#handmanagercreate)에 넘기는 설정 |
| [`ControllerConfig`](#controllerconfig) | struct | [`set_controller_config()`](hand.md#handset_controller_config)로 바꾸는 tuning |

---

## `HandConfig`

### `HandConfig::HandConfig()`

```cpp
HandConfig(std::string interface_name, HandSide hand_side);
```

**Parameters**     ｜ `interface_name` — CAN interface 이름 · `hand_side` — `Left` 또는 `Right`<br>
**Throws**         ｜ `InvalidArgument` — `interface_name` 필드가 비었거나 `hand_side` 필드가 범위 밖일 때

---

### Member variables

| Field | Type | Default | Description |
|---|---|---|---|
| `interface_name` | `std::string` | 필수 | CAN interface 이름 |
| `hand_side` | [`HandSide`](types_description.md#enum-handside) | 필수 | 로봇 핸드 좌우 구분 |
| `control_rate` | `int` | `500` | 제어 주기 [Hz]. `0` 이하면 [`create()`](hand_manager.md#handmanagercreate)가 `InvalidArgument` |
| `auto_home` | `bool` | `true` | [`run()`](hand.md#handrun)의 자동 homing 여부. `homing_state` 값이 `Succeeded`가 아닐 때만 수행 |
| `rt_cpu_affinity` | `int` | `-1` | 제어·통신 루프를 실행하는 thread를 고정할 CPU 번호. `-1`은 고정 안 함. 고정 실패는 알리지 않음 |
| `auto_reconnect` | `bool` | `false` | 통신 오류 후 자동 재연결 여부 |
| `auto_reconnect_timeout_ms` | `int` | `0` | 재연결 제한 시간 [ms]. `0`은 제한 없음. `auto_reconnect=false`면 무시 |
| `auto_reconnect_home` | `bool` | `false` | 자동 재연결에서 제어 재개 전의 homing 여부. 수동 [`reconnect()`](hand.md#handreconnect)에는 적용되지 않음 |
| `disabled_actuators` | `std::vector<int>` | `{}` | 사용하지 않을 actuator index |

`disabled_actuators` 필드에 지정한 actuator는 shutdown 상태로 유지되어 전원이 인가되지 않습니다.
자세한 동작은 [C++ guide](../07_cpp_usage_guide.md#12-handconfig)에 있습니다.

> [!NOTE]
> ROS 2 bringup은 자체 기본값을 씁니다(`auto_reconnect: true` 등). 위 SDK 기본값과 다르므로
> ROS 2 launch 문서를 함께 보십시오.

---

## `ControllerConfig`

[`HandConfig`](#handconfig)와 별개이며 [`set_controller_config()`](hand.md#handset_controller_config)로만 넣습니다.
한 번도 넣지 않으면 아래 기본값이 쓰입니다.

```cpp
struct ControllerConfig {
  JointPositionController  joint_position_controller{};
  JointImpedanceController joint_impedance_controller{};
};
```

---

### `ControllerConfig::JointPositionController`

| Field | Type | Default | Description |
|---|---|---|---|
| `filter_enabled` | `bool` | `true` | deadband·저역통과 사용 여부. `false`면 둘 다 건너뜀 |
| `cutoff_freq` | `double` | `10.0` | 3차 저역통과의 section당 corner [Hz]. `0`이면 끔 |
| `deadband` | `double` | `0.000873` | 무시할 목표 변화의 크기 [rad]. `0`이면 끔 |

두 값을 고르는 기준은 [C++ guide](../07_cpp_usage_guide.md#421-joint-position-controller)에 있습니다. 둘 중 하나라도
non-finite이거나 음수면 [`set_controller_config()`](hand.md#handset_controller_config)가 `InvalidArgument` 예외를 던집니다.

---

### `ControllerConfig::JointImpedanceController`

| Field | Type | Default | Description |
|---|---|---|---|
| `stiffness` | `std::array<double, kActuatorCount>` | 아래 표 | 위치 오차에 곱하는 gain |
| `damping` | `std::array<double, kActuatorCount>` | 모든 actuator `1e-5` | 속도에 곱하는 gain |

encoder 공간에서 다음과 같이 계산합니다.

```text
effort = stiffness × position_error − damping × velocity
```

| Actuator index | stiffness | damping |
|---|---:|---:|
| 0 – 3 (thumb) | `0.02` | `1e-5` |
| 4, 5 / 7, 8 / 10, 11 / 13, 14 (long finger a1·a2) | `0.01` | `1e-5` |
| 6 / 9 / 12 / 15 (long finger a3) | `0.02` | `1e-5` |

어느 actuator라도 non-finite이거나 음수면 [`set_controller_config()`](hand.md#handset_controller_config)가 `InvalidArgument` 예외를
던집니다. 값을 고르는 기준은 [C++ guide](../07_cpp_usage_guide.md#422-joint-impedance-controller)에 있습니다.
