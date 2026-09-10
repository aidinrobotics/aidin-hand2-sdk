[← C++ API Reference](../08_cpp_api_reference.md)

# `types/config.hpp`

| Symbol | Kind | Description |
|---|---|---|
| [`HandConfig`](#handconfig) | struct | The settings passed to [`create()`](hand_manager.md#handmanagercreate) |
| [`ControllerConfig`](#controllerconfig) | struct | Tuning changed through [`set_controller_config()`](hand.md#handset_controller_config) |

## `HandConfig`

### `HandConfig::HandConfig()`

```cpp
HandConfig(std::string interface_name, HandSide hand_side);
```

**Parameters**     ｜ `interface_name` — CAN interface name · `hand_side` — `Left` or `Right`<br>
**Throws**         ｜ `InvalidArgument` — `interface_name` is empty or `hand_side` is out of range

### Member variables

| Field | Type | Default | Description |
|---|---|---|---|
| `interface_name` | `std::string` | required | CAN interface name |
| `hand_side` | [`HandSide`](types_description.md#enum-handside) | required | Which side the robot hand is |
| `control_rate` | `int` | `500` | Control period [Hz]. `0` or less makes [`create()`](hand_manager.md#handmanagercreate) raise `InvalidArgument` |
| `auto_home` | `bool` | `true` | Whether [`run()`](hand.md#handrun) homes first. Only when `homing_state` is not `Succeeded` |
| `rt_cpu_affinity` | `int` | `-1` | The CPU that the SDK pins the thread of the control and communication loop to. `-1` for no pinning. A failed pinning is not reported |
| `auto_reconnect` | `bool` | `false` | Whether to reconnect automatically after a communication error |
| `auto_reconnect_timeout_ms` | `int` | `0` | Reconnect time limit [ms]. `0` for no limit, ignored when `auto_reconnect=false` |
| `auto_reconnect_home` | `bool` | `false` | Whether automatic reconnection homes before resuming control. Not applied to a manual [`reconnect()`](hand.md#handreconnect) |
| `disabled_actuators` | `std::vector<int>` | `{}` | Actuator indices not in use |

An actuator listed in `disabled_actuators` is held at drive shutdown and never powers up. What it
does in detail is in the [C++ guide](../07_cpp_usage_guide.md#12-handconfig).

> [!NOTE]
> The ROS 2 bringup uses its own defaults (`auto_reconnect: true` and others). They differ from the
> SDK defaults above, so read the ROS 2 launch documentation alongside this.

## `ControllerConfig`

Separate from [`HandConfig`](#handconfig) and only set through
[`set_controller_config()`](hand.md#handset_controller_config). Without a single call the SDK uses the defaults below.

```cpp
struct ControllerConfig {
  JointPositionController  joint_position_controller{};
  JointImpedanceController joint_impedance_controller{};
};
```

### `ControllerConfig::JointPositionController`

| Field | Type | Default | Description |
|---|---|---|---|
| `filter_enabled` | `bool` | `true` | Whether the deadband and the low-pass are applied. `false` to bypass both |
| `cutoff_freq` | `double` | `10.0` | Per-section corner of a 3rd-order low-pass [Hz]. `0` to turn it off |
| `deadband` | `double` | `0.000873` | Target change small enough to ignore [rad]. `0` to turn it off |

How to choose both values is in the [C++ guide](../07_cpp_usage_guide.md#421-joint-position-controller). If either is
non-finite or negative, [`set_controller_config()`](hand.md#handset_controller_config) throws `InvalidArgument`.

### `ControllerConfig::JointImpedanceController`

| Field | Type | Default | Description |
|---|---|---|---|
| `stiffness` | `std::array<double, kActuatorCount>` | see table | Gain on position error |
| `damping` | `std::array<double, kActuatorCount>` | `1e-5` on every axis | Gain on velocity |

Computed in encoder space:

```text
effort = stiffness × position_error − damping × velocity
```

| Actuator index | stiffness | damping |
|---|---:|---:|
| 0 – 3 (thumb) | `0.02` | `1e-5` |
| 4, 5 / 7, 8 / 10, 11 / 13, 14 (long finger a1, a2) | `0.01` | `1e-5` |
| 6 / 9 / 12 / 15 (long finger a3) | `0.02` | `1e-5` |

If any axis is non-finite or negative, [`set_controller_config()`](hand.md#handset_controller_config) throws `InvalidArgument`. How to
choose the values is in the [C++ guide](../07_cpp_usage_guide.md#422-joint-impedance-controller).

---
