# Python guide

Python 패키지 `aidin-hand2`로 AIDIN Hand Gen2를 제어하는 방법을 설명합니다. 장의 순서는
[C++ guide](07_cpp_usage_guide.md)와 같고, 로봇 핸드의 동작도 C++ SDK와 같습니다. 이 문서는
Python 코드와 Python에서 달라지는 점을 다루고, 두 언어에 공통인 동작의 상세는 C++ guide의 해당 절로
연결합니다. 먼저 [SDK build & install](06_sdk_build_and_install.md#2-python)으로 패키지를
설치하십시오.

모든 심볼은 `aidin_hand2` 모듈에 있고, 이 문서의 코드는 `ah2`로 import합니다. 배열은 numpy
배열입니다. 심볼과 필드 목록은 [API reference](11_python_api_reference.md)에 있습니다.

```python
import numpy as np
import aidin_hand2 as ah2
```

C++ SDK를 알고 있다면 다음 다섯 가지만 다릅니다.

| Item | C++ | Python |
|---|---|---|
| 정리 시점 | `HandManager` 소멸자 | `with` 블록의 끝, 프로그램 종료 |
| 배열 | `std::array` | numpy 배열 |
| 예외 | `aidin_hand2::Exception` | `ah2.Error` |
| enum 멤버와 상수 | `HandSide::Right`, `kActuatorCount` | `HandSide.RIGHT`, `ACTUATOR_COUNT` |
| variant | `std::variant` | 담긴 객체. 값이 없으면 `None` |

## Contents

&nbsp;&nbsp;[**1. HandManager and Hand**](#1-handmanager-and-hand)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.1 Types](#11-types)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.2 HandConfig](#12-handconfig)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.3 Create and close](#13-create-and-close)<br>
&nbsp;&nbsp;[**2. Lifecycle**](#2-lifecycle)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.1 Lifecycle states](#21-lifecycle-states)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.2 State transition calls](#22-state-transition-calls)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.3 Transition confirmation](#23-transition-confirmation)<br>
&nbsp;&nbsp;[**3. Homing**](#3-homing)<br>
&nbsp;&nbsp;[**4. Control settings**](#4-control-settings)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.1 Max effort](#41-max-effort)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.2 Controllers](#42-controllers)<br>
&nbsp;&nbsp;[**5. Command**](#5-command)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[5.1 Command types](#51-command-types)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[5.2 Validation and clamp](#52-validation-and-clamp)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[5.3 Command lifetime](#53-command-lifetime)<br>
&nbsp;&nbsp;[**6. Kinematics**](#6-kinematics)<br>
&nbsp;&nbsp;[**7. HandState**](#7-handstate)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[7.1 Actuators and joints](#71-actuators-and-joints)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[7.2 Applied command](#72-applied-command)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[7.3 Tactile](#73-tactile)<br>
&nbsp;&nbsp;[**8. Diagnostics**](#8-diagnostics)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[8.1 Actuator faults](#81-actuator-faults)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[8.2 Diagnostic record](#82-diagnostic-record)<br>
&nbsp;&nbsp;[**9. Error handling**](#9-error-handling)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[9.1 Exceptions](#91-exceptions)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[9.2 Detecting a failure](#92-detecting-a-failure)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[9.3 Recovery patterns](#93-recovery-patterns)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[9.4 Recovering from Faulted](#94-recovering-from-faulted)

## 1. HandManager and Hand

### 1.1 Types

SDK는 [`HandManager`](11_python_api_reference/hand_manager.md#handmanager)와
[`Hand`](11_python_api_reference/hand.md#hand)를 통해 AIDIN Hand Gen2에 접근합니다.

| Type | Description |
|---|---|
| `HandManager` | `HandConfig`를 검증해 로봇 핸드 하나를 다루는 자원을 만들고 정리합니다. `Hand`를 반환합니다 |
| `Hand` | 그 자원 하나를 가리키며 연결·제어·설정·조회를 수행합니다 |

자원은 CAN-FD 통신과 제어·통신 루프를 담당하며, `HandManager`가 소유합니다.

### 1.2 HandConfig

[`HandConfig`](11_python_api_reference/config.md#handconfig)는 `Hand`의 동작을 설정합니다.
`interface_name`과 `hand_side`는 필수 위치 인자이고, 나머지는 keyword 인자이며 기본값이
있습니다.

다음은 모든 항목을 지정해 config를 만드는 예입니다.

```python
# 기본값이 아니라 설정 예시입니다. 기본값은 아래 표에 있습니다.
config = ah2.HandConfig(
    "can0", ah2.HandSide.LEFT,         # 위치 인자 2개는 필수
    control_rate=500,                  # Hz
    auto_home=False,
    rt_cpu_affinity=-1,
    auto_reconnect=True,
    auto_reconnect_timeout_ms=5000,    # ms
    auto_reconnect_home=True,
    disabled_actuators=[],             # 예: [3, 7]
)
```

| Field | Type | Default | Description |
|---|---|---|---|
| `interface_name` | `str` | 필수 | CAN interface 이름 |
| `hand_side` | [`HandSide`](11_python_api_reference/description.md#enum-handside) | 필수 | 로봇 핸드 좌우 구분 (왼손/오른손) |
| `control_rate` | `int` | `500` | 제어 주기 [Hz] |
| `auto_home` | `bool` | `True` | `run()`의 자동 homing 여부.</br> `homing_state` 값이 `SUCCEEDED`가 아닐 때만 수행 |
| `rt_cpu_affinity` | `int` | `-1` | 제어·통신 루프를 실행하는 thread를 고정할 CPU 번호.</br> `-1`은 고정 안 함 |
| `auto_reconnect` | `bool` | `False` | 통신 오류 발생 시, 자동 재연결 시도 여부 |
| `auto_reconnect_timeout_ms` | `int` | `0` | 자동 재연결 제한 시간 [ms].</br> `0`은 제한 없음. `auto_reconnect=False`면 무시 |
| `auto_reconnect_home` | `bool` | `False` | 자동 재연결에서 자동 homing 여부.</br> 수동 `reconnect()`에는 적용되지 않음 |
| `disabled_actuators` | `tuple[int, ...]` | `()` | 사용하지 않을 [actuator index](11_python_api_reference/description.md#array-layout) |

만든 뒤에 속성을 바꿀 수도 있습니다. `disabled_actuators` 필드는 읽으면 tuple이므로, 바꿀 때는
목록 전체를 대입하십시오.

```python
config.auto_home = True
config.disabled_actuators = [3, 7]     # append()로는 바꿀 수 없습니다
```

config는 다음 경우에 거부됩니다.

- `interface_name` 값이 빈 문자열이면 생성자가 `Error`를 던집니다. `code`는 `INVALID_ARGUMENT`입니다.
- `hand_side`에 `HandSide`가 아닌 값을 넘기면 생성자가 `TypeError`를 던집니다.
- `control_rate` 값이 `0` 이하이면 `create()`가 `INVALID_ARGUMENT` 코드의 `Error`를 던집니다.

`disabled_actuators` 필드를 쓰는 경우와 지정 단위는
[C++ guide](07_cpp_usage_guide.md#12-handconfig)에 있습니다.

### 1.3 Create and close

[`create()`](11_python_api_reference/hand_manager.md#handmanagercreate)는 config를 검증하고 자원을
만든 뒤 그 자원을 가리키는 `Hand`를 반환합니다. CAN 연결은
[`connect()`](11_python_api_reference/hand.md#handconnect)가 수행합니다.

`HandManager`는 `with` 블록으로 쓰십시오. 블록을 벗어나면 manager가 만든 모든 hand에 actuator
quick stop을 확인하고 연결을 끊습니다. 예외나 `Ctrl-C`로 블록을 벗어나도 같습니다.

```python
with ah2.HandManager() as manager:
    hand = manager.create(config)
    hand.connect()
    # 제어 코드
# 여기서 모든 hand가 정지하고 연결이 끊깁니다
```

- 블록을 벗어난 뒤나 [`destroy()`](11_python_api_reference/hand_manager.md#handmanagerdestroy)로
  정리한 뒤에는 그 `Hand`가 무효가 되고, method를 호출하면 `WRONG_CALL_ORDER` 코드의 `Error`를
  던집니다.
- `Hand`는 자신을 만든 manager를 붙잡고 있습니다. 그래서 `hand = ah2.HandManager().create(config)`처럼
  manager를 변수에 담지 않아도 `hand`가 살아 있는 동안 manager도 유지됩니다. 이 경우 정리 시점은
  `hand`와 manager가 모두 해제될 때이므로, 정리 시점을 정하려면 `with`를 쓰십시오.
- 프로그램이 끝날 때 남아 있는 manager는 자동으로 정리됩니다. 처리하지 않은 예외로 끝나도
  정리되지만, process가 강제로 종료되면 정리되지 않습니다.
- `Hand`는 thread-safe하지 않습니다. 여러 thread에서 같은 `Hand`를 호출한다면 동기화는
  application의 책임입니다. SDK 호출이 대기하는 동안에도 다른 Python thread는 계속 실행됩니다.

`HandManager` 하나로 로봇 핸드 여러 대를 다룰 수 있습니다. 로봇 핸드 두 대를 함께 사용하려면
`interface_name`과 `hand_side` 값이 다른 config로 `create()`를 두 번 호출하십시오. hand마다
제어·통신 루프가 따로 있어 서로 독립적으로 동작합니다.

```python
with ah2.HandManager() as manager:
    left = manager.create(ah2.HandConfig("can0", ah2.HandSide.LEFT))
    right = manager.create(ah2.HandConfig("can1", ah2.HandSide.RIGHT))
```

## 2. Lifecycle

### 2.1 Lifecycle states

`Hand`의 lifecycle은 [`HandLifecycle`](11_python_api_reference/state.md#enum-handlifecycle)
enum의 5개 state 중 하나이며, `get_diagnostics().lifecycle`로 확인합니다. 상태 전이 그림은
[C++ guide](07_cpp_usage_guide.md#21-lifecycle-states)에 있습니다.

| State | Description |
|---|---|
| `DISCONNECTED` | CAN socket이 열리지 않은 상태입니다. `create()` 직후와 `disconnect()` 완료 후의 상태입니다 |
| `CONNECTED` | CAN socket이 열려 있고 state를 수신하지만 제어 명령은 송신하지 않는 상태입니다 |
| `RUNNING` | actuator enable이 확인되어 제어 명령을 송신하는 상태입니다. `set_command()`는 `RUNNING`에서만 동작합니다 |
| `STOPPED` | actuator의 quick stop이 확인된 무토크 상태입니다 |
| `FAULTED` | 통신 오류나 제어·통신 루프 예외로 통신과 제어가 종료된 상태입니다. `reconnect()`로만 벗어납니다 |

### 2.2 State transition calls

상태 전이 함수는 아래 표의 precondition에 해당하는 lifecycle에서 호출할 수 있습니다. 허용되지 않은
state에서 호출하면 `WRONG_CALL_ORDER` 코드의 `Error`가 발생합니다.

각 함수는 confirmation 조건이 확인될 때까지 대기하며, 제한 시간 안에 확인되지 않으면 `Error`를
던집니다.

| Method | Precondition | Postcondition | Confirmation | Timeout |
|---|---|---|---|---|
| [`connect()`](11_python_api_reference/hand.md#handconnect) | `DISCONNECTED` | `CONNECTED` | 첫 state 수신 | 300 ms |
| [`disconnect()`](11_python_api_reference/hand.md#handdisconnect) | `CONNECTED` · `RUNNING` · `STOPPED` | `DISCONNECTED` | actuator quick stop | 500 ms |
| [`run()`](11_python_api_reference/hand.md#handrun) | `CONNECTED` · `STOPPED` | `RUNNING` | actuator enable | 4000 ms |
| [`home()`](11_python_api_reference/hand.md#handhome) | `CONNECTED` · `RUNNING` · `STOPPED` | `RUNNING` | actuator enable과 homing 완료 | 500 Hz에서 최대 22 s |
| [`stop()`](11_python_api_reference/hand.md#handstop) | `RUNNING` | `STOPPED` | actuator quick stop | 500 ms |
| [`reconnect()`](11_python_api_reference/hand.md#handreconnect) | `FAULTED` | `CONNECTED` | 첫 state 수신 | 300 ms |

`auto_home=True`이고 `homing_state` 값이 `SUCCEEDED`가 아니면 `run()`이 homing까지 수행하므로,
`run()`에도 `home()`과 같은 제한 시간이 적용됩니다. `home()`의 제한 시간은 `control_rate` 값에 따라
달라집니다.

대기하는 동안 누른 `Ctrl-C`는 호출이 반환된 뒤에 `KeyboardInterrupt`가 됩니다. 그래서 homing
중에는 최대 위 표의 시간만큼 늦게 반영됩니다. 대기 중에도 제어·통신 루프는 계속 동작합니다.

이미 postcondition이 확인된 state에서 같은 함수를 호출하면 별도의 동작 없이 성공합니다.
`CONNECTED`에서 `connect()`, `DISCONNECTED`에서 `disconnect()`, `RUNNING`에서 `run()`,
`STOPPED`에서 `stop()`이 해당합니다.

> [!NOTE]
> 다음 함수는 `FAULTED`를 포함한 모든 state에서 호출할 수 있습니다.
>
> - [`set_max_effort()`](11_python_api_reference/hand.md#handset_max_effort)
> - [`set_controller_config()`](11_python_api_reference/hand.md#handset_controller_config)
> - [`get_state()`](11_python_api_reference/hand.md#handget_state)
> - [`get_diagnostics()`](11_python_api_reference/hand.md#handget_diagnostics)
> - [`get_command_mode()`](11_python_api_reference/hand.md#handget_command_mode)
>
> [`set_command()`](11_python_api_reference/hand.md#handset_command)는 `RUNNING`에서만 호출할 수
> 있으며, `Idle`을 제외한 command는 `homing_state` 값이 `SUCCEEDED`여야 합니다.

### 2.3 Transition confirmation

상태 전이 함수는 실제 동작이 확인되어야 반환합니다. `connect()`와 `reconnect()`는 첫 state 수신을,
`run()`은 actuator enable을, `stop()`과 `disconnect()`는 actuator quick stop을, `home()`은 enable과
homing 완료를 확인합니다.

확인에 포함되는 actuator와 제한 시간을 넘긴 뒤의 lifecycle은 C++ SDK와 같습니다.
[2.3.1 Confirmation criteria](07_cpp_usage_guide.md#231-confirmation-criteria)와
[2.3.2 Confirmation failure](07_cpp_usage_guide.md#232-confirmation-failure)를 보십시오. 같은 함수를
다시 호출하면 직전 호출을 이어서 처리하지 않고, 호출 시점의 lifecycle과 actuator state를 기준으로
조건을 다시 확인합니다.

## 3. Homing

homing은 각 finger를 hard stop까지 밀어 hard stop 지점을 원점으로 삼는 절차입니다.
[`homing_state`](11_python_api_reference/state.md#enum-homingstate) 값이 `SUCCEEDED`가 되기
전에는 `Idle`을 뺀 모든 `set_command()`가 `WRONG_CALL_ORDER` 코드의 `Error`로 거부됩니다.

[`home()`](11_python_api_reference/hand.md#handhome)은 완료까지 대기하고, 실패하면 `Error`를
던집니다.

```python
try:
    hand.home()
except ah2.Error as error:
    # 다시 시도하거나 actuator fault를 확인합니다
    print(error.code, error)
```

- `RUNNING`이 아닌 상태에서 호출하면 homing이 actuator를 enable하므로 `run()`을 먼저 호출할 필요가
  없습니다.
- `auto_home=True`이면 `run()`이 homing을 자동으로 진행합니다. `homing_state` 값이 `SUCCEEDED`가
  아니면 `RUNNING` 상태에서 호출한 `run()`도 homing을 시작하므로, finger가 hard stop까지 이동합니다.
- 성공하면 원점 자세를 유지하는 command가 적용됩니다. 이어서 필요한 command를 전송하십시오.
- 실패하면 `HARDWARE_FAULT` 코드의 `Error`를 던지고 메시지에 실패한 actuator index가 포함됩니다.
- `reconnect()`는 `homing_state` 값을 `NOT_RUN`으로 되돌리므로 homing을 다시 진행해야 합니다.

> [!WARNING]
> finger가 완전히 펴지지 못하게 막혀 있으면 막힌 지점이 hard stop으로 인식되어 원점이
> 어긋난 채 성공으로 보고됩니다. homing 전에 주변을 비우고 완료까지 접촉하지 마십시오.

## 4. Control settings

control setting은 max effort와 controller 설정 두 가지이며, command가 drive 전송값으로 변환되는
방식을 결정합니다. 두 설정은 lifecycle과 무관하게 언제든 호출할 수 있고 `reconnect()` 이후에도
유지됩니다. effort의 단위는 정격 전류의 0.1%이고, 정격 전류는 모든 모터가 400 mA이므로
`1000`(100%)이 400 mA입니다.

### 4.1 Max effort

max effort는 actuator로 전송되는 effort의 상한입니다. 기본값은 `1000`이고, `[0, 2000]` 밖의 값은
그 범위로 제한해 적용합니다.

actuator 전체에 같은 값을 적용하려면 숫자 하나를
[`set_max_effort()`](11_python_api_reference/hand.md#handset_max_effort)에 전달합니다.

```python
hand.set_max_effort(1000.0)
```

actuator마다 다르게 적용하려면 길이 16인 list나 numpy 배열을 전달합니다. 다음 값은 예시입니다.

```python
hand.set_max_effort([
    1000.0, 1000.0, 1000.0, 1000.0,   # thumb
     800.0,  800.0,  800.0,           # index
     800.0,  800.0,  800.0,           # middle
     800.0,  800.0,  800.0,           # ring
     800.0,  800.0,  800.0,           # baby
])
```

길이가 16이 아니면 `ValueError`를, NaN이나 Inf가 있으면 `INVALID_ARGUMENT` 코드의 `Error`를
던집니다. 적용 중인 값은 `get_state().commanded.max_effort_pct`로 확인합니다.

### 4.2 Controllers

controller는 command를 drive에 전송할 값으로 변환합니다. 설정은
[`ControllerConfig`](11_python_api_reference/config.md#controllerconfig)에 담아
[`set_controller_config()`](11_python_api_reference/hand.md#handset_controller_config)로
적용하며, 다음 cycle부터 반영됩니다. 한 번도 호출하지 않으면 각 필드의 기본값이 적용됩니다.

다음은 joint position controller의 세 항목을 기본값과 같은 값으로 지정하는 예입니다.

```python
controller = ah2.ControllerConfig()
controller.joint_position_controller.filter_enabled = True
controller.joint_position_controller.cutoff_freq = 10.0      # Hz
controller.joint_position_controller.deadband = 0.000873     # rad
hand.set_controller_config(controller)
```

joint impedance controller의 `stiffness`·`damping` 필드는 길이 16인 numpy 배열입니다. 배열의 일부만
바꿔도 config에 그대로 반영됩니다.

```python
controller.joint_impedance_controller.stiffness[0:4] = 0.02    # thumb
hand.set_controller_config(controller)
```

| Field | Type | Default | Description |
|---|---|---|---|
| `joint_position_controller.filter_enabled` | `bool` | `True` | filter 사용 여부 |
| `joint_position_controller.cutoff_freq` | `float` | `10.0` | 차단 주파수 [Hz]. `0`이면 생략 |
| `joint_position_controller.deadband` | `float` | `0.000873` | 무시할 변화의 크기 [rad]. `0`이면 생략 |
| `joint_impedance_controller.stiffness` | `float64` `(16,)` | thumb `0.02`, 나머지 finger는 `0.01`·`0.01`·`0.02` | 위치 오차에 곱하는 gain |
| `joint_impedance_controller.damping` | `float64` `(16,)` | `1e-5` × 16 | 속도에 곱하는 gain |

값이 NaN·Inf이거나 음수면 `set_controller_config()`가 `INVALID_ARGUMENT` 코드의 `Error`를 던지고
기존 설정을 유지합니다. 각 단계가 하는 일과 값을 고르는 기준은 C++ guide의
[4.2.1 Joint position controller](07_cpp_usage_guide.md#421-joint-position-controller)와
[4.2.2 Joint impedance controller](07_cpp_usage_guide.md#422-joint-impedance-controller)에 있습니다.

> [!NOTE]
> joint impedance controller는 아직 개발 중이므로 사용하지 마십시오. 동작하지만 이후 버전에서
> 거동이 바뀔 수 있습니다.

## 5. Command

### 5.1 Command types

command는 5종류의 class로 구분되며,
[`set_command()`](11_python_api_reference/hand.md#handset_command)에 하나를 전달하여 전송합니다.
`Idle`은 값이 없고, 나머지 넷은 `target` 필드 하나를 갖습니다.

| Command class | `target` field | Input | Controller | Sent to drive |
|---|---|---|---|---|
| [`Idle`](11_python_api_reference/command.md#idle) | 없음 | 없음 | 없음 | actuator effort `0` |
| [`JointPositionCommand`](11_python_api_reference/command.md#jointpositioncommand) | `float64` `(16,)` | joint position [rad] | joint position | actuator position |
| [`JointImpedanceCommand`](11_python_api_reference/command.md#jointimpedancecommand) | `float64` `(16,)` | joint position [rad] | joint impedance | actuator effort |
| [`ActuatorPositionCommand`](11_python_api_reference/command.md#actuatorpositioncommand) | `float64` `(16,)` | actuator position [encoder count] | 없음 | actuator position |
| [`ActuatorEffortCommand`](11_python_api_reference/command.md#actuatoreffortcommand) | `float64` `(16,)` | actuator effort [정격 전류의 0.1%] | 없음 | actuator effort |

`target` 필드는 네 command 모두 길이가 16이고, thumb 4개 뒤에 나머지 finger가 3개씩 배치됩니다.
index 배치는 [API reference](11_python_api_reference/description.md#array-layout)에 있습니다.

command는 `target`을 생성자에 넘겨 만듭니다. list든 numpy 배열이든 길이 16인 숫자 배열이면 되고,
생략하면 전부 `0`입니다. 만든 뒤에는 `target` 필드의 값을 바로 바꿔도 됩니다.

```python
command = ah2.JointPositionCommand([
     0.20,  0.35,  0.10,  0.25,   # thumb   j0 j1 j2 j3
     0.08,  0.45,  0.30,          # index   j1 j2 j3
     0.04,  0.55,  0.40,          # middle
    -0.04,  0.65,  0.50,          # ring
    -0.08,  0.75,  0.60,          # baby
])
hand.set_command(command)

command.target[5] += 0.1          # index j2를 더 굽힙니다
hand.set_command(command)         # 바꾼 값은 다시 전송해야 적용됩니다
```

`Idle`은 drive를 활성 상태로 유지한 채 effort를 `0`으로 전송합니다. 토크를 완전히 제거하려면
[`stop()`](11_python_api_reference/hand.md#handstop)을 사용하십시오.

```python
hand.set_command(ah2.Idle())
```

`JointImpedanceCommand`는 `JointPositionCommand`와 같은 target을 받고, actuator position이 아니라
actuator effort를 전송합니다.

`ActuatorPositionCommand`의 target은 encoder count 단위이고 controller를 거치지 않습니다.
`ActuatorEffortCommand`의 target은 정격 전류의 0.1% 단위이며, 부호가 방향을 나타내고
±`max_effort` 값으로 제한된 뒤 전송됩니다.

```python
hand.set_command(ah2.ActuatorEffortCommand(np.full(ah2.ACTUATOR_COUNT, 300.0)))   # 30%
```

같은 index의 joint와 actuator는 서로 대응하지 않습니다. 두 공간의 변환은
[6. Kinematics](#6-kinematics)에 있습니다.

### 5.2 Validation and clamp

`set_command()`는 state를 검사하고, 값을 검사하고, joint command인 경우 target을 도달 범위로 투영한
뒤 command를 적용합니다.

- **state**: `lifecycle` 값이 `RUNNING`이고 `homing_state` 값이 `SUCCEEDED`여야 합니다. 아니면
  `WRONG_CALL_ORDER` 코드의 `Error`를 던집니다. `Idle`은 `homing_state` 조건에서 제외됩니다.
- **값**: target에 NaN·Inf가 있거나 `ActuatorPositionCommand`의 target이 `int32` 범위를 벗어나면
  command가 적용되지 않고 **직전 command가 계속 전송됩니다.** 예외는 없고 `nan_command_count` 값이
  1 늘며 warning log가 남습니다. target을 계산한 직후 `np.isfinite(command.target).all()`로 확인하기를
  권장합니다.
- **도달 범위**: joint command의 target은 finger별 도달 범위로 투영된 뒤 적용됩니다. 전송 전에 결과를
  보려면 [`clamp()`](11_python_api_reference/command.md#jointpositioncommandclamp)를 호출합니다.
  `clamp()`는 `target` 필드의 값을 그 자리에서 바꿉니다.

```python
command.clamp()                   # 투영 결과로 target을 바꿉니다
print(command.target)
```

범위 모델은 [Workspace limits](14_workspace_limits.md)에 있습니다.

> [!IMPORTANT]
> clamp는 도달 범위만 검사합니다. self-collision, 주변 물체, cable, payload는 검사하지 않습니다.
> 충돌·속도·힘 제한은 application이 따로 걸어야 합니다.

### 5.3 Command lifetime

command는 다음 `set_command()`까지 유지되지만, `RUNNING` 상태를 벗어나면 무효가 됩니다. 다시
`RUNNING` 상태가 될 때는 아래 command에서 시작하므로, 필요한 자세는 복귀한 뒤 다시 전송하십시오.

| Path back to `RUNNING` | First command applied |
|---|---|
| `STOPPED`에서 `run()` | 재개 시점의 자세를 유지하는 command |
| homing 성공 | target이 전부 `0`인 `ActuatorPositionCommand` |
| 나머지 경로 | `Idle` |

## 6. Kinematics

kinematics 함수는 joint 공간과 actuator 공간을 변환하는 모듈 함수입니다. `Hand` 없이도 호출할 수
있어, command를 전송하기 전에 target을 확인하거나 기록한 encoder 값을 나중에 joint 각도로 변환할
때 사용합니다.

| Function | Input | Output |
|---|---|---|
| [`fk_actuator_to_joint()`](11_python_api_reference/kinematics.md#fk_actuator_to_joint) | 길이 16, encoder count | `float64` `(21,)`, rad |
| [`ik_joint_to_actuator()`](11_python_api_reference/kinematics.md#ik_joint_to_actuator) | 길이 16, rad | `int32` `(16,)`, encoder count |

다음은 target을 actuator position으로 변환한 뒤 다시 joint 각도로 되돌리는 예입니다.

```python
encoder = ah2.ik_joint_to_actuator(command.target)   # int32, shape (16,)
joints = ah2.fk_actuator_to_joint(encoder)           # float64, shape (21,)
```

`get_state().joints.position_rad`는 SDK가 `fk_actuator_to_joint()`로 채운 값입니다.

> [!IMPORTANT]
> forward kinematics 결과는 passive joint를 포함한 21개이고 command의 target은 active joint 16개입니다.
> 결과를 command에 그대로 넘기면 길이가 달라 `ValueError`가 발생합니다. 두 배열의 index 대응은
> [API reference](11_python_api_reference/description.md#array-layout)에 있습니다.

## 7. HandState

[`HandState`](11_python_api_reference/state.md#handstate)는 로봇 핸드에서 읽은 값을 한 객체에 모은
것이며, [`get_state()`](11_python_api_reference/hand.md#handget_state)가 반환합니다.

```python
state = hand.get_state()
```

| Field | Type | Description |
|---|---|---|
| `timestamp` | `int` | 해당 cycle의 `CLOCK_REALTIME` epoch ns. `0`은 아직 값이 없다는 뜻 |
| `actuators` | [`ActuatorState`](11_python_api_reference/state.md#actuatorstate) | actuator 관측 |
| `joints` | [`JointState`](11_python_api_reference/state.md#jointstate) | forward kinematics 결과 |
| `tactile` | [`TactileState`](11_python_api_reference/state.md#tactilestate) | 촉각 |
| `commanded` | [`CommandedState`](11_python_api_reference/state.md#commandedstate) | 이번 cycle의 command 입력·출력 |

`get_state()`가 반환하는 값은 호출한 시점의 사본입니다. 객체를 가지고 있어도 값이 갱신되지
않으므로, 새 값이 필요하면 다시 호출하십시오. 안의 배열은 읽기 전용이며, 값을 바꿔 쓰려면
`.copy()`로 복사하십시오.

`get_state()`와 [`get_diagnostics()`](11_python_api_reference/hand.md#handget_diagnostics)는
각각 따로 읽으므로 두 호출 사이에 cycle이 하나 지날 수 있습니다. `timestamp` 필드는 해당 cycle의
`CLOCK_REALTIME`이라 `time.time_ns()`와 비교할 수 있지만, 주기 측정에는 `last_period_ms` 필드를
사용하십시오.

### 7.1 Actuators and joints

`actuators` 필드는 drive가 보고한 값이고, `joints` 필드는 actuator 값을 forward kinematics로
변환한 결과입니다.

| Field | dtype · shape | Unit |
|---|---|---|
| `actuators.position_count` | `int32` `(16,)` | encoder count |
| `actuators.velocity_rpm` | `int32` `(16,)` | rpm |
| `actuators.current_mA` | `int16` `(16,)` | mA |
| `joints.position_rad` | `float64` `(21,)` | rad |

### 7.2 Applied command

`state.commanded` 필드는 한 cycle의 처리 과정을 단계별로 담습니다.

| Field | Description |
|---|---|
| `controller_input` | 현재 적용 중인 command 객체 |
| `controller_output` | limit과 fault mask를 통과한 setpoint. `ActuatorPositionSetpoint`·`ActuatorEffortSetpoint`·`None` 중 하나 |
| `selected_source` | 이번 cycle에 실제로 사용한 출처: `NONE`·`CONTROLLER`·`QUICK_STOP`·`HOMING` |
| `max_effort_pct` | 적용 중인 actuator별 상한 |

`controller_output` 필드는 position 계열 command에서 `ActuatorPositionSetpoint`, effort 계열
command와 `Idle`에서 `ActuatorEffortSetpoint`이고, control session(`RUNNING`·`STOPPED`·`FAULTED`)
밖에서는 `None`입니다. 다음은 position setpoint와 측정 위치의 오차를 계산하는 예입니다.

```python
output = state.commanded.controller_output
if isinstance(output, ah2.ActuatorPositionSetpoint):
    error = output.target_position_cnt - state.actuators.position_count
```

추종을 확인할 때는 같은 공간끼리 비교하십시오. `target_position_cnt`와 `actuators.position_count`가
짝이고, effort 계열은 단위가 달라 측정값과 직접 비교할 수 없습니다. `controller_output` 필드에 값이
있다고 actuator가 그 값을 적용했다는 뜻은 아니므로 `selected_source` 값이 `CONTROLLER`인지 함께
확인하십시오. 자세한 설명은 [C++ guide](07_cpp_usage_guide.md#72-applied-command)에 있습니다.

### 7.3 Tactile

`state.tactile` 필드는 finger와 palm의 taxel 값입니다. 센서가 전송한 16-bit raw value를 그대로 담아
단위도 정규화도 없습니다.

| Field | dtype · shape | Description |
|---|---|---|
| `tactile.fingers` | `uint16` `(5, 17)` | 첫 번째 index가 [`Finger`](11_python_api_reference/description.md#enum-finger) 순서 |
| `tactile.palm` | `uint16` `(58,)` | palm1 upper `[0:20]`, lower `[20:40]`, palm2 `[40:58]` |

접촉 판정 임계값은 SDK가 제시하지 않습니다. 접촉이 없는 상태를 baseline으로 정하고 차이를
확인하십시오. 값이 `uint16`이므로 빼기 전에 부호 있는 정수로 바꾸십시오. 그대로 빼면 음수가 큰 양수로
바뀝니다.

```python
baseline = hand.get_state().tactile.fingers.astype(np.int32)

delta = hand.get_state().tactile.fingers.astype(np.int32) - baseline
index_finger = delta[ah2.Finger.INDEX]
```

## 8. Diagnostics

[`Diagnostics`](11_python_api_reference/diagnostics.md#diagnostics)는 SDK와 제어·통신 루프의 상태를
한 객체에 모은 것이며, [`get_diagnostics()`](11_python_api_reference/hand.md#handget_diagnostics)가
반환합니다.

```python
diag = hand.get_diagnostics()
```

정상 운전 중에는 Normal 열의 조건이 성립합니다. 값이 다르면 Check 열의 절을 참고하십시오.

| Field | Normal | Check |
|---|---|---|
| `lifecycle` | `RUNNING` | [9.4 Recovering from Faulted](#94-recovering-from-faulted) |
| `homing_state` | `SUCCEEDED` | [3. Homing](#3-homing) |
| `nan_command_count` | 늘지 않음 | [5.2 Validation and clamp](#52-validation-and-clamp) |
| `actuator_health` | fault 없음 | [8.1 Actuator faults](#81-actuator-faults) |

제어·통신 루프의 상태는 `control_cycles`·`deadline_misses`·`last_period_ms`·`last_compute_ms` 필드로
확인합니다. 네 값을 해석하는 방법은 [C++ guide](07_cpp_usage_guide.md#8-diagnostics)에 있습니다.
주기 준수 여부는 일정 구간의 증가율로 판단합니다.

```python
import time

before = hand.get_diagnostics()
time.sleep(1.0)
after = hand.get_diagnostics()
miss_rate = ((after.deadline_misses - before.deadline_misses)
             / max(after.control_cycles - before.control_cycles, 1))
```

### 8.1 Actuator faults

actuator fault는 `actuator_health.fault`에 actuator마다 담깁니다. 각 항목은
[`ActuatorFault`](11_python_api_reference/state.md#enum-actuatorfault)이고, 값은 drive가 보고한
error code입니다. enum에 정의되지 않은 code는 정수 그대로 담깁니다.

```python
diag = hand.get_diagnostics()
for index, fault in enumerate(diag.actuator_health.fault):
    if fault != ah2.ActuatorFault.NONE:
        print(f"actuator {index}: {getattr(fault, 'name', 'UNKNOWN')} 0x{fault:04X}")
```

fault가 발생해도 SDK는 스스로 정지하지 않습니다. fault가 있는 actuator는 command 적용 대상에서
제외되고 제어·통신 루프가 fault reset을 반복 시도하며, 대응은 application이 결정합니다. 값별 점검
항목은 [Error messages](15_error_messages.md#11-actuator-fault)에 있습니다. `disabled_actuators`
필드에 지정한 actuator는 보고되지 않습니다.

### 8.2 Diagnostic record

원인을 추적하려면 다음 항목을 같은 시각에 함께 남기십시오. 항목별 설명은
[C++ guide](07_cpp_usage_guide.md#82-diagnostic-record)에 있습니다.

- `state.timestamp` 필드와 읽은 시점의 `time.monotonic_ns()`
- `state.actuators`·`state.joints`·`state.commanded`
- 함께 읽은 `Diagnostics` 전체
- 적용 중이던 `ControllerConfig`. getter가 없으므로 application이 보관합니다

`controller_output` 필드가 `None`이면 control session 밖이라 setpoint가 없다는 뜻입니다. `None`을
`0`으로 기록하면 무토크 command와 구분되지 않습니다.

## 9. Error handling

예외는 SDK가 실패를 알리는 수단입니다. 실패란 호출이 요청한 동작을 완료하지 못한 경우를 말합니다.
actuator fault와 값이 유효하지 않아 적용되지 않은 command는 호출의 실패가 아니므로 예외 없이
진단으로 관측합니다([8. Diagnostics](#8-diagnostics)).

### 9.1 Exceptions

SDK의 실패는 [`Error`](11_python_api_reference/error.md#error)로 전달됩니다. `Error`는
`RuntimeError`를 상속하고, `code` 속성에 [`ErrorCode`](11_python_api_reference/error.md#enum-errorcode)
값을 담습니다. `str(error)`는 사람이 읽을 사유이며 code는 포함하지 않습니다.

SDK는 `Error`를 던지기 전에 `[exception] <ErrorCode>: <message>`를 error level로 log에 남깁니다.

```python
try:
    hand.run()
except ah2.Error as error:
    code = error.code           # 무엇이 잘못됐는지
    reason = str(error)         # 사람이 읽을 사유
```

`ErrorCode`는 점검 대상에 따라 세 묶음으로 나뉩니다.

| `ErrorCode` | Check | Retryable |
|---|---|---|
| `INVALID_ARGUMENT` · `WRONG_CALL_ORDER` | 호출한 코드 | 없습니다. 호출을 고쳐야 합니다 |
| `INTERFACE_UNAVAILABLE` · `COMMUNICATION_LOST` · `HARDWARE_FAULT` | 장비와 환경 | 원인이 해소되면 성공합니다 |
| `CONTROL_LOOP_FAULT` · `UNEXPECTED_ERROR` | SDK에 보고 | 없습니다 |

code는 log에 남기고 사람이 원인을 특정할 때 쓰는 값입니다. 복구 절차는 code가 아니라 현재
lifecycle로 정하십시오. 문구별 정리는 [Error messages](15_error_messages.md)에 있습니다.

인자의 형태가 맞지 않으면 Python의 표준 예외가 발생합니다. 배열 길이가 틀리면 `ValueError`, 타입이
틀리면 `TypeError`입니다. 이 둘은 SDK까지 전달되지 않은 호출이라 SDK log에 남지 않습니다.

### 9.2 Detecting a failure

실패를 감지하는 경로는 두 가지입니다.

| Path | Detects |
|---|---|
| 예외 | application이 호출한 함수가 실패한 경우 |
| `get_diagnostics().lifecycle` | 제어·통신 루프가 스스로 정지한 경우 |

로봇 핸드가 자세를 유지하는 동안 통신이 끊기면 다음 호출 전까지 예외가 발생하지 않으므로,
`lifecycle` 필드를 주기적으로 읽어야 제때 감지합니다. `get_state()`와 `get_diagnostics()`는
`FAULTED` 상태에서도 예외를 던지지 않습니다. 반대로 예외가 발생했다고 `FAULTED` 상태가 된 것은
아니며, 예외 없이 lifecycle이 바뀌는 경우도 있습니다. 상세는
[C++ guide](07_cpp_usage_guide.md#92-detecting-a-failure)에 있습니다.

### 9.3 Recovery patterns

복구를 어느 단계부터 수행할지는 현재 lifecycle이 결정합니다.

| Lifecycle | Resume from |
|---|---|
| `FAULTED` | `reconnect()` ([9.4 Recovering from Faulted](#94-recovering-from-faulted)) |
| `DISCONNECTED` | `connect()` |
| `CONNECTED` · `STOPPED` | `run()` |
| `RUNNING` | 곧바로 command 전송 |

구조는 lifecycle을 누가 결정하는가로 갈리며 C++ guide의 세 구조와 같습니다. 여기서는 루프가
lifecycle을 결정하는 self-managed 구조와, 실패하면 종료하는 구조의 Python 코드를 싣습니다. 외부가
`run`·`stop`·`disconnect`를 지시하는 구조는
[9.3.2 Externally managed lifecycle](07_cpp_usage_guide.md#932-externally-managed-lifecycle)을
보십시오.

**self-managed**

매 주기 lifecycle을 확인해 빠진 단계를 수행한 뒤 command를 전송합니다. 첫 주기에는
`DISCONNECTED` 상태이므로 같은 코드가 초기 연결까지 처리합니다. `Ctrl-C`를 누르면
`KeyboardInterrupt`가 `with` 블록을 벗어나며 로봇 핸드를 정지시킵니다.

```python
import time
import aidin_hand2 as ah2

POSE = [
     0.20,  0.35,  0.10,  0.25,   # thumb   j0 j1 j2 j3
     0.08,  0.45,  0.30,          # index   j1 j2 j3
     0.04,  0.55,  0.40,          # middle
    -0.04,  0.65,  0.50,          # ring
    -0.08,  0.75,  0.60,          # baby
]

config = ah2.HandConfig(
    "can0", ah2.HandSide.RIGHT,
    auto_home=True,                  # run()이 homing까지 처리합니다
    auto_reconnect=True,             # 통신 오류는 제어·통신 루프가 복구합니다
    auto_reconnect_home=True,
)
Lifecycle = ah2.HandLifecycle

with ah2.HandManager() as manager:
    hand = manager.create(config)
    hand.set_max_effort(1000.0)      # lifecycle과 무관하므로 여기서 설정합니다
    command = ah2.JointPositionCommand(POSE)

    while True:
        try:
            lifecycle = hand.get_diagnostics().lifecycle
            if lifecycle is Lifecycle.FAULTED:
                hand.reconnect()
                lifecycle = Lifecycle.CONNECTED
            if lifecycle is Lifecycle.DISCONNECTED:
                hand.connect()
                lifecycle = Lifecycle.CONNECTED
            if lifecycle in (Lifecycle.CONNECTED, Lifecycle.STOPPED):
                hand.run()
            hand.set_command(command)
        except ah2.Error:
            pass                     # 사유는 SDK가 log에 남겼습니다. 다음 주기에 남은 단계를 수행합니다
        time.sleep(0.01)
```

**exit on failure**

한 번 실행하고 끝나는 프로그램은 전체를 하나의 `try`로 감싸고, 실패하면 그대로 종료합니다. `with`
블록이 실패 경로에서도 로봇 핸드를 정지시킵니다. AIDIN Hand Gen2가 실제로 움직이므로 target과
effort 상한은 환경에 맞게 변경하십시오.

```python
import sys
import time
import aidin_hand2 as ah2

POSE = [
     0.20,  0.35,  0.10,  0.25,   # thumb   j0 j1 j2 j3
     0.08,  0.45,  0.30,          # index   j1 j2 j3
     0.04,  0.55,  0.40,          # middle
    -0.04,  0.65,  0.50,          # ring
    -0.08,  0.75,  0.60,          # baby
]

ah2.set_log_to_file("/var/log/my_robot/aidin-hand2.log")   # create()보다 먼저 정합니다

exit_code = 0
try:
    with ah2.HandManager() as manager:
        hand = manager.create(ah2.HandConfig("can0", ah2.HandSide.RIGHT, auto_home=True))
        hand.connect()
        hand.set_max_effort(1000.0)
        hand.run()                   # homing까지 여기서 끝납니다
        hand.set_command(ah2.JointPositionCommand(POSE))
        time.sleep(1.0)
except ah2.Error:
    exit_code = 1                    # 사유는 SDK가 log에 남겼습니다

ah2.flush_log()
sys.exit(exit_code)
```

### 9.4 Recovering from Faulted

`FAULTED` 상태에서 벗어나는 수단은 [`reconnect()`](11_python_api_reference/hand.md#handreconnect)
하나입니다. 복구를 누가 수행하는지는 `auto_reconnect` 값이 결정합니다.

| `auto_reconnect` | Recovery by | Application |
|---|---|---|
| `False` (기본) | application | `FAULTED`를 확인하고 `reconnect()`를 호출합니다 |
| `True` | 제어·통신 루프 | 통신 오류라면 대기합니다. 제어·통신 루프 예외라면 직접 호출해야 합니다 |

수동 복구는 `reconnect()`부터 순서대로 호출합니다.

```python
hand.reconnect()   # 성공하면 CONNECTED
hand.run()         # auto_home=True이면 homing까지 여기서 끝납니다
```

`reconnect()`는 `homing_state` 값과 직전 command를 되돌리고, max effort와 `ControllerConfig` 설정은
유지합니다. `auto_home=False`이면 `run()` 뒤에 `home()`을 호출해야 `Idle`이 아닌 command를 보낼 수
있습니다. 자동 복구의 동작과 제한 시간은
[9.4.2 Automatic recovery](07_cpp_usage_guide.md#942-automatic-recovery)에 있습니다.
