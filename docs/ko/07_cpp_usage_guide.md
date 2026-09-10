# C++ guide

AIDIN Hand Gen2는 5 손가락을 actuator 16개로 joint 21개(active 16 + passive 5)를 움직이는 로봇
핸드이고, 손끝, 손마디, 손바닥에 촉각 센서가 있습니다. 이 문서는 SDK를 user application에 연결해
제어하는 방법을 설명합니다.
</br>type 구성과 설정에서 시작해 상태 전이, homing, command, controller 설정, 한계, 관측, 오류 처리 순으로
다룹니다. 먼저 [SDK build & install](06_sdk_build_and_install.md)을 마치십시오.

모든 심볼은 `aidin_hand2` namespace에 있고, 이 문서의 코드는 `ah2`를 alias로 씁니다. 심볼과 필드 목록은
[API reference](08_cpp_api_reference.md)에 있습니다.

각 장 끝에는 장에서 설명한 내용을 실행해 확인하는 예제를 적었습니다. 예제는
[`cpp/examples/`](../../cpp/examples)에 있고 SDK를 빌드할 때 함께 만들어지므로, `cpp/build/` 아래의
같은 이름 실행 파일을 실행하면 됩니다. 인자는 왼손·오른손 구분과 interface 두 개이고, 기본값은
`right`와 `can0`입니다.

```bash
./cpp/build/04_homing left can0      # [right|left] [interface]
```

모든 예제는 결과를 터미널에 출력하고, `Ctrl-C`를 받으면 진행 중인 반복을 중단한 뒤 quick stop을 확인하고
연결을 끊고 종료합니다. 예제마다 첫 주석에 확인 대상과 AIDIN Hand Gen2가 실제로 움직이는지를
적었으므로 실행 전에 읽으십시오.

## Contents

&nbsp;&nbsp;[**1. HandManager and Hand**](#1-handmanager-and-hand)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.1 Types](#11-types)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.2 HandConfig](#12-handconfig)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.3 Create and destroy](#13-create-and-destroy)<br>
&nbsp;&nbsp;[**2. Lifecycle**](#2-lifecycle)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.1 Lifecycle states](#21-lifecycle-states)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.2 State transition calls](#22-state-transition-calls)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.3 Transition confirmation](#23-transition-confirmation)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.3.1 Confirmation criteria](#231-confirmation-criteria)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[2.3.2 Confirmation failure](#232-confirmation-failure)<br>
&nbsp;&nbsp;[**3. Homing**](#3-homing)<br>
&nbsp;&nbsp;[**4. Control settings**](#4-control-settings)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.1 Max effort](#41-max-effort)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.2 Controllers](#42-controllers)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.2.1 Joint position controller](#421-joint-position-controller)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[4.2.2 Joint impedance controller](#422-joint-impedance-controller)<br>
&nbsp;&nbsp;[**5. Command**](#5-command)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[5.1 Command types](#51-command-types)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[5.2 Validation and clamp](#52-validation-and-clamp)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[5.2.1 Rejected by state](#521-rejected-by-state)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[5.2.2 Rejected by value](#522-rejected-by-value)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[5.2.3 Workspace clamp](#523-workspace-clamp)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[5.3 Command lifetime](#53-command-lifetime)<br>
&nbsp;&nbsp;[**6. Kinematics**](#6-kinematics)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[6.1 Conversion functions](#61-conversion-functions)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[6.2 Active and passive joints](#62-active-and-passive-joints)<br>
&nbsp;&nbsp;[**7. HandState**](#7-handstate)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[7.1 Actuators and joints](#71-actuators-and-joints)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[7.2 Applied command](#72-applied-command)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[7.3 Tactile](#73-tactile)<br>
&nbsp;&nbsp;[**8. Diagnostics**](#8-diagnostics)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[8.1 Actuator faults](#81-actuator-faults)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[8.2 Diagnostic record](#82-diagnostic-record)<br>
&nbsp;&nbsp;[**9. Error handling**](#9-error-handling)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[9.1 Exception](#91-exception)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[9.2 Detecting a failure](#92-detecting-a-failure)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[9.3 Recovery patterns](#93-recovery-patterns)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[9.3.1 Self-managed lifecycle](#931-self-managed-lifecycle)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[9.3.2 Externally managed lifecycle](#932-externally-managed-lifecycle)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[9.3.3 Exit on failure](#933-exit-on-failure)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[9.4 Recovering from Faulted](#94-recovering-from-faulted)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[9.4.1 Manual recovery](#941-manual-recovery)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[9.4.2 Automatic recovery](#942-automatic-recovery)

## 1. HandManager and Hand

### 1.1 Types

SDK는 [`HandManager`](08_cpp_api_reference/hand_manager.md#handmanager)와 [`Hand`](08_cpp_api_reference/hand.md#hand)를 통해 AIDIN Hand Gen2에 접근합니다.

| Type | Description |
|---|---|
| `HandManager` | `HandConfig`를 검증해 `HandCore`를 생성하고 파기하는 factory. `Hand`를 반환합니다 |
| `Hand` | `HandCore` 하나를 참조하는 handle. 연결·제어·설정·조회를 수행합니다 |

`HandCore`는 CAN-FD 통신을 통해 AIDIN Hand Gen2를 제어하는 내부 클래스입니다. public header에 정의가 없어 직접
생성하거나 참조할 수 없고, `HandManager`가 소유하며 `destroy()`로 파기합니다.

### 1.2 HandConfig

[`HandConfig`](08_cpp_api_reference/types_config.md#handconfig)는 `Hand`의 동작을 설정합니다. `interface_name`과 `hand_side` 필드는 생성자
인자이고, 나머지 필드는 기본값이 존재합니다.

다음은 모든 필드를 명시하여 config를 구성하는 예입니다.

```cpp
// 기본값이 아니라 설정 예시입니다. 기본값은 아래 표에 있습니다.
ah2::HandConfig config{"can0", ah2::HandSide::Left};   // 생성자 인자 2개는 필수
config.control_rate              = 500;                // Hz
config.auto_home                 = false;
config.rt_cpu_affinity           = -1;
config.auto_reconnect            = true;
config.auto_reconnect_timeout_ms = 5000;               // ms
config.auto_reconnect_home       = true;
config.disabled_actuators        = {};                 // 예: {3, 7}
```

| Field | Type | Default | Description |
|---|---|---|---|
| `interface_name` | `std::string` | 필수 | CAN interface 이름 |
| `hand_side` | [`HandSide`](08_cpp_api_reference/types_description.md#enum-handside) | 필수 | 로봇 핸드 좌우 구분 (왼손/오른손) |
| `control_rate` | `int` | `500` | 제어 주기 [Hz] |
| `auto_home` | `bool` | `true` | [`run()`](08_cpp_api_reference/hand.md#handrun)의 자동 homing 여부.</br> `homing_state` 값이 `Succeeded`가 아닐 때만 수행 |
| `rt_cpu_affinity` | `int` | `-1` | 제어·통신 루프를 실행하는 thread를 고정할 CPU 번호.</br> `-1`은 고정 안 함.|
| `auto_reconnect` | `bool` | `false` | 통신 오류 발생 시, 자동 재연결 시도 여부 |
| `auto_reconnect_timeout_ms` | `int` | `0` | 자동 재연결 제한 시간 [ms]. </br>`0`은 제한 없음. `auto_reconnect=false`면 무시 |
| `auto_reconnect_home` | `bool` | `false` | 자동 재연결에서 자동 homing 여부. </br> 수동 [`reconnect()`](08_cpp_api_reference/hand.md#handreconnect)에는 적용되지 않음 |
| `disabled_actuators` | `std::vector<int>` | `{}` | 사용하지 않을 [actuator index](08_cpp_api_reference/types_description.md#actuator-index) |

생성자와 `create()`는 다음 두 경우를 거부합니다.

- `interface_name` 필드가 비었거나 `hand_side` 필드가 정의되지 않은 값이면 생성자가 예외를 던집니다.
- `control_rate` 값이 `0` 이하이면 `create()`가 `InvalidArgument` 예외를 던집니다.

`disabled_actuators` 필드는 전원을 인가하지 않을 actuator를 지정합니다. 물리적으로 없는 actuator 또는
고장으로 당장 사용하지 않을 actuator를 제외하고 구동할 때 사용합니다.
되도록 **finger 단위**로 지정하십시오.

### 1.3 Create and destroy

[`create()`](08_cpp_api_reference/hand_manager.md#handmanagercreate)는 config를 검증하고 `HandCore`를 생성한 뒤 해당 `HandCore`를 참조하는 `Hand`를
반환합니다. CAN 연결은 [`connect()`](08_cpp_api_reference/hand.md#handconnect)가 수행합니다.

```cpp
ah2::HandManager manager;
ah2::Hand hand = manager.create(config);
```

`Hand`는 복사해도 같은 `HandCore`를 참조합니다. handle이 하나 증가할 뿐 `HandCore`가 증가하지는 않습니다.

```cpp
ah2::Hand copy = hand;       // 같은 hand를 참조합니다
manager.destroy(hand);       // copy도 함께 무효가 됩니다
```

- [`destroy()`](08_cpp_api_reference/hand_manager.md#handmanagerdestroy)·[`destroy_all()`](08_cpp_api_reference/hand_manager.md#handmanagerdestroy_all)·manager 소멸 뒤에는 파기된 `HandCore`를 참조하던 `Hand`가 모두
  무효가 되고, method를 호출하면 예외를 던집니다.
- `destroy()`는 연결 중이면 actuator quick stop을 확인하고 연결을 끊은 뒤 `HandCore`를 파기하므로,
  `destroy()` 앞의 [`disconnect()`](08_cpp_api_reference/hand.md#handdisconnect)는 생략해도 됩니다.
- `Hand`는 thread-safe하지 않습니다. 여러 thread에서 같은 `Hand`를 호출한다면 동기화는
  application의 책임입니다.

`HandManager` 하나가 다수의 `HandCore`를 소유할 수 있습니다. 로봇 핸드 두 대를 함께 사용하려면
`interface_name`과 `hand_side` 필드가 다른 config로 `create()`를 두 번 호출하십시오. hand마다
`HandCore`와 제어·통신 루프가 따로 있어 서로 독립적으로 동작하고, `destroy_all()`이나 manager
소멸은 소유한 `HandCore`를 모두 파기합니다.

> [!TIP]
> 이 장의 내용은 다음 예제로 확인할 수 있습니다.
>
> - [`01_create_and_destroy.cpp`](../../cpp/examples/01_create_and_destroy.cpp) — 자원은
>   `HandManager`가 소유하고 `Hand`는 사본을 만들어도 같은 자원을 참조하며, `destroy()` 호출 뒤에는
>   사본까지 예외를 던집니다. 로봇 핸드 없이도 실행됩니다.
> - [`02_two_hands.cpp`](../../cpp/examples/02_two_hands.cpp) — `HandManager` 하나로 로봇 핸드 두
>   개를 만들고, 각 로봇 핸드의 cycle이 서로 무관하게 진행되는 것을 출력합니다.

## 2. Lifecycle

### 2.1 Lifecycle states

[`Hand`](08_cpp_api_reference/hand.md#hand)의 lifecycle은 [`HandLifecycle`](08_cpp_api_reference/types_state.md#enum-handlifecycle) enum에 정의된 다음 5개 state 중 하나입니다. 현재
lifecycle은 `get_diagnostics()`로 확인합니다.</br></br>
![Hand lifecycle 상태 전이](../assets/hand_lifecycle.webp)</br></br>

| State | Description |
|---|---|
| `Disconnected` | CAN socket이 열리지 않은 상태입니다. `create()` 직후와 `disconnect()` 완료 후의 상태입니다. |
| `Connected` | CAN socket이 열려 있고 state를 수신하지만 제어 명령은 송신하지 않는 상태입니다. |
| `Running` | actuator enable이 확인되어 제어 명령을 송신하는 상태입니다. `set_command()`는 `Running`에서만 동작합니다. |
| `Stopped` | actuator의 quick stop이 확인된 무토크 상태입니다. |
| `Faulted` | 통신 오류나 제어·통신 루프 예외로 인해 통신과 제어가 종료된 상태입니다. `reconnect()`를 통해서만 다른 state로 전이할 수 있습니다. |


### 2.2 State transition calls

상태 전이 함수는 아래 표의 precondition에 해당하는 lifecycle에서 호출할 수 있습니다. 허용되지
않은 state에서 호출하면 `WrongCallOrder` 예외가 발생합니다. 문구별 정리는
[Error messages](15_error_messages.md#4-wrongcallorder)에 있습니다.

각 함수는 요청을 전달한 뒤 곧바로 반환하지 않습니다. 표의 confirmation 조건이 확인될 때까지
대기하며, 제한 시간 안에 확인되지 않으면 예외를 던집니다.

| Method | Precondition | Postcondition | Confirmation | Timeout |
|---|---|---|---|---|
| [`connect()`](08_cpp_api_reference/hand.md#handconnect) | `Disconnected` | `Connected` | 첫 state 수신 | 300 ms |
| [`disconnect()`](08_cpp_api_reference/hand.md#handdisconnect) | `Connected` · `Running` · `Stopped` | `Disconnected` | actuator quick stop | 500 ms |
| [`run()`](08_cpp_api_reference/hand.md#handrun) | `Connected` · `Stopped` | `Running` | actuator enable | 4000 ms |
| [`home()`](08_cpp_api_reference/hand.md#handhome) | `Connected` · `Running` · `Stopped` | `Running` | actuator enable과 homing 완료 | 500 Hz에서 최대 22 s |
| [`stop()`](08_cpp_api_reference/hand.md#handstop) | `Running` | `Stopped` | actuator quick stop | 500 ms |
| [`reconnect()`](08_cpp_api_reference/hand.md#handreconnect) | `Faulted` | `Connected` | 첫 state 수신 | 300 ms |

`auto_home=true`이고 `homing_state` 값이 `Succeeded`가 아니면 `run()`이 homing까지 수행합니다. 이
경우 `run()`에도 `home()`과 같은 제한 시간이 적용됩니다.

`home()`의 제한 시간은 homing 단계별 cycle 예산의 합으로 결정되므로 `control_rate` 값에 따라
달라집니다. 나머지 상태 전이 함수의 제한 시간은 `control_rate`와 무관합니다.

이미 postcondition을 만족하는 state에서 같은 함수를 호출하는 경우에는 별도의 동작 없이 성공하고
로그만 남깁니다.

- `Connected`에서 `connect()`
- `Disconnected`에서 `disconnect()`
- `Running`에서 `run()`
- `Stopped`에서 `stop()`

단, postcondition이 이미 **확인된 경우**에만 즉시 성공합니다. state 값은 해당 postcondition이지만
아직 전이 완료가 확인되지 않은 경우에는 확인을 계속 대기합니다.

`home()`의 상세 절차는 [3. Homing](#3-homing)에서 설명합니다. 상태 전이가 적용 중인 command에
미치는 영향은 [5.3 Command lifetime](#53-command-lifetime)을 참고하십시오.

> [!NOTE]
> 다음 함수는 lifecycle과 관계없이 `Faulted`를 포함한 모든 state에서 호출할 수 있습니다.
>
> - [`set_max_effort()`](08_cpp_api_reference/hand.md#handset_max_effort)
> - [`set_controller_config()`](08_cpp_api_reference/hand.md#handset_controller_config)
> - [`get_state()`](08_cpp_api_reference/hand.md#handget_state)
> - [`get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics)
> - [`get_command_mode()`](08_cpp_api_reference/hand.md#handget_command_mode)
>
> [`set_command()`](08_cpp_api_reference/hand.md#handset_command)는 예외입니다. lifecycle을 변경하지는 않지만 `Running`에서만 호출할 수
> 있으며, `Idle`을 제외한 command는 `homing_state` 값이 `Succeeded`여야 합니다. 자세한 조건은
> [5.2 Validation and clamp](#52-validation-and-clamp)를 참고하십시오.

### 2.3 Transition confirmation

상태 전이 함수는 전이 요청을 보낸 뒤 실제 동작이 확인될 때까지 대기합니다. 확인이 완료되어야
함수가 성공적으로 반환합니다. 함수별 확인 대상은 다음과 같습니다.

- `connect()`와 `reconnect()`는 CAN을 통해 첫 state가 수신되는지 확인합니다.
- `run()`은 필요한 actuator가 enable되었는지 확인합니다.
- `stop()`과 `disconnect()`는 필요한 actuator가 quick stop에 도달했는지 확인합니다.
- `home()`은 actuator enable과 homing 완료를 확인합니다.

제한 시간은 [2.2 State transition calls](#22-state-transition-calls)의 표를 따릅니다.

#### 2.3.1 Confirmation criteria

전이 확인에 포함되는 actuator는 enable과 quick stop에서 서로 다릅니다.

▪ **enable**

- `disabled_actuators` 필드에 지정된 actuator와 fault가 발생한 actuator는 확인 대상에서 제외합니다.
- 제외 후 남은 actuator가 하나 이상이어야 하며, 남은 actuator가 모두 enable되어야 확인에
  성공합니다.
- 따라서 일부 actuator에 fault가 발생하더라도 나머지 actuator가 모두 enable되면 `Running`으로
  전이할 수 있습니다.

▪ **quick stop**

- `disabled_actuators` 필드에 지정된 actuator와 한 번도 동작한 적이 없는 actuator는 확인 대상에서
  제외합니다. 한 번도 동작하지 않은 actuator에는 해제할 토크가 없기 때문입니다.
- actuator fault는 quick stop 확인 대상에서 제외하는 조건이 아닙니다.
- fault가 발생하기 전에 동작했던 actuator는 fault 이후에도 quick stop 상태인지 확인합니다.

#### 2.3.2 Confirmation failure

제한 시간 안에 confirmation 조건이 확인되지 않으면 상태 전이 함수는 예외를 던집니다.

timeout 이후의 lifecycle과 SDK 동작은 호출한 함수에 따라 다릅니다. 같은 함수를 다시 호출하면 직전
호출 상태를 기억해 이어서 처리하는 것이 아니라, **호출 시점의 현재 lifecycle과 actuator state를
기준으로 조건을 다시 확인합니다.**

| Method | Lifecycle&nbsp;after&nbsp;timeout | Behavior |
|---|---|---|
| `connect()` | `Disconnected` | socket을 닫고 `Disconnected`로 되돌아갑니다. |
| `run()` | 변화 없음 | actuator에 quick stop을 지시합니다. 이후 quick stop이 확인되면 `Stopped`가 됩니다. |
| `stop()` | 변화 없음 | quick stop 지시는 유지됩니다. drive가 마지막으로 수신한 목표를 유지하고 있을 수 있습니다. |
| `disconnect()` | 변화 없음 | CAN 연결을 끊지 않습니다. 강제로 종료해야 하면 [`destroy()`](08_cpp_api_reference/hand_manager.md#handmanagerdestroy)를 호출하십시오. |
| `reconnect()` | `Faulted` | socket을 닫은 상태로 `Faulted`에 남습니다. 다시 `reconnect()`를 호출할 수 있습니다. |

confirmation을 대기하는 동안 fault가 발생하면 timeout까지 대기하지 않습니다. 발생한 fault의 원인을
예외로 반환하고 lifecycle은 `Faulted`로 전이합니다. 이후 복구는 `reconnect()`로 수행합니다.

> [!TIP]
> 이 장의 내용은 다음 예제로 확인할 수 있습니다.
>
> - [`03_lifecycle_walkthrough.cpp`](../../cpp/examples/03_lifecycle_walkthrough.cpp) — 숫자 키로
>   전이 함수를 선택해 호출합니다. 어느 lifecycle에서든 모든 함수를 호출할 수 있어 거부되는 경우도
>   확인할 수 있고, 상태 줄이 계속 갱신되므로 CAN cable을 분리하면 `Faulted`로 전이하는 과정이
>   나타납니다.

## 3. Homing

homing은 각 finger를 hard stop까지 밀어 hard stop 지점을 원점으로 삼는 절차입니다. homing이
완료되기 전에는 절대 기준 위치가 존재하지 않습니다. 그래서
[`homing_state`](08_cpp_api_reference/types_state.md#enum-homingstate)가 `Succeeded`가 되기 전까지는
[`Idle`](08_cpp_api_reference/types_command.md#idle)을 뺀 모든 [`set_command()`](08_cpp_api_reference/hand.md#handset_command)가 `WrongCallOrder` 예외로 거부됩니다.

[`home()`](08_cpp_api_reference/hand.md#handhome)은 완료까지 대기하고, 실패하면 예외를 던집니다. 다음은 예외를 처리하는 예입니다.

```cpp
try {
  hand.home();
} catch (const ah2::Exception& error) {
  // 다시 시도하거나 actuator fault를 확인합니다
}
```

- `Running`이 아닌 상태에서 호출하면 homing이 actuator를 enable하고, enable이 확인된 시점에
  `Running`이 됩니다. 그래서 [`run()`](08_cpp_api_reference/hand.md#handrun)을 먼저 호출할 필요가 없습니다.
- `auto_home=true`이면 `run()`이 homing을 자동으로 진행합니다. `homing_state` 값이 `Succeeded`가
  아니면 `Running`에서 호출한 `run()`도 homing을 시작하므로, finger가 hard stop까지 이동합니다.
- 성공하면 원점 자세를 유지하는 command가 적용됩니다. 이어서 필요한 command를 전송하십시오.
  자세한 내용은 [5.3 Command lifetime](#53-command-lifetime)에 있습니다.
- 실패하면 `HardwareFault` 예외를 던지고 message에 실패한 actuator index가 포함됩니다.
- [`reconnect()`](08_cpp_api_reference/hand.md#handreconnect)는 `homing_state` 값을 `NotRun`으로 되돌리므로 homing을 다시 진행해야 합니다.

> [!WARNING]
> finger가 완전히 펴지지 못하게 막혀 있으면 막힌 지점이 hard stop으로 인식되어 원점이
> 어긋난 채 성공으로 보고됩니다. homing 전에 주변을 비우고 완료까지 접촉하지 마십시오.
> 완료는 `homing_state` 값이 `Succeeded`인지 또는 log의 `homing complete`로 확인하십시오.

> [!TIP]
> 이 장의 내용은 다음 예제로 확인할 수 있습니다.
>
> - [`04_homing.cpp`](../../cpp/examples/04_homing.cpp) — homing 전에 joint command를 전송해
>   `WrongCallOrder` 예외로 거부되는 것을 확인하고, homing 뒤에 같은 command가 적용되는 것을
>   확인합니다. encoder count를 homing 전후로 함께 출력합니다.

## 4. Control settings

control setting은 max effort와 controller 설정 두 가지이며, command가 drive 전송값으로 변환되는
방식을 결정합니다. SDK가 매 cycle drive에 전송하는 값은 actuator마다 position(encoder count)과
effort(정격 전류의 0.1%, 부호가 방향) 둘 중 하나입니다. 정격 전류는 모든 모터가 400 mA이므로
`1000`(100%)이 400 mA입니다.

두 설정은 lifecycle과 무관하게 언제든 호출할 수 있고
[`reconnect()`](08_cpp_api_reference/hand.md#handreconnect) 이후에도 유지됩니다.

### 4.1 Max effort

max effort는 actuator로 전송되는 effort의 상한입니다. 단위는 정격 전류의 0.1%이고 기본값은
`1000`입니다. `[0, 2000]` 밖의 값을 전달하면 해당 범위로 제한하여 적용합니다.

상한은 [`set_max_effort()`](08_cpp_api_reference/hand.md#handset_max_effort)로 설정합니다. actuator
전체에 같은 값을 적용하려면 `double` 하나를 전달합니다.

```cpp
hand.set_max_effort(1000.0);
```

actuator마다 다르게 적용하려면 `std::array<double, kActuatorCount>`를 전달합니다. 다음 값은
예시입니다.

```cpp
const std::array<double, ah2::kActuatorCount> max_effort = {
    1000.0, 1000.0, 1000.0, 1000.0,   // thumb
     800.0,  800.0,  800.0,           // index
     800.0,  800.0,  800.0,           // middle
     800.0,  800.0,  800.0,           // ring
     800.0,  800.0,  800.0};          // baby
hand.set_max_effort(max_effort);
```

적용 중인 값은 `state.commanded.max_effort_pct` 필드로 확인합니다.

### 4.2 Controllers

controller는 command를 drive에 전송할 값으로 변환합니다. 변환에 사용하는 filter와 gain은
[`ControllerConfig`](08_cpp_api_reference/types_config.md#controllerconfig)에 있고,
[`set_controller_config()`](08_cpp_api_reference/hand.md#handset_controller_config)가 구조체 전체를
인자로 받으므로 두 controller의 설정이 함께 적용됩니다. 호출 시점에는 제약이 없고 다음 cycle부터
반영됩니다. 한 번도 호출하지 않으면 각 필드의 기본값이 적용됩니다.

값이 non-finite이거나 음수면 `set_controller_config()`가 `InvalidArgument` 예외를 던지고 기존
설정을 유지합니다.

#### 4.2.1 Joint position controller

joint position controller는 joint 각도 명령을 actuator position으로 변환합니다. 입력된 명령은 매
cycle 다음 세 단계를 거칩니다.

1. 직전에 통과한 명령에서 deadband 이내로 움직인 입력은 무시합니다.
2. 3차 low-pass filter가 적용됩니다. `cutoff_freq` 값보다 빠른 변화가 줄어듭니다.
3. [`ik_joint_to_actuator()`](08_cpp_api_reference/hand_kinematics.md#ik_joint_to_actuator)로 actuator position으로 변환됩니다.

deadband는 입력에 섞인 noise를 억제하고, low-pass filter는 SDK 제어 주기보다 낮은 rate로 들어와
불연속한 입력을 여러 cycle에 걸쳐 반영합니다. `deadband`나 `cutoff_freq` 값을 `0`으로 설정하면
해당 단계만 생략됩니다.

다음은 `joint_position_controller` 필드의 세 항목을 기본값과 같은 값으로 지정하는 예입니다.

```cpp
ah2::ControllerConfig controller;
controller.joint_position_controller.filter_enabled = true;
controller.joint_position_controller.cutoff_freq    = 10.0;      // Hz
controller.joint_position_controller.deadband       = 0.000873;  // rad
hand.set_controller_config(controller);
```

| Field | Type | Default | Description |
|---|---|---|---|
| `filter_enabled` | `bool` | `true` | filter 사용 여부. `false`면 두 단계를 모두 생략 |
| `cutoff_freq` | `double` | `10.0` | 차단 주파수 [Hz]. `0`이면 생략 |
| `deadband` | `double` | `0.000873` | 무시할 변화의 크기 [rad]. `0`이면 생략 |

`cutoff_freq` 값은 입력이 실제로 갱신되는 주기를 기준으로 결정합니다. `set_command()`를 호출하는
주기는 기준이 아닙니다. 낮은 값에서 시작해 올려 가며 조정하십시오. 낮추면 지연이 커지고 높이면
filter 효과가 감소합니다. 어느 값에서도 overshoot은 발생하지 않습니다.

`deadband` 필드가 필요한 이유는 감속비가 크기 때문입니다. joint에서 미세한 변동이라도 모터축에서는
큰 반전이 되어 backlash 구간을 반복해서 통과합니다. 입력에 섞인 변동의 크기를 확인하며
조정하십시오.

#### 4.2.2 Joint impedance controller

joint impedance controller는 joint 각도 명령을 actuator effort로 변환합니다. 명령을
`ik_joint_to_actuator()`로 목표 actuator position으로 변환한 뒤, encoder 공간에서 다음 식으로
effort를 산출합니다.

```text
effort = stiffness × position_error − damping × velocity
```

접촉 힘이 변위에 비례하므로 파지력을 제어할 때 사용합니다. 두 gain은 길이가 `kActuatorCount`이고,
joint가 아니라 **actuator 공간**의 값입니다.

다음은 `joint_impedance_controller` 필드의 두 gain을 기본값과 같은 값으로 지정하는 예입니다.

```cpp
ah2::ControllerConfig controller;
controller.joint_impedance_controller.stiffness = {
    0.02, 0.02, 0.02, 0.02,   // thumb   a0 a1 a2 a3
    0.01, 0.01, 0.02,         // index   a1 a2 a3
    0.01, 0.01, 0.02,         // middle
    0.01, 0.01, 0.02,         // ring
    0.01, 0.01, 0.02};        // baby
controller.joint_impedance_controller.damping = {
    1e-5, 1e-5, 1e-5, 1e-5,   // thumb
    1e-5, 1e-5, 1e-5,         // index
    1e-5, 1e-5, 1e-5,         // middle
    1e-5, 1e-5, 1e-5,         // ring
    1e-5, 1e-5, 1e-5};        // baby
hand.set_controller_config(controller);
```

| Field | Type | Default | Description |
|---|---|---|---|
| `stiffness` | `std::array<double, kActuatorCount>` | thumb `0.02`, 나머지 finger는 `0.01`·`0.01`·`0.02` | 위치 오차에 곱하는 gain |
| `damping` | `std::array<double, kActuatorCount>` | `1e-5` × 16 | 속도에 곱하는 gain |

> [!NOTE]
> joint impedance controller는 아직 개발 중이므로 사용하지 마십시오. 동작하지만 이후 버전에서
> 거동이 바뀔 수 있습니다.

> [!TIP]
> 이 장의 내용은 다음 예제로 확인할 수 있습니다. controller마다 파일이 하나씩입니다.
>
> - [`05_max_effort.cpp`](../../cpp/examples/05_max_effort.cpp) — 같은 command를 유지한 채 상한만
>   세 번 올리며 측정 전류가 함께 증가하는 것을 출력합니다. `[0, 2000]` 밖의 값이 예외 없이
>   제한되는 것도 확인합니다.
> - [`06_position_controller.cpp`](../../cpp/examples/06_position_controller.cpp) — 같은 계단 입력을
>   filter를 끈 상태와 차단 주파수가 다른 두 상태에서 각각 전송해 setpoint를 나란히 출력하고,
>   `deadband` 값보다 작은 변화가 무시되는 것을 확인합니다.
> - [`07_impedance_controller.cpp`](../../cpp/examples/07_impedance_controller.cpp) — 같은 target을
>   유지한 채 `stiffness` 필드만 올리며 effort setpoint가 증가하는 것을 출력합니다. finger를 붙잡아
>   변위를 만들면 값이 달라집니다.

## 5. Command

### 5.1 Command types

command는 5종류의 struct로 구분되며, [`set_command()`](08_cpp_api_reference/hand.md#handset_command)에
하나를 전달하여 전송합니다. `Idle`은 빈 struct이고, 나머지 넷은 field가 `target` 하나뿐입니다.
전송한 command가 유지되는 기간은 [5.3 Command lifetime](#53-command-lifetime)에 있습니다.

| Command struct | `target` field | Input | Controller | Sent to drive |
|---|---|---|---|---|
| [`Idle`](08_cpp_api_reference/types_command.md#idle) | 없음 | 없음 | 없음 | actuator effort `0` |
| [`JointPositionCommand`](08_cpp_api_reference/types_command.md#jointpositioncommand) | `std::array<double, kActiveJointCount>` | joint position [rad] | joint position | actuator position |
| [`JointImpedanceCommand`](08_cpp_api_reference/types_command.md#jointimpedancecommand) | `std::array<double, kActiveJointCount>` | joint position [rad] | joint impedance | actuator effort |
| [`ActuatorPositionCommand`](08_cpp_api_reference/types_command.md#actuatorpositioncommand) | `std::array<double, kActuatorCount>` | actuator position [encoder count] | 없음 | actuator position |
| [`ActuatorEffortCommand`](08_cpp_api_reference/types_command.md#actuatoreffortcommand) | `std::array<double, kActuatorCount>` | actuator effort [정격 전류의 0.1%] | 없음 | actuator effort |

`target` 필드는 네 command 모두 길이가 16이고 finger별 index 묶음이 동일합니다. thumb 4개 뒤에 나머지
finger가 3개씩 배치됩니다. 세 index 공간의 정의는
[API reference](08_cpp_api_reference/types_description.md#배열-규약)에 있습니다.

다음은 command마다 값을 채워 전송하는 예입니다.

`Idle`은 target field를 갖지 않습니다. drive를 활성 상태로 유지한 채 effort를 `0`으로 전송하므로
joint를 외력으로 움직이면 저항이 발생하며, 토크를 완전히 제거하려면
[`stop()`](08_cpp_api_reference/hand.md#handstop)을 사용하십시오. 다음은 빈 struct를 전달하는 예입니다.

```cpp
ah2::Idle idle;
hand.set_command(idle);
```

`JointPositionCommand`의 target은 rad 단위입니다.

```cpp
ah2::JointPositionCommand joint_position;
joint_position.target = {
    0.20,  0.35,  0.10, 0.25,   // thumb   j0 j1 j2 j3
    0.08,  0.45,  0.30,         // index   j1 j2 j3
    0.04,  0.55,  0.40,         // middle
   -0.04,  0.65,  0.50,         // ring
   -0.08,  0.75,  0.60};        // baby
hand.set_command(joint_position);
```

`JointImpedanceCommand`의 target도 rad 단위입니다. 동일한 target을 `JointPositionCommand` 대신
`JointImpedanceCommand`로 전달하면 actuator position이 아니라 actuator effort가 전송됩니다.

```cpp
ah2::JointImpedanceCommand joint_impedance;
joint_impedance.target = {
    0.20,  0.35,  0.10, 0.25,   // thumb   j0 j1 j2 j3
    0.08,  0.45,  0.30,         // index   j1 j2 j3
    0.04,  0.55,  0.40,         // middle
   -0.04,  0.65,  0.50,         // ring
   -0.08,  0.75,  0.60};        // baby
hand.set_command(joint_impedance);
```

`ActuatorPositionCommand`의 target은 encoder count 단위이고 controller를 거치지 않습니다. 다음 값은
`JointPositionCommand`에 지정한 target을
[`ik_joint_to_actuator()`](08_cpp_api_reference/hand_kinematics.md#ik_joint_to_actuator)로 변환한
결과이므로 동일한 자세에 해당합니다.

```cpp
ah2::ActuatorPositionCommand actuator_position;
actuator_position.target = {
    26818, 85284, 104265, 26142,  // thumb   a0 a1 a2 a3
    53608, 46004, 17883,          // index   a1 a2 a3
    64654, 60853, 26413,          // middle
    74720, 78521, 35984,          // ring
    87527, 95122, 46613};         // baby
hand.set_command(actuator_position);
```

`ActuatorEffortCommand`의 target은 정격 전류의 0.1% 단위입니다. 부호가 방향을 나타내며 ±`max_effort`로
제한된 뒤 전송됩니다. `300`은 30%에 해당하는 예시입니다.

```cpp
ah2::ActuatorEffortCommand actuator_effort;
actuator_effort.target = {
    300, 300, 300, 300,           // thumb   a0 a1 a2 a3
    300, 300, 300,                // index   a1 a2 a3
    300, 300, 300,                // middle
    300, 300, 300,                // ring
    300, 300, 300};               // baby
hand.set_command(actuator_effort);
```

동일한 index의 joint와 actuator는 서로 대응하지 않습니다. 두 공간의 변환은
[6. Kinematics](#6-kinematics)에 있습니다.

### 5.2 Validation and clamp

`set_command()`는 state를 검사하고, 값을 검사하고,
joint command인 경우 target을 도달 범위로 투영한 뒤 command를 적용합니다. 앞의 두 검사는 command를
거부하며, 마지막 단계는 값을 수정하여 통과시킵니다.

#### 5.2.1 Rejected by state

state 조건은 `lifecycle` 값이 `Running`이면서 `homing_state` 값이 `Succeeded`라는 두 가지입니다.
만족하지 않으면 command가 적용되지 않고 `WrongCallOrder` 예외를 던집니다.
`Idle`만 `homing_state` 조건에서 제외되어 `Running`이면 통과합니다.

#### 5.2.2 Rejected by value

값 검사를 통과하지 못한 command는 적용되지 않고 **직전 command가 그대로 계속 전송됩니다.**
다음 두 조건 중 하나에 해당하는 경우입니다.

- target에 NaN이나 Inf가 있는 경우
- `ActuatorPositionCommand`의 target이 `int32` 범위를 벗어난 경우

값 검사 실패는 예외를 던지지 않습니다. `nan_command_count` 값이 1 늘어나고 warning log가 남습니다.
log는 잘못된 값이 이어지는 동안 한 번만 남고, 정상 값이 전달된 뒤 다시 잘못되면 새로 남습니다.
`nan_command_count` 필드는 [8. Diagnostics](#8-diagnostics)에서 읽습니다.

target을 계산한 직후 application이 finite 여부를 검사하기를 권장합니다.

#### 5.2.3 Workspace clamp

값 검사를 통과한 joint command는 target이 finger별 도달 범위로 투영된 뒤 적용됩니다. 전송 전에
투영 결과를 확인할 필요가 있을 때만 [`clamp()`](08_cpp_api_reference/types_command.md#jointpositioncommandclamp)를 직접 호출합니다.

```cpp
command.clamp();                    // 투영만 미리 확인합니다
```

범위 모델은 [Workspace limits](14_workspace_limits.md)에 있습니다.

> [!IMPORTANT]
> clamp는 도달 범위만 검사합니다. self-collision, 주변 물체, cable, payload는 검사하지 않습니다.
> 충돌·속도·힘 제한은 application이 따로 걸어야 합니다.

### 5.3 Command lifetime

command는 다음 `set_command()`까지 유지되지만, `Running`을 벗어나면 무효가 됩니다.
`set_command()`를 받는 state가 `Running`뿐이기 때문입니다. 다시 `Running`이 될 때는 아래 표의
command에서 시작하므로, 필요한 자세는 `Running`으로 복귀한 뒤 다시 전송해야 합니다.

| Path back to `Running` | First command applied |
|---|---|
| `Stopped`에서 [`run()`](08_cpp_api_reference/hand.md#handrun) | 재개 시점의 자세를 유지하는 command |
| homing 성공 | target이 전부 `0`인 `ActuatorPositionCommand` |
| 나머지 경로 | `Idle` |

앞의 두 경로는 자세가 갑자기 무너지지 않도록 SDK가 command를 채우는 경우입니다. 나머지는 모두
`Idle`이므로 actuator에 토크가 걸리지 않습니다.

> [!TIP]
> 이 장의 내용은 다음 예제로 확인할 수 있습니다. command 종류마다 파일이 하나씩입니다.
>
> - [`08_idle.cpp`](../../cpp/examples/08_idle.cpp) — `Idle`과 `stop()`을 차례로 실행해 측정 전류와
>   속도의 차이를 출력하고, `Running`을 벗어났다 복귀할 때 어느 command에서 시작하는지 확인합니다.
> - [`09_joint_position.cpp`](../../cpp/examples/09_joint_position.cpp) — 정상 target, 도달 범위를
>   벗어난 target, NaN이 섞인 target을 차례로 전송해 세 검사 중 예외를 던지는 것이 하나뿐임을
>   확인합니다.
> - [`10_joint_impedance.cpp`](../../cpp/examples/10_joint_impedance.cpp) — 같은 16개 값을 두 command
>   struct에 각각 대입해 전송하고, `controller_output` 필드에 담기는 값이 달라지는 것을 출력합니다.
> - [`11_actuator_position.cpp`](../../cpp/examples/11_actuator_position.cpp) — 측정 count에 offset을
>   더해 target을 만들고, setpoint가 첫 cycle에 그대로 반영되는 것과 `int32` 범위 검사를 확인합니다.
> - [`12_actuator_effort.cpp`](../../cpp/examples/12_actuator_effort.cpp) — 부호를 바꾸며 방향이
>   바뀌는 것을 출력하고, 상한보다 큰 값을 전송해 어느 값이 적용되는지 확인합니다.

## 6. Kinematics

kinematics 함수는 joint 공간과 actuator 공간을 변환하는 free function입니다. [`Hand`](08_cpp_api_reference/hand.md#hand) 없이도 호출할 수
있어, command를 전송하기 전에 target을 확인하거나 기록한 encoder 값을 나중에 joint 각도로 변환할
때 사용합니다.

### 6.1 Conversion functions

변환 함수는 forward kinematics와 inverse kinematics 두 개이며, 입력과 출력의 단위가 서로
반대입니다.

| Function | Input | Output |
|---|---|---|
| [`fk_actuator_to_joint()`](08_cpp_api_reference/hand_kinematics.md#fk_actuator_to_joint) | `std::array<int, kActuatorCount>`, encoder count | `std::array<double, kJointCount>`, rad |
| [`ik_joint_to_actuator()`](08_cpp_api_reference/hand_kinematics.md#ik_joint_to_actuator) | `std::array<double, kActiveJointCount>`, rad | `std::array<int, kActuatorCount>`, encoder count |

다음은 target을 actuator position으로 변환한 뒤 다시 joint 각도로 되돌리는 예입니다.

```cpp
const auto encoder = ah2::ik_joint_to_actuator(command.target);
const auto joints  = ah2::fk_actuator_to_joint(encoder);
```

`get_state().joints.position_rad`는 SDK가 `fk_actuator_to_joint()`로 채운 값입니다. 같은 encoder
값을 직접 전달하면 같은 결과를 얻습니다. `actuators.position_count` 필드는 `double`이므로 정수
count로 반올림해 전달하십시오.

### 6.2 Active and passive joints

active joint는 command로 각도를 지정하는 16개이고, passive joint는 지정 대상이 아닌 5개입니다.
inverse kinematics 입력은 active joint 16개이고 forward kinematics 출력은 passive를 포함한
21개이므로, 두 함수는 입력과 출력의 크기가 다릅니다. 배열 배치는 [API reference](08_cpp_api_reference/types_description.md#배열-규약)에
있습니다.

> [!IMPORTANT]
> forward kinematics 결과를 command의 `target` 필드에 그대로 대입할 수 없습니다. 크기가 21과 16으로 다르므로, passive
> joint를 제거하지 않고 전달하면 의도한 자세와 다른 command가 전송됩니다.

> [!TIP]
> 이 장의 내용은 다음 예제로 확인할 수 있습니다.
>
> - [`13_kinematics.cpp`](../../cpp/examples/13_kinematics.cpp) — target을 actuator로 변환한 뒤 다시
>   joint로 되돌려 왕복 오차를 출력합니다. 같은 비교를 앞 16개 항목으로 수행한 결과도 함께 출력하므로
>   두 index 공간의 차이가 수치로 나타납니다. 변환 부분은 로봇 핸드 없이도 실행됩니다.

## 7. HandState

[`HandState`](08_cpp_api_reference/types_state.md#handstate)는 로봇 핸드에서 읽은 값을 한 구조체에 모은 것이며, [`get_state()`](08_cpp_api_reference/hand.md#handget_state)가 반환합니다.
호출은 다음 한 줄입니다.

```cpp
const ah2::HandState state = hand.get_state();
```

| Field | Type | Description |
|---|---|---|
| `timestamp` | `std::int64_t` | 해당 cycle의 `CLOCK_REALTIME` epoch ns. `0`은 아직 값이 없다는 뜻 |
| `actuators` | [`ActuatorState`](08_cpp_api_reference/types_state.md#actuatorstate) | actuator 관측 |
| `joints` | [`JointState`](08_cpp_api_reference/types_state.md#jointstate) | forward kinematics 결과 |
| `tactile` | [`TactileState`](08_cpp_api_reference/types_state.md#tactilestate) | 촉각 |
| `commanded` | [`CommandedState`](08_cpp_api_reference/types_state.md#commandedstate) | 이번 cycle의 command 입력·출력 |

`get_state()`와 [`get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics)는 각각 최신
buffer를 개별적으로 읽습니다. 두 호출 사이에 cycle이 하나 경과할 수 있으므로, 양쪽 값을 비교할
때는 한 번 읽은 결과를 재사용하십시오.

SDK는 snapshot의 최신 여부를 boolean으로 제공하지 않습니다. snapshot을 읽은 monotonic 시각을
application이 함께 기록해야 경과 시간을 판단할 수 있습니다. `timestamp` 필드는 해당 cycle의
`CLOCK_REALTIME`이므로 rosbag과 시각을 맞추는 용도이며, 주기 측정에는 `last_period_ms` 필드를
사용하십시오.

### 7.1 Actuators and joints

`actuators` 필드는 drive가 보고한 값이고, `joints` 필드는 actuator 값을 forward kinematics로
변환한 결과입니다.

| Field | Type | Unit |
|---|---|---|
| `actuators.position_count` | `std::array<double, kActuatorCount>` | encoder count |
| `actuators.velocity_rpm` | `std::array<double, kActuatorCount>` | rpm |
| `actuators.current_mA` | `std::array<double, kActuatorCount>` | mA |
| `joints.position_rad` | `std::array<double, kJointCount>` | rad |
| `joints.velocity_rad_s` | `std::array<double, kJointCount>` | rad/s |
| `joints.effort_Nm` | `std::array<double, kJointCount>` | N·m |

`joints` 필드는 passive를 포함해 21개이고 `actuators` 필드는 16개입니다. 두 공간의 변환은
[6. Kinematics](#6-kinematics)에 있습니다.

> [!WARNING]
> `joints.velocity_rad_s`와 `joints.effort_Nm` 필드는 아직 `0` placeholder입니다. 측정값이 아니므로
> recorder schema에 측정값으로 표시하지 마십시오.

### 7.2 Applied command

`state.commanded` 필드는 한 cycle의 처리 과정을 단계별로 담습니다.

| Field | Description |
|---|---|
| `controller_input` | 현재 적용 중인 command |
| `controller_output` | limit과 fault mask를 통과한 actuator position 또는 effort setpoint |
| `selected_source` | 이번 cycle에 실제로 사용한 출처: `None`/`Controller`/`QuickStop`/`Homing` |
| `max_effort_pct` | 적용 중인 actuator별 상한 |

`controller_output` 필드는 담기는 값이 달라지는 variant입니다.

- [`JointPositionCommand`](08_cpp_api_reference/types_command.md#jointpositioncommand) · [`ActuatorPositionCommand`](08_cpp_api_reference/types_command.md#actuatorpositioncommand) → [`ActuatorPositionSetpoint`](08_cpp_api_reference/types_state.md#actuatorpositionsetpoint--actuatoreffortsetpoint)
- [`JointImpedanceCommand`](08_cpp_api_reference/types_command.md#jointimpedancecommand) · [`ActuatorEffortCommand`](08_cpp_api_reference/types_command.md#actuatoreffortcommand) · [`Idle`](08_cpp_api_reference/types_command.md#idle) → `ActuatorEffortSetpoint`
  (`Idle`은 전부 `0`)
- control session(`Running`·`Stopped`·`Faulted`) 밖 → `std::monostate`

다음은 position setpoint를 읽어 측정 위치와의 오차를 계산하는 예입니다.

```cpp
if (const auto* out =
        std::get_if<ah2::ActuatorPositionSetpoint>(&state.commanded.controller_output)) {
  const double error = out->target_position_cnt[0] - state.actuators.position_count[0];
}
```

추종을 확인할 때는 같은 공간끼리 비교하십시오. `target_position_cnt`와
`state.actuators.position_count` 필드가 짝입니다. rad `target` 값에서 encoder count를 빼면 의미가 없습니다.

effort 계열은 짝이 없습니다. `target_effort_pct` 필드는 정격 전류의 0.1% 단위이고 측정값
`actuators.current_mA` 필드는 mA라, 두 값을 직접 비교할 수 없습니다.

`controller_output` 필드에 값이 있다고 actuator가 해당 값을 그대로 적용했다는 뜻은 아닙니다.
`selected_source` 값이 `Controller`인지 함께 확인하십시오.

### 7.3 Tactile

`state.tactile` 필드는 finger와 palm의 taxel 값입니다. 센서가 전송한 16-bit 원시 count를
그대로 담은 값이라 단위도 정규화도 없습니다.

| Field | Size | Description |
|---|---|---|
| `tactile.fingers` | `kFingerCount` × `kTactileTaxelsPerFinger` (5 × 17) | finger index는 [`Finger`](08_cpp_api_reference/types_description.md#enum-finger) enum 순서 |
| `tactile.palm` | `kPalmTactileCount` (58) | palm1 upper 20 + lower 20 + palm2 18 |

접촉 판정 임계값은 SDK가 기준을 제시하지 않습니다. 접촉이 없는 상태를 baseline으로 설정하고
baseline과의 차이를 확인하십시오.

> [!TIP]
> 이 장의 내용은 다음 예제로 확인할 수 있습니다.
>
> - [`14_read_state.cpp`](../../cpp/examples/14_read_state.cpp) — 필드를 차례로 출력하며 placeholder
>   두 개가 `0`으로 읽히는 것, control session 밖에서 `controller_output` 필드가 `std::monostate`인
>   것, 두 호출이 각각 buffer를 개별적으로 읽는 것을 확인합니다.
> - [`15_tactile.cpp`](../../cpp/examples/15_tactile.cpp) — 접촉이 없는 상태를 1초간 평균해 baseline을
>   만들고, 이후 baseline과의 차이를 finger와 palm 영역별로 갱신해 출력합니다. 토크를 걸지 않습니다.

## 8. Diagnostics

[`Diagnostics`](08_cpp_api_reference/types_diagnostics.md#diagnostics)는 SDK와 제어·통신 루프의 상태를 한 구조체에 모은 것이며, [`get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics)가 반환합니다.
호출은 다음 한 줄입니다.

```cpp
const ah2::Diagnostics diag = hand.get_diagnostics();
```

정상 운전 중에는 Normal 열의 조건이 성립합니다. 값이 다르면 Check 열이 지시하는 절을
참고하십시오.

| Field | Normal | Check |
|---|---|---|
| `lifecycle` | `Running` | [9.4 Recovering from Faulted](#94-recovering-from-faulted) |
| `homing_state` | `Succeeded` | [3. Homing](#3-homing) |
| `nan_command_count` | 늘지 않음 | [5.2 Validation and clamp](#52-validation-and-clamp) |
| `actuator_health` | fault 없음 | [8.1 Actuator faults](#81-actuator-faults) |

제어·통신 루프의 상태는 다음 네 필드로 확인합니다. 개별 값보다 네 값의 조합으로 판정하므로
Normal 열을 두지 않았습니다.

| Field | Description |
|---|---|
| `control_cycles` | 제어·통신 루프 누적 cycle |
| `deadline_misses` | 계산 시간이 목표 주기를 넘긴 cycle 누적 |
| `last_period_ms` | cycle 시작 사이의 간격 |
| `last_compute_ms` | 마지막 cycle의 계산 시간(RX→FK→IK→TX) |

제어·통신 루프의 동작 여부는 `control_cycles`와 `state.timestamp` 필드를 함께 확인합니다. 양쪽이
함께 증가하면 정상이고, 모두 정지하면 `lifecycle` 값이 `Faulted`이거나 제어·통신 루프가 중단된
상태이며, `timestamp` 값이 `0`이면 아직 유효한 state가 없습니다. 두 값은 `Faulted`가 아닌 동안
frame 수신 여부와 무관하게 매 cycle 갱신되므로 RX 생존의 근거가 되지 않습니다. RX가 중단되면 약
100 ms 후 `lifecycle` 값이 `Faulted`로 전이하고, 전이 시점부터 양쪽이 정지하며 자동 재연결 중에도
정지 상태를 유지합니다.

주기 준수 여부는 `deadline_misses` 필드로 확인합니다. 500 Hz면 목표 주기가 2 ms이고, 계산 시간이 목표
주기를 넘긴 cycle이 누적됩니다. 순간적으로 급증한 값보다 일정 구간의 증가율을 확인하십시오.
증가율은 두 값의 증분으로 계산합니다.

```text
miss_rate = Δdeadline_misses / Δcontrol_cycles
```

임계값은 CPU·payload·controller와 허용 위험에 따라 달라지므로 SDK가 기준을 제시하지 않습니다.

### 8.1 Actuator faults

actuator fault는 `actuator_health` 필드의 `fault` 배열에 actuator마다 담깁니다. 다음은 fault가 발생한
actuator를 출력하는 예입니다.

```cpp
const auto diag = hand.get_diagnostics();

for (std::size_t i = 0; i < ah2::kActuatorCount; ++i) {
  const auto fault = diag.actuator_health.fault[i];
  if (fault != ah2::ActuatorFault::None) {
    std::cerr << "actuator " << i << ": " << ah2::to_string(fault) << '\n';
  }
}
```

fault가 발생한 actuator는 제어·통신 루프가 fault reset을 반복 시도합니다. fault가 있거나
enable되지 않은 actuator는 command 적용 대상에서 mask되어, position setpoint는 측정 위치로
대체되고 effort setpoint는 `0`이 됩니다. position setpoint를 `0`으로 떨어뜨리면 로봇 핸드가
급격히 움직이기 때문입니다.

다만 fault가 발생해도 SDK는 스스로 정지하지 않습니다. 대응은 application이 결정합니다. 값의 종류는
[`ActuatorFault`](08_cpp_api_reference/types_state.md#enum-actuatorfault) enum에, 값별 점검 항목은
[Error messages](15_error_messages.md#11-actuator-fault)에 있습니다.

`disabled_actuators` 필드에 지정한 index는 보고 자체가 이루어지지 않으므로, 지정한 목록을 함께 남겨야
"정상이어서 fault가 없는 actuator"와 "보고 대상이 아닌 actuator"를 구분할 수 있습니다.

### 8.2 Diagnostic record

진단 기록은 나중에 원인을 추적하기 위한 것이므로, 다음 항목을 같은 시각에 남기십시오.

- `state.timestamp` 필드와 읽은 시점의 monotonic 시각
- [`ActuatorState`](08_cpp_api_reference/types_state.md#actuatorstate)의 position·velocity·current
- [`JointState`](08_cpp_api_reference/types_state.md#jointstate)의 position
- `state.commanded` 필드 전체
- 함께 읽은 `Diagnostics` 전체
- 적용 중이던 [`ControllerConfig`](08_cpp_api_reference/types_config.md#controllerconfig) (getter가 없으므로 application이 보관합니다)

한 번의 호출로 얻은 `Diagnostics`에는 두 시점이 섞여 있으므로, 기록을 맞출 때는 cycle 번호가 아니라
`state.timestamp` 필드와 monotonic 시각을 사용하십시오.

`controller_output` 필드는 [`ControllerOutput`](08_cpp_api_reference/types_state.md#type-aliases) variant이라 setpoint 종류에 따라 담기는
값이 달라집니다. `std::monostate`면 control session 밖이라 setpoint 자체가 없다는 뜻입니다.
`std::monostate`를 `0`으로 기록하면 무토크 command와 구분되지 않습니다. `joints` 필드의 velocity와 effort가 placeholder인
점은 [7.1 Actuators and joints](#71-actuators-and-joints)를 참고하십시오.

> [!TIP]
> 이 장의 내용은 다음 예제로 확인할 수 있습니다.
>
> - [`16_diagnostics.cpp`](../../cpp/examples/16_diagnostics.cpp) — 진단 값만 읽어 출력하는
>   monitor입니다. 네 필드와 구간별 증가율, actuator fault를 갱신해 출력하고, 종료할 때 8.2의 기록
>   항목을 한 번에 출력합니다. 토크를 걸지 않으므로 다른 프로그램과 함께 실행할 수 있습니다.

## 9. Error handling

예외는 SDK가 실패를 알리는 유일한 수단입니다. 실패란 public 호출이 요청한 동작을 완료하지 못한
경우를 말합니다. 9장은 실패를 감지하는 방법과 감지 이후 제어를 재개하는 방법을 다룹니다.

다음 두 조건은 호출의 실패가 아니므로 예외가 아니라 진단으로 관측하며, 9장에서 다루지 않습니다.

| Condition | Observed by |
|---|---|
| actuator fault | `actuator_health`. lifecycle을 바꾸지 않습니다([8.1 Actuator faults](#81-actuator-faults)) |
| 값이 유효하지 않아 적용되지 않은 command | `nan_command_count` 값이 증가하고 warning log가 남습니다([5.2 Validation and clamp](#52-validation-and-clamp)) |

### 9.1 Exception

[`Exception`](08_cpp_api_reference/types_error.md#exception--stdruntime_error)은 `std::runtime_error`를 상속하고 [`ErrorCode`](08_cpp_api_reference/types_error.md#enum-errorcode) enum 값을 함께 담습니다.

SDK는 예외를 던지기 전에 `[exception] <ErrorCode>: <message>`를 error 레벨로 log에 남깁니다.
그래서 catch에서 같은 내용을 다시 출력할 필요가 없습니다.

다음은 예외에서 code와 사유를 읽는 예입니다.

```cpp
try {
  hand.run();
} catch (const ah2::Exception& error) {
  const ah2::ErrorCode code = error.code();   // 무엇이 잘못됐는지
  const char* reason        = error.what();   // 사람이 읽을 사유. code는 포함되지 않습니다

  // 두 값의 처리는 application이 결정합니다. 구조는 9.3 Recovery patterns에 있습니다
}
```

`what()`에 code가 포함되지 않으므로 code와 사유를 함께 전달할 때는 [`to_string()`](08_cpp_api_reference/types_error.md#enum-errorcode)으로 code를
문자열로 변환해 앞에 추가하십시오. 전달받은 값으로 제어를 재개하는 방법은
[9.3 Recovery patterns](#93-recovery-patterns)에 있고, 상위에 사유를 반환하는 예는 9.3.2에
있습니다.

`ErrorCode`는 점검 대상에 따라 세 묶음으로 나뉩니다.

| `ErrorCode` | Check | Retryable |
|---|---|---|
| `InvalidArgument` · `WrongCallOrder` | 호출한 코드 | 없습니다. 호출을 고쳐야 합니다 |
| `InterfaceUnavailable` · `CommunicationLost` · `HardwareFault` | 장비와 환경 | 원인이 해소되면 성공합니다 |
| `ControlLoopFault` · `UnexpectedError` | SDK에 보고 | 없습니다 |

code는 log에 기록하고 사람이 원인을 특정할 때 사용하는 값입니다. 복구 절차를 code로 분기하지
마십시오. 같은 code가 서로 다른 lifecycle에서 발생하고, 복구 방법은 lifecycle이 결정하기
때문입니다. 문구별 정리는
[Error messages](15_error_messages.md)에 있습니다.

### 9.2 Detecting a failure

실패를 감지하는 경로는 두 가지이고, 경로마다 감지하는 실패가 다릅니다.

| Path | Detects |
|---|---|
| 예외 | application이 호출한 함수가 실패한 경우 |
| [`get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics)의 `lifecycle` | 제어·통신 루프가 스스로 정지한 경우 |

`Faulted`로의 전이는 제어·통신 루프만 수행합니다. 로봇 핸드가 자세를 유지하는 동안 통신이
끊기면 다음 호출 전까지 예외가 발생하지 않으므로, `lifecycle` 필드를 주기적으로 읽어야 제때
감지합니다. [`get_state()`](08_cpp_api_reference/hand.md#handget_state)와 `get_diagnostics()`는 `Faulted`에서도 예외를 던지지 않으므로
감시에 사용할 수 있습니다.

반대로 예외가 발생했다고 `Faulted`가 된 것도 아닙니다. [`connect()`](08_cpp_api_reference/hand.md#handconnect)가 실패하면
`Disconnected`로 되돌아가고, [`run()`](08_cpp_api_reference/hand.md#handrun)·[`stop()`](08_cpp_api_reference/hand.md#handstop)·[`disconnect()`](08_cpp_api_reference/hand.md#handdisconnect)가 확인에 실패하면
lifecycle은 유지됩니다. 자세한 내용은 [2.3 Transition confirmation](#23-transition-confirmation)에 있습니다.

예외 없이 lifecycle이 바뀌는 경우도 있습니다. `auto_reconnect` 값이 `true`이면 복구하는 동안
`Faulted`에서 `Connected`를 지나 `Running`까지 자동으로 복귀하고, 다른 경로가 `stop()`을 호출하면
`Stopped`가 됩니다. 어느 쪽도 호출이 실패한 것이 아니므로 예외는 발생하지 않습니다. 그래서
`lifecycle` 필드를 매 주기 확인하는 9.3.1 구조가 예외 없는 lifecycle 변화까지 반영합니다.

### 9.3 Recovery patterns

복구를 어느 단계부터 수행할지는 현재 lifecycle이 결정합니다. 직전에 호출한 함수를 기억할 필요가
없습니다.

| Lifecycle | Resume from |
|---|---|
| `Faulted` | `reconnect()` ([9.4 Recovering from Faulted](#94-recovering-from-faulted)) |
| `Disconnected` | `connect()` |
| `Connected` · `Stopped` | `run()` |
| `Running` | 곧바로 command 전송 |

전이 호출은 모두 블로킹입니다. 대기하는 동안 제어·통신 루프는 계속 동작하고 actuator는 마지막
command를 유지하므로 위험하지는 않지만, 호출한 thread가 확인에 소요되는 시간만큼 정지합니다.
호출별 시간은
[2.2 State transition calls](#22-state-transition-calls)에 있습니다.

구조는 **lifecycle을 누가 결정하는가**로 갈립니다. command의 출처는 기준이 아닙니다. 어느
구조에서도 catch를 두는 지점은 한 곳이면 충분합니다.

| Pattern | Lifecycle owner | catch |
|---|---|---|
| [Self-managed](#931-self-managed-lifecycle) | 루프가 결정합니다. 항상 `Running`을 유지합니다 | 루프 안에 하나 |
| [Externally managed](#932-externally-managed-lifecycle) | 외부가 결정합니다. `run`·`stop`·`disconnect`를 지시받습니다 | 요청 처리 함수에 하나 |
| [Exit on failure](#933-exit-on-failure) | 결정하지 않습니다. 순서대로 호출하고 실패하면 종료합니다 | 전체를 감싸는 하나 |

config는 구조를 바꾸지 않습니다. `auto_reconnect`와 `auto_home` 값을 어떻게 설정하든 아래 세 구조는
그대로 사용하며, 예제마다 값을 다르게 설정한 것은 여러 경우를 함께 제시하기 위해서입니다. 두
설정이 복구 절차에 미치는 영향은 9.4에서 다룹니다.

세 예제에서 `ah2::` 접두어가 없는 이름은 application이 채울 자리이고, 필요한 항목은 각 예제
상단의 선언에 명시했습니다.

#### 9.3.1 Self-managed lifecycle

self-managed 구조는 매 주기 lifecycle을 확인해 미완료 단계를 수행한 뒤 command를 전송합니다. 첫
주기에는 `Disconnected`이므로 같은 switch가 초기 연결까지 처리합니다.

루프가 담당하는 것은 lifecycle을 `Running`으로 유지하는 일이고, command의 출처는 무관합니다.
`next_command()`는 코드에 고정한 궤적일 수도, 파일에서 읽은 기록일 수도, 상위 제어기가 전송한
목표일 수도 있습니다. 다만 상위에서 수신하는 경우라면 복구하는 동안 상위와의 통신이 중단된다는 점을
감안하십시오. 상위가 중단을 허용하지 못하면 9.3.2 구조가 적합합니다.

다음은 lifecycle 확인과 command 전송을 한 루프에 둔 예입니다.

```cpp
#include <aidin_hand2/aidin_hand2.hpp>

namespace ah2 = aidin_hand2;

// ------------------------------ Application ---------------------------------
// 목표를 만드는 방법과 주기를 맞추는 방법은 application마다 다릅니다

bool shutdown_requested()
{
  // 종료 신호를 확인해 돌려줍니다
  return false;
}

ah2::JointPositionCommand next_command()
{
  // 이번 주기에 보낼 목표를 만들어 돌려줍니다. 여기서는 고정 자세를 예로 들었습니다
  ah2::JointPositionCommand command;
  command.target = {
      0.20,  0.35,  0.10, 0.25,   // thumb   j0 j1 j2 j3
      0.08,  0.45,  0.30,         // index   j1 j2 j3
      0.04,  0.55,  0.40,         // middle
     -0.04,  0.65,  0.50,         // ring
     -0.08,  0.75,  0.60};        // baby
  return command;
}

void wait_next_period()
{
  // 다음 주기까지 기다립니다
}

// ------------------------------ Control loop --------------------------------

int main()
{
  try {
    ah2::HandConfig config{"can0", ah2::HandSide::Right};
    config.auto_home           = true;   // run()이 homing까지 처리합니다
    config.auto_reconnect      = true;   // 통신 오류는 제어·통신 루프가 복구합니다
    config.auto_reconnect_home = true;

    // manager가 scope를 벗어나면 소유한 hand를 파기합니다. 파기는 quick stop을 확인하고
    // 연결을 끊으므로 stop()·disconnect()를 따로 부르지 않아도 됩니다
    ah2::HandManager manager;
    ah2::Hand hand = manager.create(config);
    hand.set_max_effort(1000.0);         // lifecycle과 무관하므로 여기서 설정합니다

    while (!shutdown_requested()) {
      try {
        switch (hand.get_diagnostics().lifecycle) {
          case ah2::HandLifecycle::Faulted:      hand.reconnect();  [[fallthrough]];
          case ah2::HandLifecycle::Disconnected: hand.connect();    [[fallthrough]];
          case ah2::HandLifecycle::Connected:
          case ah2::HandLifecycle::Stopped:      hand.run();        break;
          case ah2::HandLifecycle::Running:      break;
        }
        hand.set_command(next_command());
      } catch (const ah2::Exception&) {
        // 사유는 SDK가 이미 log에 남겼습니다. 이 주기를 건너뛰면
        // 다음 주기의 switch가 남은 단계를 수행합니다
      }
      wait_next_period();
    }
  } catch (const ah2::Exception&) {
    // 시작하지 못했습니다. config가 거부됐거나 hand를 만들지 못한 경우입니다
    return 1;
  }
  return 0;
}
```

try가 둘이지만 역할이 다릅니다. 바깥은 시작하지 못한 경우라 종료하고, 안쪽은 운전 중 실패라
다음 주기로 이어갑니다.

switch를 catch가 아니라 try 안에 배치한 이유는 셋입니다. `reconnect()` 같은 복구 호출도
실패할 수 있으므로 복구 실패까지 같은 catch가 잡아야 하고, 첫 주기가 `Disconnected`라 초기
연결까지 같은 코드가 처리하며, 예외 없이 lifecycle이 바뀌는
상황([9.2](#92-detecting-a-failure))도 반영하기 때문입니다. `get_diagnostics()`는 lock-free
buffer를 읽을 뿐이고 `Running`이면 switch가 곧바로 끝나므로 매 주기 확인해도 부담이 없습니다.

#### 9.3.2 Externally managed lifecycle

externally managed 구조는 외부가 `run`·`stop`·`disconnect`까지 지시합니다. 루프가 스스로 `run()`을
호출하면 사용자가 내린 `stop()` 지시를 무효화하게 되므로, 9.3.1의 switch를 사용하지 마십시오.

다음은 외부 요청을 받아 lifecycle을 전이하는 예이며, 복구까지 사용자가 지시하도록
`auto_reconnect` 값을 껐습니다. 통신 오류만 SDK가 처리하도록 하려면 켜 두어도 되고, 켜 두면 사용자가
`Reconnect`를 지시할 일이 줄어듭니다.

```cpp
#include <optional>
#include <string>
#include <aidin_hand2/aidin_hand2.hpp>

namespace ah2 = aidin_hand2;

// ------------------------------ Application ---------------------------------
// 명령을 받고 결과를 돌려주는 방법은 통신 방식에 따라 다릅니다

enum class Kind { Connect, Disconnect, Run, Stop, Home, Reconnect, Command };

struct Request {
  Kind kind;
  ah2::JointPositionCommand command;   // kind가 Command일 때만 씁니다
};

bool shutdown_requested()
{
  // 종료 신호를 확인해 돌려줍니다
  return false;
}

std::optional<Request> poll_request()
{
  // 외부에서 들어온 명령을 하나 꺼냅니다. 없으면 nullopt를 돌려줍니다
  return std::nullopt;
}

void send_state(ah2::HandLifecycle lifecycle, ah2::HomingState homing)
{
  // 현재 상태를 client에 내보냅니다
}

void send_response(const Request& request, bool ok, const std::string& reason)
{
  // 명령 처리 결과를 돌려줍니다. 실패하면 reason에 사유가 들어 있습니다
}

void wait_next_period()
{
  // 다음 주기까지 기다립니다
}

// ------------------------------ Command loop --------------------------------

int main()
{
  try {
    ah2::HandConfig config{"can0", ah2::HandSide::Right};
    config.auto_home      = false;   // homing 시점을 사용자가 지시하게 둡니다
    config.auto_reconnect = false;   // 복구도 사용자가 지시하게 둡니다

    // manager가 scope를 벗어나면 소유한 hand를 파기하며 quick stop까지 확인합니다
    ah2::HandManager manager;
    ah2::Hand hand = manager.create(config);

    while (!shutdown_requested()) {
      // 명령이 없는 동안에도 Faulted가 될 수 있으므로 상태는 계속 내보냅니다.
      // get_diagnostics()는 Faulted에서도 예외를 던지지 않습니다
      const ah2::Diagnostics diag = hand.get_diagnostics();
      send_state(diag.lifecycle, diag.homing_state);

      if (auto request = poll_request(); request) {
        try {
          switch (request->kind) {
            case Kind::Connect:    hand.connect();                    break;
            case Kind::Disconnect: hand.disconnect();                 break;
            case Kind::Run:        hand.run();                        break;
            case Kind::Stop:       hand.stop();                       break;
            case Kind::Home:       hand.home();                       break;
            case Kind::Reconnect:  hand.reconnect();                  break;
            case Kind::Command:    hand.set_command(request->command); break;
          }
          send_response(*request, true, "");
        } catch (const ah2::Exception& error) {
          // 재시도하지 않고 사유를 돌려줍니다. 다시 시도할지는 사용자가 정합니다
          send_response(*request, false,
                        std::string(ah2::to_string(error.code())) + ": " + error.what());
        }
      }
      wait_next_period();
    }
  } catch (const ah2::Exception&) {
    return 1;   // 사유는 SDK가 log에 남겼습니다
  }
  return 0;
}
```

#### 9.3.3 Exit on failure

exit on failure 구조는 한 번 실행하고 종료하는 프로그램에서 전체를 하나의 try로 감쌉니다. 복구할
이유가 없으므로 실패하면 그대로 종료합니다. homing만 수행하는 캘리브레이션, 상태를 한 번 읽는
진단 도구, 정해진 자세를 재생하는 테스트가 해당합니다. 실패하면 사람이 원인을 조치하고 다시
실행합니다.

다음은 logging 설정부터 종료까지 순서대로 호출하는 예입니다. AIDIN Hand Gen2가 실제로 움직이므로
target과 effort 상한은 환경에 맞게 변경하십시오. log 설정은 [Logging](09_cpp_logging.md)에
있습니다.

```cpp
#include <chrono>
#include <thread>
#include <aidin_hand2/aidin_hand2.hpp>

namespace ah2 = aidin_hand2;

int main()
{
  ah2::set_log_level(ah2::LogLevel::Info);
  ah2::set_log_to_console(true);
  ah2::set_log_to_file("/var/log/my_robot/aidin-hand2.log");   // create()보다 먼저 정합니다

  // main을 벗어날 때 소유한 hand를 파기하며 quick stop까지 확인하므로,
  // 예외로 빠져나온 경로에서도 actuator가 토크를 낸 채 남지 않습니다
  ah2::HandManager manager;
  int exit_code = 0;

  try {
    ah2::HandConfig config{"can0", ah2::HandSide::Right};
    config.auto_home = true;

    ah2::Hand hand = manager.create(config);
    hand.connect();
    hand.set_max_effort(1000.0);
    hand.run();                          // homing까지 여기서 끝납니다

    ah2::JointPositionCommand command;
    command.target = {
        0.20,  0.35,  0.10, 0.25,   // thumb   j0 j1 j2 j3
        0.08,  0.45,  0.30,         // index   j1 j2 j3
        0.04,  0.55,  0.40,         // middle
       -0.04,  0.65,  0.50,         // ring
       -0.08,  0.75,  0.60};        // baby
    hand.set_command(command);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  } catch (const ah2::Exception&) {
    exit_code = 1;   // 사유는 SDK가 log에 남겼습니다
  }

  ah2::flush_log();
  return exit_code;
}
```

### 9.4 Recovering from Faulted

`Faulted`에서 벗어나는 public 수단은 [`reconnect()`](08_cpp_api_reference/hand.md#handreconnect) 하나입니다. 물리적 원인을 제거한 뒤에도 계속
실패하면 해당 hand를 파기하고 새로 생성하는 방법이 있습니다. [`destroy()`](08_cpp_api_reference/hand_manager.md#handmanagerdestroy)는 어느 상태에서든
호출할 수 있으므로 [`create()`](08_cpp_api_reference/hand_manager.md#handmanagercreate)로 재시작합니다.

복구를 누가 수행하는지는 `auto_reconnect` 값이 결정합니다.

| `auto_reconnect` | Recovery by | Application |
|---|---|---|
| `false` (기본) | application | `Faulted`를 확인하고 `reconnect()`를 호출합니다 |
| `true` | 제어·통신 루프 | 통신 오류라면 대기합니다. 제어·통신 루프 예외라면 직접 호출해야 합니다 |

#### 9.4.1 Manual recovery

수동 복구는 `Faulted`를 확인한 application이 `reconnect()`부터 순서대로 호출하는 절차입니다.

```cpp
hand.reconnect();   // 성공하면 Connected
hand.run();         // auto_home=true이면 homing까지 여기서 끝납니다
```

`reconnect()`가 되돌리는 것은 `homing_state` 필드와 직전 command입니다([5.3 Command lifetime](#53-command-lifetime)).
`max_effort` 값과 [`ControllerConfig`](08_cpp_api_reference/types_config.md#controllerconfig) 설정은 그대로 유지됩니다. 이후 절차는
`auto_home` 값에 따라 갈립니다.

- `auto_home=true`(기본) → 이어지는 `run()`이 homing부터 수행합니다.
  [`home()`](08_cpp_api_reference/hand.md#handhome)을 따로 호출할 필요가 없습니다.
- `auto_home=false` → `run()`이 곧바로 `Running`이 되지만 command가 [`Idle`](08_cpp_api_reference/types_command.md#idle)이라
  actuator가 무토크 상태로 남습니다. `home()`이 성공하기 전에는 `Idle`이 아닌 command가 거부되므로
  `home()`을 호출하십시오.

#### 9.4.2 Automatic recovery

`auto_reconnect=true`이면 제어·통신 루프가 스스로 복구합니다. 다만 정지한 원인에 따라 갈립니다.

- 통신 오류 → 연결이 복구될 때까지 재시도합니다
- 제어·통신 루프 예외 → 동작하지 않습니다. `reconnect()`를 직접 호출하십시오

둘 중 어느 쪽인지는 예외의 [`code()`](08_cpp_api_reference/types_error.md#exceptioncode)로 갈립니다. `CommunicationLost`면 통신 오류이고
`ControlLoopFault`면 제어·통신 루프 예외입니다. 문구별 구분은
[Error messages](15_error_messages.md#10-faulted-원인-문구)에 있습니다.

자동 복구는 [9.4.1](#941-manual-recovery)의 수동 절차와 **동작이 다릅니다.** 제어를 스스로 다시
요구하므로, `run()`을 따로 호출하지 않아도 actuator enable이 확인되는 시점에 `Running`으로
복귀합니다.
`auto_reconnect_home=true`이면 재개 전에 homing도 수행합니다.

`homing_state` 값은 `auto_reconnect_home` 값과 무관하게 `NotRun`으로 돌아갑니다. 그래서
`auto_reconnect_home=false`로 두면 복구 뒤 `Idle`이 아닌
[`set_command()`](08_cpp_api_reference/hand.md#handset_command)가 `WrongCallOrder`
예외로 거부되므로, application이 `home()`을 직접 호출해야 합니다.

`auto_reconnect_timeout_ms` 값이 `0`이면 제한이 없으므로, 오래 복구 중인 상태를 정상 운전으로
오해하지 않도록 `lifecycle`과 `control_cycles` 필드를 따로 감시하십시오. 시한을 넘기면 자동 복구를 포기하고
`Faulted`로 남으므로 `reconnect()`를 직접 호출해야 합니다.

> [!TIP]
> 이 장의 내용은 다음 예제로 확인할 수 있습니다. 뒤의 세 항목이 9.3의 세 구조에 각각 대응합니다.
>
> - [`17_fault_recovery.cpp`](../../cpp/examples/17_fault_recovery.cpp) — 예외를 처리하며 `code()`로
>   점검 대상을 구분하고, `Faulted`에서 복구해 command를 다시 전송합니다. 복구 시도 횟수에 상한을
>   둡니다.
> - [`18_self_managed.cpp`](../../cpp/examples/18_self_managed.cpp) — 9.3.1의 구조입니다. 매 cycle
>   lifecycle을 읽어 빠진 단계를 채우므로 첫 cycle이 초기 연결을 수행합니다.
> - [`19_external_server.cpp`](../../cpp/examples/19_external_server.cpp) ·
>   [`19_external_client.cpp`](../../cpp/examples/19_external_client.cpp) — 9.3.2의 구조입니다. server가
>   로봇 핸드를 소유하고 client는 요청만 보냅니다. server를 먼저 실행하십시오.
> - [`20_exit_on_failure.cpp`](../../cpp/examples/20_exit_on_failure.cpp) — 9.3.3의 구조입니다. 첫
>   실패에서 종료하며, 소멸자가 실패 경로에서도 동작하도록
>   [`HandManager`](08_cpp_api_reference/hand_manager.md#handmanager)를 `try` 밖에 선언합니다.
