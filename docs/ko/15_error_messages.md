# Error messages and remedies

SDK가 예외와 log에 붙이는 오류 번호(`E401` 등)로 원인과 조치를 찾는 문서입니다. 호출 순서와 배경은
[C++ guide](07_cpp_usage_guide.md), 심볼과 필드는 [API reference](08_cpp_api_reference.md)에 있습니다.
log를 남기고 전달하는 방법은 [Logging](09_cpp_logging.md)을 보십시오.

## Contents

&nbsp;&nbsp;[**1. Reading a message**](#1-reading-a-message)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.1 Message format](#11-message-format)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.2 Code ranges](#12-code-ranges)<br>
&nbsp;&nbsp;[**2. E1xx `InvalidArgument`**](#2-e1xx-invalidargument)<br>
&nbsp;&nbsp;[**3. E2xx `WrongCallOrder`**](#3-e2xx-wrongcallorder)<br>
&nbsp;&nbsp;[**4. E3xx `InterfaceUnavailable`**](#4-e3xx-interfaceunavailable)<br>
&nbsp;&nbsp;[**5. E4xx `CommunicationLost`**](#5-e4xx-communicationlost)<br>
&nbsp;&nbsp;[**6. E5xx `HardwareFault`**](#6-e5xx-hardwarefault)<br>
&nbsp;&nbsp;[**7. E6xx `HandFaulted`**](#7-e6xx-handfaulted)<br>
&nbsp;&nbsp;[**8. E7xx `ControlLoopFault`**](#8-e7xx-controlloopfault)<br>
&nbsp;&nbsp;[**9. E9xx `UnexpectedError`**](#9-e9xx-unexpectederror)<br>
&nbsp;&nbsp;[**10. Actuator fault values**](#10-actuator-fault-values)

## 1. Reading a message

### 1.1 Message format

[`Exception`](08_cpp_api_reference/types_error.md#exception--stdruntime_error)의 `what()`과 log의
오류 줄은 같은 모양입니다.

```text
[<side>] [E<code>] cannot <action>: <reason> (<detail>)    예외의 what()
[<side>] [E<code>] <event> (<detail>)                      log에만 남는 사건
```

아래 번호별 message는 `[<side>]`를 빼고 번호부터 적습니다. 번호마다 원인·상태·조치를 적고, 함수마다
다르면 "`stop()`이 실패한 경우"처럼 나눠 적습니다. `destroy()`의 E403·E503은 예외 대신 같은 message를
log에 남깁니다.

### 1.2 Code ranges

| [`ErrorKind`](08_cpp_api_reference/types_error.md#enum-errorkind) | What it means | Code |
|---|---|---|
| `InvalidArgument` | 인자나 설정 값이 유효하지 않음 | E1xx |
| `WrongCallOrder` | 현재 lifecycle에서 허용되지 않는 호출 | E2xx |
| `InterfaceUnavailable` | CAN interface를 열지 못함 | E3xx |
| `CommunicationLost` | CAN 통신 실패(수신 끊김·송신 실패) | E4xx |
| `HardwareFault` | drive가 요청한 상태에 도달하지 못하거나 fault를 보고함 | E5xx |
| `HandFaulted` | lifecycle이 `Faulted`라 호출이 거부됨 | E6xx |
| `ControlLoopFault` | SDK thread를 시작하지 못하거나 thread가 예외로 종료됨 | E7xx |
| `UnexpectedError` | 예상하지 못한 예외나 내부 상태 오류 | E9xx |

번호별 조치로 해결되지 않거나 같은 오류가 반복되면 SDK log와 application이 남긴 `what()`을 함께
전달해 문의해 주십시오. SDK 내부 오류라고 적힌 번호는 바로 문의해 주십시오. 보고 채널은
[SDK build & install](06_sdk_build_and_install.md)에 있는 issue 트래커입니다.

## 2. E1xx `InvalidArgument`

이 범위의 예외를 던진 호출은 아무것도 바꾸지 않습니다. 값을 고쳐 다시 호출하십시오. E101~E103은
[`HandConfig`](08_cpp_api_reference/types_config.md#handconfig) 생성자가 던지며 log에는 남지 않습니다.
값이 잘못된 command는 예외를 던지지 않고 적용되지 않으며, 직전 command가 계속 전송됩니다
([C++ guide](07_cpp_usage_guide.md#522-rejected-by-value)).

```text
[E101] cannot create HandConfig: interface_name must not be empty
```

- 원인: 생성자의 첫 인자가 빈 문자열

```text
[E102] cannot create HandConfig: hand_side must be HandSide::Left or HandSide::Right
```

- 원인: `hand_side` 값이 enum 범위 밖

```text
[E103] cannot create HandConfig: hand_model must be HandModel::Gen2 or HandModel::Gen2B
```

- 원인: `hand_model` 값이 enum 범위 밖
- 조치: 문자열에서 바꿀 때는 [`parse_hand_model()`](08_cpp_api_reference/types_description.md#parse_hand_model)을 씁니다

```text
[E104] cannot set max effort: limit contains NaN/Inf
```

- 원인: [`set_max_effort()`](08_cpp_api_reference/hand.md#handset_max_effort) 인자에 NaN이나 Inf가 있음. 범위 밖의 값은 거부하지 않고 `[0, 2000]`으로 자릅니다

```text
[E105] cannot set controller config: joint position cutoff_freq and deadband must be finite and >= 0
```

- 원인: 두 값 중 하나가 non-finite이거나 음수

```text
[E106] cannot set controller config: joint impedance stiffness must be finite and in [0, 1]
```

- 원인: `stiffness` 배열에 non-finite이거나 `[0, 1]` 밖의 값이 있음

```text
[E107] cannot set controller config: joint impedance cutoff_freq must be finite and >= 0
```

- 원인: `cutoff_freq` 값이 non-finite이거나 음수

## 3. E2xx `WrongCallOrder`

현재 상태에서 할 수 없는 호출입니다. E203을 뺀 나머지는 호출이 아무것도 바꾸지 않습니다. 현재
lifecycle은 [`get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics)`.lifecycle` 필드로
확인하고, 상태별로 허용되는 호출은 [C++ guide](07_cpp_usage_guide.md#22-state-transition-calls)의 표를
보십시오.

```text
[E201] cannot connect hand: already connected
```

- 원인: `Running`·`Stopped`에서 [`connect()`](08_cpp_api_reference/hand.md#handconnect)를 호출함. `Connected`에서는 아무것도 하지 않고 성공합니다
- 조치: 다시 연결하려면 [`disconnect()`](08_cpp_api_reference/hand.md#handdisconnect) 뒤에 `connect()`를 호출합니다

```text
[E202] cannot <action>: not connected
```

- 원인: `Disconnected`에서 [`run()`](08_cpp_api_reference/hand.md#handrun)·[`stop()`](08_cpp_api_reference/hand.md#handstop)·[`home()`](08_cpp_api_reference/hand.md#handhome)·[`reconnect()`](08_cpp_api_reference/hand.md#handreconnect)·[`set_command()`](08_cpp_api_reference/hand.md#handset_command)·[`set_tactile_bias()`](08_cpp_api_reference/hand.md#handset_tactile_bias)·[`reset_tactile_bias()`](08_cpp_api_reference/hand.md#handreset_tactile_bias)를 호출함
- 조치: `connect()`를 먼저 호출합니다. `reconnect()`는 `Faulted`에서 복구할 때만 씁니다

```text
[E203] cannot home hand: interrupted during homing
```

- 원인: `home()`이 homing을 기다리는 동안 다른 thread가 `stop()`이나 `disconnect()`를 호출함. `auto_home` 값이 `true`이면 `run()`도 이 message를 던집니다
- 조치: 제어를 이어 가려면 `home()`을 다시 호출합니다

```text
[E204] cannot <action>: not running
```

- 원인: `Connected`에서 `stop()`을 호출했거나, `Connected`·`Stopped`에서 또는 enable이 확인되기 전에 `set_command()`를 호출함
- `stop()`이 거부된 경우: 멈출 것이 없으므로 그대로 둡니다
- `set_command()`가 거부된 경우: `run()`이 성공한 뒤에 다시 보냅니다

```text
[E205] cannot set command: not homed
```

- 원인: homing이 성공하기 전에 [`Idle`](08_cpp_api_reference/types_command.md#idle)이 아닌 command를 보냄
- 조치: `home()`이 성공한 뒤에 다시 보냅니다

```text
[E206] cannot reconnect hand: not faulted
```

- 원인: `Connected`·`Running`·`Stopped`에서 `reconnect()`를 호출함
- 조치: `reconnect()`는 `Faulted`에서만 호출합니다. 다시 연결하려면 `disconnect()` 뒤에 `connect()`를 호출합니다

```text
[E207] cannot use hand: hand is destroyed
```

- 원인: 파기한 hand를 호출함. [`destroy()`](08_cpp_api_reference/hand_manager.md#handmanagerdestroy)·`destroy_all()`이 이미 끝났거나 다른 thread에서 진행 중인 경우, 또는 manager가 handle보다 먼저 소멸한 경우입니다
- 조치: 파기한 handle은 쓰지 않습니다. 필요하면 [`create()`](08_cpp_api_reference/hand_manager.md#handmanagercreate)로 새로 만들고, manager를 handle보다 나중에 소멸시킵니다

## 4. E3xx `InterfaceUnavailable`

`connect()`나 `reconnect()`가 CAN interface를 열지 못했을 때 나옵니다. `connect()`에서 실패하면
`lifecycle` 값은 `Disconnected`로, `reconnect()`에서 실패하면 `Faulted`로 남습니다. 원인을 해결한 뒤
실패한 함수를 다시 호출하십시오.

```text
[E301] cannot connect hand: CAN interface '<name>' not found
```

- 원인: 그런 이름의 network interface가 없음. 이름이 틀렸거나 CAN adapter가 연결되어 있지 않은 경우입니다
- 조치: `ip link`로 이름을 확인하고 [`HandConfig`](08_cpp_api_reference/types_config.md#handconfig)의 `interface_name`과 맞춥니다

```text
[E302] cannot connect hand: permission denied opening CAN socket on '<name>'
```

- 원인: OS가 CAN socket을 열지 못하게 막음. CAN socket을 여는 데에는 root 권한이 필요하지 않으므로, process를 실행하는 환경(container, AppArmor·SELinux 같은 보안 정책)이 막는 경우입니다
- 조치: 실행 환경이 CAN socket을 허용하는지 확인합니다

```text
[E303] cannot connect hand: CAN not supported by the kernel
```

- 원인: kernel의 CAN module(`can`, `can_raw`)을 load하지 못함
- 조치: `sudo modprobe can can_raw`를 실행합니다. 직접 빌드한 kernel이면 [1.5 Configure the kernel](04_real_time_kernel_setup.md#15-configure-the-kernel)의 `CONFIG_CAN`·`CONFIG_CAN_RAW`를 확인합니다

```text
[E304] cannot connect hand: kernel or interface '<name>' is not configured for CAN-FD (CAN_RAW_FD_FRAMES)
```

- 원인: kernel이 CAN socket의 CAN-FD 옵션(`CAN_RAW_FD_FRAMES`)을 받아들이지 않음
- 조치: [1.4 Verify the link](05_can_fd_setup.md#14-verify-the-link)의 명령으로 interface가 CAN-FD(`<FD>`)로 올라와 있는지 확인합니다

```text
[E305] cannot connect hand: CAN interface '<name>' is down
```

- 원인: interface가 DOWN 상태
- 조치: [1.3 Bring up that interface](05_can_fd_setup.md#13-bring-up-that-interface)의 절차로 interface를 올립니다

```text
[E306] cannot connect hand: CAN interface '<name>' could not be opened (<reason>)
```

- 원인: 위의 경우가 아닌 이유로 interface를 열지 못함. `<reason>`은 OS가 돌려준 오류 설명입니다
- 조치: [CAN-FD setup](05_can_fd_setup.md#1-bring-up-the-interface)의 절차로 interface를 다시 올려 봅니다

```text
[E307] cannot connect hand: another master is commanding this hand on <name>
```

- 원인: 다른 프로그램이 이 로봇 핸드에 command를 보내고 있음. 같은 컴퓨터의 프로그램일 수도 있고, 같은 CAN bus에 연결된 다른 컴퓨터일 수도 있습니다
- 조치: 다른 프로그램을 끝내고 실패한 함수를 다시 호출합니다

## 5. E4xx `CommunicationLost`

이 범위는 호스트와 로봇 핸드 사이의 통신이 끊겼다는 뜻입니다. 먼저 다음을 확인하십시오.

1. 로봇 핸드 전원이 켜져 있는지
2. CAN cable이 연결되어 있는지
3. [1.4 Verify the link](05_can_fd_setup.md#14-verify-the-link)의 명령으로 interface가 CAN-FD(`<FD>`)로 올라와 있는지. 아니면 [1.3 Bring up that interface](05_can_fd_setup.md#13-bring-up-that-interface)대로 다시 올림
4. `interface_name`과 `hand_side`가 실제로 연결된 로봇 핸드와 맞는지. SDK는 `hand_side`로 정한 쪽의 frame만 받습니다

통신을 복구한 뒤 번호별로 적힌 호출을 다시 하십시오.

> [!WARNING]
> 통신이 끊긴 동안에는 SDK가 quick stop을 보낼 수 없으므로 drive가 마지막 command를 그대로 유지할 수
> 있습니다.

```text
[E401] cannot connect hand: no data received from <name> within 300 ms
```

- 원인: `connect()`나 `reconnect()`가 300 ms 안에 이 로봇 핸드의 frame을 하나도 받지 못함. `reconnect()`도 같은 message를 던집니다
- `connect()`가 실패한 경우: `lifecycle` 값은 `Disconnected`로 남습니다. `connect()`를 다시 호출합니다
- `reconnect()`가 실패한 경우: `lifecycle` 값은 `Faulted`로 남습니다. `reconnect()`를 다시 호출합니다

```text
[E402] cannot <action>: CAN communication failed
```

- 원인: [`run()`](08_cpp_api_reference/hand.md#handrun)·[`home()`](08_cpp_api_reference/hand.md#handhome)·[`set_tactile_bias()`](08_cpp_api_reference/hand.md#handset_tactile_bias)를 호출한 시점이나 `run()`이 enable을 기다리는 동안 통신이 끊김
- `run()`이 enable을 기다리다 실패한 경우: SDK가 quick stop을 요청합니다
- `set_tactile_bias()`가 실패한 경우: bias는 바뀌지 않습니다
- 조치: `lifecycle` 값이 `Faulted`가 되면 `reconnect()`를, 아니면 실패한 함수를 다시 호출합니다

```text
[E403] cannot <action>: quick stop not confirmed (CAN communication failed)
```

- 원인: quick stop을 요청했지만 송신이 실패해 500 ms 안에 확인하지 못함
- `stop()`이 실패한 경우: `lifecycle` 값은 바뀌지 않습니다. `stop()`을 다시 호출하고, 통신을 바로 복구할 수 없으면 로봇 핸드 전원을 내립니다
- `disconnect()`가 실패한 경우: 연결이 끊기지 않습니다. `disconnect()`를 다시 호출하거나, 그대로 끊어야 하면 `destroy()`를 호출합니다
- `destroy()`의 경우: 예외 없이 파기를 마치고, `cannot stop hand`로 시작하는 같은 message를 log에 남깁니다

```text
[E404] communication lost (no RX)
```

- 원인: 제어·통신 루프가 100 ms 동안 이 로봇 핸드의 frame을 받지 못해 `lifecycle` 값을 `Faulted`로 바꿈. log는 그 순간 한 번 남습니다
- 상태: `Faulted`이고 `fault_code` 필드 값은 `404`입니다. SDK는 이 뒤로 로봇 핸드에 아무것도 보내지 않고, 그 뒤 호출은 [E601](#7-e6xx-handfaulted)로 거부됩니다
- 조치: 통신을 복구한 뒤 `reconnect()`를 호출합니다. `auto_reconnect` 값이 `true`이면 SDK가 스스로 재시도합니다

log 줄의 `no RX` 뒤에 `, last RX error: conflicting command`가 붙으면, 연결한 뒤 다른 프로그램이 같은
로봇 핸드에 command를 보낸 적이 있다는 뜻입니다. 이번 끊김의 원인이라는 보장은 없습니다. 다른 프로그램을
끝낸 뒤 `reconnect()`를 호출하십시오.

```text
[E405] CAN TX failed (errno <number>: <text>)
```

- 원인: `Running`에서 송신이 실패해 제어·통신 루프가 `lifecycle` 값을 `Faulted`로 바꿈. `(errno …)`는 OS가 돌려준 오류이고 없을 수도 있습니다
- 상태: `Faulted`이고 `fault_code` 필드 값은 `405`입니다. SDK는 이 뒤로 로봇 핸드에 아무것도 보내지 않고, 그 뒤 호출은 [E601](#7-e6xx-handfaulted)로 거부됩니다
- 조치: 통신을 복구한 뒤 `reconnect()`를 호출합니다

```text
[E406] auto-reconnect timed out
```

- 원인: `auto_reconnect` 값이 `true`이고 `auto_reconnect_timeout_ms` 안에 통신이 복구되지 않음
- 상태: `Faulted`로 남고 자동 재시도를 멈춥니다
- 조치: 통신을 복구한 뒤 `reconnect()`를 직접 호출합니다

## 6. E5xx `HardwareFault`

통신은 정상이지만 drive가 요청을 끝내지 못했을 때 나옵니다. 먼저
[`get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics)의 `actuator_health.fault` 필드에서
fault가 보고된 actuator가 있는지 확인하고([10. Actuator fault values](#10-actuator-fault-values)), 번호별로
적힌 호출을 다시 하십시오.

```text
[E501] cannot run hand: drives did not reach operation enabled within 4000 ms
```

- 원인: fault가 없는 actuator가 4000 ms 안에 모두 enable되지 않았거나, 모든 actuator가 fault 상태
- 상태: SDK가 quick stop을 요청하고, quick stop이 확인되면 `Stopped`가 됩니다
- 조치: [`run()`](08_cpp_api_reference/hand.md#handrun)을 다시 호출합니다

```text
[E502] homing failed at step (<k>/3) <step>: <reason> (actuators <list>)
[E502] cannot home hand: homing failed (actuators <list>)
```

- 원인: homing이 한 단계를 끝내지 못함. 단계와 이유는 log 줄에 있습니다(아래 표). `<list>`는 실패한 actuator index를 쉼표로 이은 값입니다. 실패한 actuator가 없으면 예외에는 `none`으로 적히고, log 줄에서는 괄호까지 생략됩니다
- 상태: `homing_state` 값이 `Failed`가 됩니다. `auto_home` 값이 `true`이면 `run()`도 이 message를 던집니다
- 조치: [`home()`](08_cpp_api_reference/hand.md#handhome)을 다시 호출합니다

| Step | `<reason>` | Meaning |
|---|---|---|
| `(1/3) preparing actuators` | `no actuator could be brought under control` | 제한 시간 안에 어느 actuator도 enable되지 않았거나, 모든 actuator가 `disabled_actuators` 필드에 지정됨 |
| `(1/3) preparing actuators` | `some actuators could not be prepared for homing` | homing 준비가 제한 시간 안에 끝나지 않음 |
| `(3/3) setting origin` | `some actuators reported a homing error` | drive가 homing 오류를 보고함 |
| `(3/3) setting origin` | `some actuators did not finish homing` | homing을 시작한 뒤 제한 시간 안에 drive가 완료를 알리지 않음 |

enable 단계에서 일부 actuator만 응답하면, 응답하지 않은 actuator는 homing에서 제외되고 `<list>`에도
나오지 않습니다.

```text
[E503] cannot <action>: drives did not reach quick stop within 500 ms
```

- 원인: 통신은 정상이지만 drive가 500 ms 안에 quick stop 상태가 되지 않음
- `stop()`이 실패한 경우: `lifecycle` 값은 바뀌지 않습니다. `stop()`을 다시 호출하거나 로봇 핸드 전원을 내립니다
- `disconnect()`가 실패한 경우: 연결이 끊기지 않습니다. 로봇 핸드 전원을 내리거나, 그대로 끊어야 하면 `destroy()`를 호출합니다
- `destroy()`의 경우: 예외 없이 파기를 마치고, `cannot stop hand`로 시작하는 같은 message를 log에 남깁니다

> [!WARNING]
> quick stop을 확인하지 못한 채 끝나면 drive가 마지막 command를 그대로 유지할 수 있습니다.

```text
[E504] actuator <index> fault set (<name> 0x<code>)
```

- 원인: drive가 actuator fault를 보고함. `<name>`은 [`ActuatorFault`](08_cpp_api_reference/types_state.md#enum-actuatorfault) 값의 이름이고 `0x<code>`는 drive가 보낸 code입니다
- 상태: `lifecycle` 값은 바뀌지 않습니다. 그 actuator는 command 적용 대상에서 제외되고, 제어·통신 루프가 fault reset을 반복 시도합니다. fault가 풀리면 info level로 `actuator <index> fault cleared`가 남습니다
- 조치: 대응은 application이 결정합니다([C++ guide](07_cpp_usage_guide.md#81-actuator-faults))

## 7. E6xx `HandFaulted`

이 범위는 로봇 핸드가 `Faulted` 상태라서 호출을 거부했다는 뜻입니다. `Faulted`로 만든 원인은 그보다
먼저 log에 남은 줄과 [`Diagnostics`](08_cpp_api_reference/types_diagnostics.md#diagnostics)의
`fault_code` 필드에 있습니다.

```text
[E601] cannot <action>: hand faulted (E<code> <event>)
```

- 원인: `lifecycle` 값이 `Faulted`일 때 `reconnect()`가 아닌 함수를 호출함. 괄호 안은 `Faulted`로 만든 사건의 번호와, 그 사건이 log에 남긴 말입니다. 원인이 기록되지 않았으면 괄호 없이 `hand faulted`로 끝나며, 이는 SDK 내부 오류입니다
- 상태: `Faulted`로 남습니다
- 조치: 괄호 안 번호([E404](#5-e4xx-communicationlost), [E405](#5-e4xx-communicationlost), [E702](#8-e7xx-controlloopfault))의 조치를 따른 뒤 `reconnect()`를 호출합니다

## 8. E7xx `ControlLoopFault`

```text
[E701] cannot connect hand: failed to start SDK threads (<reason>)
```

- 원인: SDK의 thread를 만들지 못함. `<reason>`은 OS가 돌려준 오류입니다
- `connect()`가 실패한 경우: `lifecycle` 값은 `Disconnected`로 남습니다
- `reconnect()`가 실패한 경우: `lifecycle` 값은 `Faulted`로 남습니다
- 조치: process의 thread·메모리 한도를 확인한 뒤 실패한 함수를 다시 호출합니다

```text
[E702] control loop terminated, quick stop sent without confirmation (<reason>)
```

- 원인: 제어·통신 루프가 예외로 끝남. SDK 내부 오류입니다
- 상태: `Faulted`이고 `fault_code` 필드 값은 `702`입니다. SDK는 quick stop을 보내지만 확인하지는 못합니다. `auto_reconnect` 값이 `true`여도 자동으로 복구하지 않고, 그 뒤 호출은 [E601](#7-e6xx-handfaulted)로 거부됩니다
- 조치: `reconnect()`를 직접 호출합니다

```text
[E703] event logging thread terminated (<reason>)
```

- 원인: 제어·통신 루프의 사건을 log로 옮기는 thread가 예외로 끝남. SDK 내부 오류입니다
- 상태: 제어는 계속되지만, 그 뒤의 사건(통신 끊김, actuator fault, homing 실패 등)은 log에 남지 않습니다
- 조치: [`get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics)로 상태를 감시합니다

## 9. E9xx `UnexpectedError`

```text
[E901] cannot <action>: unexpected failure (<detail>)
```

- 원인: 분류되지 않은 예외. SDK 내부 오류입니다. `<action>`은 실패한 호출이고, `<detail>`은 예외의 내용입니다. 내용을 알 수 없으면 `unknown exception`으로 적힙니다

```text
[E902] cannot connect hand: CAN transport already open on '<name>'
```

- 원인: SDK 내부 상태가 예상과 다름. SDK 내부 오류입니다. 같은 hand의 `connect()`를 두 thread에서 동시에 호출해도 나옵니다
- 조치: `Hand`와 그 사본은 한 thread에서만 호출합니다

## 10. Actuator fault values

[`get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics)의 `actuator_health.fault` 필드는
drive가 보고한 actuator별 fault입니다. 예외를 던지지 않아도 값이 남으므로 이 값도 함께 기록하십시오.
값은 [`ActuatorFault`](08_cpp_api_reference/types_state.md#enum-actuatorfault) enum이고, `None`이면 해당
actuator에 fault가 없습니다. fault가 생기는 순간에는 [E504](#6-e5xx-hardwarefault) log도 남습니다. 배열
전체를 훑는 예제는 [C++ guide](07_cpp_usage_guide.md#81-actuator-faults)에 있습니다.

| Value | Code |
|---|---|
| `OverVoltageError` · `UnderVoltageError` | `0x3210` · `0x3220` |
| `OverCurrentError` · `OverLoadError` | `0x2310` · `0xFF02` |
| `OverTemperatureError` | `0x4210` |
| `CurrentDetectionError` | `0x5210` |
| `FollowingError` · `SpeedError` | `0x8611` · `0x7310` |
| `CommunicationError` | `0x7500` |
| `HallSensorError` | `0xFF01` |
| `PositiveLimitSwitchError` · `NegativeLimitSwitchError` | `0xFF03` · `0xFF04` |
| `EmergencySwitchError` · `Sto1Error` · `Sto2Error` | `0xFF05` · `0xFF06` · `0xFF07` |
| `SerialEncoderChannelAError` · `SerialEncoderChannelBError` | `0xFF24` · `0xFF26` |
| `SerialEncoderChannelADisconnectedError` · `SerialEncoderChannelBDisconnectedError` | `0xFF25` · `0xFF27` |
| 표에 없는 값 | drive가 보낸 code가 그대로 들어옴. `to_string()`은 `"Unknown"`을 돌려주고, E504 log에는 `UnknownError`로 적힘 |
