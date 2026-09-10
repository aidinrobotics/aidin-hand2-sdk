# Error messages and remedies

SDK가 던지는 예외의 message를 그대로 찾아 조치를 확인하는 문서입니다. 호출 순서와 배경은
[C++ guide](07_cpp_usage_guide.md), 심볼과 필드는 [API reference](08_cpp_api_reference.md)에
있습니다. 아래에서 log를 남기라고 하는 자리는 [Logging](09_cpp_logging.md)을 보십시오.

## Contents

&nbsp;&nbsp;[**1. 메시지 읽는 법**](#1-메시지-읽는-법)<br>
&nbsp;&nbsp;[**2. ErrorCode 요약**](#2-errorcode-요약)<br>
&nbsp;&nbsp;[**3. `InvalidArgument`**](#3-invalidargument)<br>
&nbsp;&nbsp;[**4. `WrongCallOrder`**](#4-wrongcallorder)<br>
&nbsp;&nbsp;[**5. `InterfaceUnavailable`**](#5-interfaceunavailable)<br>
&nbsp;&nbsp;[**6. `CommunicationLost`**](#6-communicationlost)<br>
&nbsp;&nbsp;[**7. `HardwareFault`**](#7-hardwarefault)<br>
&nbsp;&nbsp;[**8. `ControlLoopFault`**](#8-controlloopfault)<br>
&nbsp;&nbsp;[**9. `UnexpectedError`**](#9-unexpectederror)<br>
&nbsp;&nbsp;[**10. Faulted 원인 문구**](#10-faulted-원인-문구)<br>
&nbsp;&nbsp;[**11. Actuator fault**](#11-actuator-fault)

## 1. 메시지 읽는 법

[`Exception`](08_cpp_api_reference/types_error.md#exception--stdruntime_error)의 `what()`에는 message만 들어 있고
error code는 붙지 않습니다. message와 error code를 함께 남기십시오 —
예제는 [C++ guide의 9.1 Exception](07_cpp_usage_guide.md#91-exception)에 있습니다.

대부분의 message는 `Cannot <동작>: <이유>` 형태이고, 이 문서의 표는 `<이유>` 부분으로 찾습니다.
`<...>`는 실행 시 값이 채워지는 자리입니다.

`<이유>`가 `hand faulted (<원인>)`이면 `lifecycle` 값이 이미 `Faulted`입니다 — 원인 문구는
[10. Faulted 원인 문구](#10-faulted-원인-문구)에 있습니다. 이 상태에서는
[`reconnect()`](08_cpp_api_reference/hand.md#handreconnect) 전까지 lifecycle을 바꾸는 호출이 모두 거부되고,
[C++ guide](07_cpp_usage_guide.md#22-state-transition-calls)가 설정·조회로 분류한 호출은 그대로 허용됩니다.

멈춘 원인은 거부된 호출이 던지는 예외 message에 실립니다. log에도 남는지는 원인에 따라 갈립니다.

- 제어·통신 루프 예외 → log에도 바로 남습니다
- 통신 오류 → log에서 빠질 수 있으므로 예외 message를 근거로 삼으십시오

`Faulted`가 아닌 상태에서 거부된 경우는 code마다 조치가 달라, 아래 각 절 표의 Remedy 열을 그대로
따르십시오.

## 2. ErrorCode 요약

| [`ErrorCode`](08_cpp_api_reference/types_error.md#enum-errorcode) | What it means | Look at |
|---|---|---|
| `InvalidArgument` | 넘긴 값이 규칙에 맞지 않음 | 호출한 코드 |
| `WrongCallOrder` | 지금 상태에서 부를 수 없는 호출 | 호출한 코드(lifecycle) |
| `InterfaceUnavailable` | CAN interface를 열 수 없음 | interface 이름·권한·점유 |
| `CommunicationLost` | 통신이 끊김 | 전원·배선·bus 상태 |
| `HardwareFault` | 장비가 요청을 끝내지 못함 | drive fault·기구 |
| `ControlLoopFault` | 제어·통신 루프 비정상 종료 | SDK에 보고 |
| `UnexpectedError` | 분류되지 않은 실패 | SDK에 보고 |

보고 채널은 [SDK build & install](06_sdk_build_and_install.md)에 있는 issue 트래커입니다.

## 3. `InvalidArgument`

| Message | Cause | Remedy |
|---|---|---|
| `HandConfig: interface_name must not be empty` | 생성자 첫 인자가 빈 문자열 | CAN interface 이름을 넘깁니다 |
| `HandConfig: hand_side must be HandSide::Left or HandSide::Right` | `hand_side` 값이 범위 밖 | enum 값을 그대로 씁니다 |
| `Cannot create hand: control_rate must be positive` | `control_rate` 값이 `0` 이하 | 양수로 설정합니다(기본값 `500`) |
| `Cannot set max effort: limit contains NaN/Inf` | [`set_max_effort()`](08_cpp_api_reference/hand.md#handset_max_effort) 인자에 NaN·Inf | 값을 검사합니다(유효 범위 `[0, 2000]`) |
| `Cannot set controller config: joint position cutoff_freq and deadband must be finite and >= 0` | 두 값 중 하나가 non-finite이거나 음수 | 기존 설정이 유지됩니다. 값을 고쳐 다시 호출합니다 |
| `Cannot set controller config: joint impedance stiffness and damping must be finite and >= 0` | gain 배열에 non-finite이나 음수 | 위와 같습니다 |

[`set_command()`](08_cpp_api_reference/hand.md#handset_command)는 값이 잘못돼도 예외를 던지지 않습니다.
값 검사를 통과하지 못한 command는 적용되지 않고 **직전 command가 그대로 계속 전송됩니다.**
warning log가 남으므로 감지는
[`get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics)`.nan_command_count` 필드로 합니다 —
[C++ guide](07_cpp_usage_guide.md#522-rejected-by-value)를 보십시오.

## 4. `WrongCallOrder`

Situation 열의 상태 이름은 [`HandLifecycle`](08_cpp_api_reference/types_state.md#enum-handlifecycle) enum의 값이고, 상태 전이 함수가
어느 상태에서 허용되는지는 [C++ guide](07_cpp_usage_guide.md#22-state-transition-calls)에 표로 정리돼 있습니다.

아래 표의 Reason 열은 `Cannot <동작>: ` 뒤에 붙는 이유입니다. 현재 lifecycle은 [`get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics)`.lifecycle`로 확인합니다.

| Reason | Situation | Remedy |
|---|---|---|
| `already connected — call disconnect() first to rebuild the link` | `Running`·`Stopped`에서 [`connect()`](08_cpp_api_reference/hand.md#handconnect) | 통신을 다시 세우려면 [`disconnect()`](08_cpp_api_reference/hand.md#handdisconnect)를 호출한 뒤 `connect()`를 호출합니다 |
| `not connected — call connect() first` | `Disconnected`에서 [`run()`](08_cpp_api_reference/hand.md#handrun)·[`stop()`](08_cpp_api_reference/hand.md#handstop)·[`home()`](08_cpp_api_reference/hand.md#handhome) | `connect()`를 먼저 호출합니다 |
| `not connected yet — call connect()` | `Disconnected`에서 `reconnect()` | 최초 연결은 `connect()`를 호출합니다. `reconnect()`는 복구용입니다 |
| `not connected yet — call connect(), then run()` | `Disconnected`에서 [`set_command()`](08_cpp_api_reference/hand.md#handset_command) | 연결·실행 뒤 다시 보냅니다 |
| `control not running — call run() first` | `Connected`에서 `stop()` | 그대로 두십시오. 제어 중이 아니라 세울 것이 없습니다 |
| `not faulted — reconnect() recovers from Faulted only` | `Faulted`가 아닌데 `reconnect()` | `Faulted`에서만 복구용으로 호출하십시오. `Faulted`가 아닌 상태에서는 호출하지 마십시오 |
| `not running — call run() first` | `Connected`·`Stopped`에서 `set_command()`, 또는 enable이 확인되기 전 | [`run()`](08_cpp_api_reference/hand.md#handrun)이 성공한 뒤에 보냅니다 |
| `not homed — call home() first` | 원점 전에 [`Idle`](08_cpp_api_reference/types_command.md#idle)이 아닌 command | `home()` 성공 후 다시 보냅니다 |
| `hand is destroyed — create a hand` | 파기된 `HandCore`를 가리키는 handle | [`create()`](08_cpp_api_reference/hand_manager.md#handmanagercreate)로 새로 만듭니다 |
| `Hand is no longer valid (destroyed or manager gone)` | manager가 먼저 소멸 | manager 수명을 handle보다 길게 잡습니다 |
| `Cannot home hand: interrupted during homing — call run() and home() again` | homing 중 `Running`에서 벗어남 | 원인을 확인한 뒤 다시 수행합니다 |
| `CAN transport already open: <이름> — close it before reopening` | 같은 handle이 이미 열려 있음 | `disconnect()` 후 다시 연결합니다 |

`Faulted`에서 거부되면 `WrongCallOrder` 예외가 아니라 멈춘 원인을 담은 예외를 던집니다.
[10. Faulted 원인 문구](#10-faulted-원인-문구)를 보십시오.

## 5. `InterfaceUnavailable`

| Message | Cause | Remedy |
|---|---|---|
| `CAN interface '<이름>' not found (check the name with `ip link`)` | 그런 interface가 없음 | `ip link`로 이름을 대조하고 [`HandConfig`](08_cpp_api_reference/types_config.md#handconfig)와 맞춥니다 |
| `CAN interface '<이름>' is down and cannot be configured (needs CAP_NET_ADMIN). Bring it up manually: sudo ip link set <이름> type can bitrate 1000000 sample-point 0.875 sjw 10 …` | interface가 `DOWN`이고 SDK에 올릴 권한이 없음 | 메시지에 실린 명령을 그대로 실행하거나 [CAN-FD setup](05_can_fd_setup.md)의 절차를 따릅니다 |
| `CAN interface '<이름>' not found (check the name with `ip link`, and that it exists)` | bind 시점에 interface가 사라짐 | adapter 연결 상태를 확인합니다 |
| `permission denied opening CAN socket on '<이름>' (need root or CAP_NET_RAW)` | socket 생성 권한 부족 | `CAP_NET_RAW`를 주거나 root로 실행합니다 |
| `CAN not supported by the kernel (try `modprobe can can_raw`)` | kernel에 CAN module 없음 | `modprobe can can_raw`를 실행합니다 |
| `kernel or interface '<이름>' is not configured for CAN-FD (CAN_RAW_FD_FRAMES); check with `ip -details link show <이름>`` | CAN-FD 미지원 | 메시지의 명령으로 확인하고 CAN-FD를 지원하는 adapter를 씁니다 |
| `failed to open CAN interface '<이름>': <사유>` | 그 밖의 socket 열기 실패 | `<사유>`의 errno 설명을 확인합니다 |
| `failed to bring up CAN interface '<이름>': <사유>` | link up 실패 | adapter 연결과 driver를 확인합니다 |
| `failed to configure CAN interface '<이름>': <사유>` | bit timing 설정 실패 | CAN-FD 지원 여부와 timing 값을 확인합니다 |
| `failed to set CAN error mask on <이름>` | socket 옵션 설정 실패 | kernel의 SocketCAN 지원을 확인합니다 |
| `failed to set CAN RX filters on <이름>` | filter 설정 실패 | 위와 같습니다 |
| `another master is commanding this hand on <이름>` | 다른 process가 같은 hand를 제어 중 | 중복 실행을 정리하고 하나만 남깁니다 |

## 6. `CommunicationLost`

| Message | Cause | Remedy |
|---|---|---|
| `Cannot <동작>: no data received from <이름> within 300 ms — check hand power and CAN wiring` | [`connect()`](08_cpp_api_reference/hand.md#handconnect)가 첫 frame을 받지 못함 | 전원·배선·termination·bit rate를 확인한 뒤 `connect()`를 다시 호출합니다. `lifecycle` 값은 `Disconnected`로 남습니다 |
| `Cannot stop hand: cannot confirm quick stop — no RX on <이름>; drives may hold last command. Restore CAN link and hand power` | [`stop()`](08_cpp_api_reference/hand.md#handstop)이 quick stop을 확인하지 못함 | 통신과 전원을 복구하거나 전원을 내립니다. `lifecycle` 값은 `Running`으로 남습니다 |
| `Cannot disconnect hand: cannot confirm stop — no RX on <이름>; drives may hold last command. Use reconnect() or destroy` | [`disconnect()`](08_cpp_api_reference/hand.md#handdisconnect)가 quick stop을 확인하지 못함 | 위와 같습니다. [`reconnect()`](08_cpp_api_reference/hand.md#handreconnect) 또는 [`destroy()`](08_cpp_api_reference/hand_manager.md#handmanagerdestroy)로 처리합니다 |
| `Cannot run hand: cannot confirm drive enable — no RX on <이름>; restore CAN link and hand power, then reconnect()` | [`run()`](08_cpp_api_reference/hand.md#handrun)이 drive enable을 확인하지 못함 | 통신과 전원을 복구한 뒤 `reconnect()`를 호출합니다 |
| `Cannot enable hand: interface <이름> is gone — reconnect the CAN adapter, then recover with reconnect()` | interface가 사라짐 | adapter를 다시 연결하면 `lifecycle` 값이 `Faulted`로 전이하므로, 전이한 뒤에 `reconnect()`를 호출합니다 |
| `Cannot enable hand: interface <이름> was re-created — recover with reconnect() once the hand faults` | 같은 이름으로 재생성되어 기존 socket이 무효 | 수신이 끊겼으므로 `lifecycle` 값이 곧 `Faulted`로 전이합니다. 전이한 뒤에 `reconnect()`를 호출합니다 |
| `Cannot enable hand: communication not restored (no RX on <이름>) — restore CAN link and hand power, then recover with reconnect()` | 아직 수신이 없음 | 통신과 전원을 복구하면 `lifecycle` 값이 `Faulted`로 전이하므로, 전이한 뒤에 `reconnect()`를 호출합니다 |

`Faulted`에서 호출이 거부되면 message가 이 표의 문구가 아니라 `hand faulted (<원인>)` 형태입니다 —
[10. Faulted 원인 문구](#10-faulted-원인-문구)를 보십시오.

SDK는 RX가 약 100 ms 없으면 통신 오류로 판정합니다. 송신 큐가 일시적으로 가득 찬 경우(`EAGAIN`·`EWOULDBLOCK`·
`ENOBUFS`)는 오류로 보지 않습니다.

quick stop을 요구한 동안 제어·통신 루프는 매 cycle quick stop을 계속 전송합니다. 그래서 통신을
복구하면 drive가 quick stop을 수신합니다. `stop()`을 다시 호출하면 호출 시점의 상태를 다시
판정하므로 확인 수단으로 쓸 수 있습니다 — 여전히 도달하지 못했으면 다시 예외를 던지고, 도달했으면
`lifecycle` 값이 `Stopped`로 전이합니다.

> [!WARNING]
> quick stop을 확인하지 못한 채 끝나면 drive가 마지막 command를 그대로 유지할 수 있습니다.

## 7. `HardwareFault`

| Message | Cause | Remedy |
|---|---|---|
| `Cannot home hand: no actuator could be brought under control (actuators <목록>)` | 어느 actuator도 제어 상태로 들어가지 못함 | 전원과 drive fault를 확인합니다 |
| `Cannot home hand: some actuators could not be prepared for homing (actuators <목록>)` | 일부 actuator가 homing 준비 단계를 넘기지 못함 | 해당 index의 fault와 배선을 확인합니다 |
| `Cannot home hand: some actuators reported a homing error (actuators <목록>)` | drive가 homing 오류를 보고함 | 기구적 걸림과 hard stop 주변을 확인합니다 |
| `Cannot home hand: some actuators did not finish homing (actuators <목록>)` | 일부 actuator가 제한 시간 안에 homing을 끝내지 못함 | 기구적 저항과 hard stop 도달 여부를 확인합니다 |
| `Cannot home hand: homing did not complete (actuators <목록>)` | 위 어디에도 해당하지 않는 종료 | log를 남기고 SDK에 보고합니다 |
| `Cannot stop hand: drives did not reach quick stop within 500 ms — retry stop() or power off the hand` | 통신은 살아 있으나 drive가 quick stop에 도달하지 못함 | [`stop()`](08_cpp_api_reference/hand.md#handstop)을 다시 호출하거나 전원을 내립니다 |
| `Cannot disconnect hand: drives did not reach quick stop within 500 ms — power off the hand, or destroy the hand to disconnect anyway` | 위와 같음. quick stop을 확인하지 못했으므로 연결을 끊지 않음 | 전원을 내리거나, 그대로 끊어야 하면 [`destroy()`](08_cpp_api_reference/hand_manager.md#handmanagerdestroy)를 호출합니다 |
| `Cannot run hand: drives did not reach operation enabled within 4000 ms — retry run(), or check actuator faults via diagnostics()` | 통신은 살아 있고 fault도 없는데 drive가 Operation Enabled에 도달하지 못함 | actuator fault와 전원을 확인한 뒤 다시 [`run()`](08_cpp_api_reference/hand.md#handrun)을 호출합니다 |

homing 실패 문구는 모두 ` — retry home(), or check actuator faults via diagnostics()`로 끝납니다.
`<목록>`에는 실패한 actuator index가 들어가고, `disabled_actuators` 필드에 지정한 index는 보고되지 않습니다.

문구에 적힌 `diagnostics()`는 [`get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics)입니다.
값별 점검 항목은 [11. Actuator fault](#11-actuator-fault)에 있습니다.

`run()`이 enable을 확인하지 못하면 이미 올라온 actuator가 있을 수 있어 command를 지우고 quick stop을
요구합니다. 그래서 예외를 받은 뒤 lifecycle은 quick stop이 확인되는 시점에 `Stopped`가 됩니다.

## 8. `ControlLoopFault`

| Message | Cause | Remedy |
|---|---|---|
| `Cannot connect hand: failed to start SDK threads (<사유>)` | thread 생성 실패 | 자원 한도와 권한을 확인합니다 |
| `hand faulted (control loop terminated at cycle <n>) — call reconnect()` | 제어·통신 루프가 예외로 종료 | [`reconnect()`](08_cpp_api_reference/hand.md#handreconnect)로 복구하고, 재현 조건과 log를 SDK에 보고합니다 |

`Faulted`에서 호출이 거부되면 `hand faulted (<원인>)` 형태입니다 —
[10. Faulted 원인 문구](#10-faulted-원인-문구)를 보십시오.

`auto_reconnect` 값은 호출이 거부된 경우에는 동작하지 않습니다. 조건과 동작은
[C++ guide](07_cpp_usage_guide.md#94-recovering-from-faulted)에 있습니다.

## 9. `UnexpectedError`

| Message | Cause | Remedy |
|---|---|---|
| `Unexpected failure in <동작>: <내용>` | 분류되지 않은 예외 | 재현 조건과 전체 log를 SDK에 보고합니다 |
| `Unexpected failure in <동작>: unknown exception` | 알 수 없는 예외 | 위와 같습니다 |

## 10. Faulted 원인 문구

`Faulted`에서 호출이 거부되면 `Cannot <동작>: hand faulted (<원인>)<안내>` 형태이고, error code는
`<원인>`에 따라 갈립니다.

| `<원인>` | Meaning | [`ErrorCode`](08_cpp_api_reference/types_error.md#enum-errorcode) |
|---|---|---|
| `no RX since cycle <n>` | 해당 cycle 이후 수신이 끊김 | `CommunicationLost` |
| `CAN TX failed at cycle <n> (errno <번호>: <설명>)` | 송신이 비일시적 오류로 실패 | `CommunicationLost` |
| `control loop terminated at cycle <n>` | 제어·통신 루프가 예외로 종료 | `ControlLoopFault` |

`<안내>`는 부른 함수에 따라 달라집니다.

| Call | `<안내>` |
|---|---|
| [`connect()`](08_cpp_api_reference/hand.md#handconnect) | ` — call reconnect() to rebuild` |
| [`disconnect()`](08_cpp_api_reference/hand.md#handdisconnect) | ` — use reconnect() or destroy` |
| [`run()`](08_cpp_api_reference/hand.md#handrun) | ` — call reconnect(), then run()` |
| [`home()`](08_cpp_api_reference/hand.md#handhome) | ` — call reconnect() first` |
| [`stop()`](08_cpp_api_reference/hand.md#handstop) · [`set_command()`](08_cpp_api_reference/hand.md#handset_command) | ` — call reconnect()` |

`no RX since cycle <n>` 뒤에는 마지막 수신 오류가 붙기도 합니다. 오류 없이 수신만 끊긴
경우에는 붙지 않습니다.

| Suffix | Check first |
|---|---|
| `, last RX error: bus off` | 물리 bus 원인을 제거한 뒤 [CAN-FD setup](05_can_fd_setup.md)의 `restart-ms`와 `ip -details link show`로 kernel 복구를 확인합니다 |
| `, last RX error: bus error (no ACK)` | 전원이 꺼졌거나 hand가 분리됐는지 확인합니다 |
| `, last RX error: bus error` | termination·noise·bit timing을 확인합니다 |
| `, last RX error: malformed data` | bit rate와 CAN-FD 지원을 확인합니다 |
| `, last RX error: conflicting command` | 다른 master가 같은 hand를 제어 중인지 확인합니다 |
| `, last RX error: unknown data` | bus에 섞인 다른 traffic을 확인합니다 |

## 11. Actuator fault

[`get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics)의 `actuator_health.fault` 필드는
[`ErrorCode`](08_cpp_api_reference/types_error.md#enum-errorcode)와 서로 독립적인 정보입니다. 예외를 던지지 않아도 값이 남으므로 이 값도 함께 기록하십시오.
값은 [`ActuatorFault`](08_cpp_api_reference/types_state.md#enum-actuatorfault) enum이고 `None`이면 해당 actuator에
fault가 없습니다. 배열 전체를 훑는 예제는 [C++ guide](07_cpp_usage_guide.md#81-actuator-faults)에
있습니다.

| `to_string()` value | Check first |
|---|---|
| `OverVoltageError` · `UnderVoltageError` | 공급 전압, cable 전압 강하, 회생 조건 |
| `OverCurrentError` · `OverLoadError` | 기구적 걸림, 충돌, gain, effort 상한 |
| `OverTemperatureError` | duty cycle, 냉각, 주변 온도 |
| `CurrentDetectionError` | drive 전류 센싱 계통 |
| `FollowingError` · `SpeedError` | 목표 step 크기, 부하, encoder, controller 튜닝 |
| `CommunicationError` | drive 쪽 bus와 cable |
| `HallSensorError` | hall sensor 배선과 connector |
| `PositiveLimitSwitchError` · `NegativeLimitSwitchError` | limit switch와 기구 위치 |
| `EmergencySwitchError` · `Sto1Error` · `Sto2Error` (STO = Safe Torque Off) | 비상 정지 회로와 safety 배선 |
| `SerialEncoderChannelAError` · `SerialEncoderChannelBError` | encoder 신호 품질과 connector |
| `SerialEncoderChannelADisconnectedError` · `SerialEncoderChannelBDisconnectedError` | encoder cable 분리 |
| `Unknown` | 위 이름에 없는 코드 |

`Unknown`은 SDK의 RT log에 `UnknownError 0x<코드>`로 남으므로 해당 코드를 함께 보고하십시오.

원인을 제거하기 전에 반복해서 재가동하지 마십시오.
