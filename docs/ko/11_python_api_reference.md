# Python API Reference

Python 패키지 `aidin_hand2`의 public 심볼을 주제별로 정리했습니다. 함수는 signature와 계약을, class는
필드 표를 싣습니다. 쓰는 순서와 배경은 [Python guide](10_python_usage_guide.md), 오류 문구별 조치는
[Error messages](15_error_messages.md)에 있습니다.

```python
import aidin_hand2 as ah2   # public 심볼 전부
```

함수 항목은 **Parameters → Returns → Raises → Preconditions → Postconditions → Notes** 순서로
쓰고, 해당 없는 줄은 비웁니다. 배열은 numpy 배열이며 dtype과 shape를 함께 적습니다. 배열 인자에는
같은 길이의 list나 tuple도 넘길 수 있습니다.

## Contents

&nbsp;&nbsp;[**1. Function index**](#1-function-index)<br>
&nbsp;&nbsp;[**2. Type index**](#2-type-index)<br>
&nbsp;&nbsp;[**3. Version**](#3-version)

주제별 문서는 다음과 같습니다.

&nbsp;&nbsp;[**HandManager**](11_python_api_reference/hand_manager.md) — `HandManager`<br>
&nbsp;&nbsp;[**Hand**](11_python_api_reference/hand.md) — `Hand`<br>
&nbsp;&nbsp;[**Kinematics**](11_python_api_reference/kinematics.md) — FK / IK<br>
&nbsp;&nbsp;[**Hand description**](11_python_api_reference/description.md) — 상수, `HandSide`, `Finger`, 배열 배치<br>
&nbsp;&nbsp;[**Config**](11_python_api_reference/config.md) — `HandConfig`, `ControllerConfig`<br>
&nbsp;&nbsp;[**Command**](11_python_api_reference/command.md) — `CommandMode`, command 5종<br>
&nbsp;&nbsp;[**State**](11_python_api_reference/state.md) — `HandLifecycle`, `HomingState`, `ActuatorFault`, `HandState`<br>
&nbsp;&nbsp;[**Diagnostics**](11_python_api_reference/diagnostics.md) — `Diagnostics`<br>
&nbsp;&nbsp;[**Errors**](11_python_api_reference/error.md) — `ErrorCode`, `Error`<br>
&nbsp;&nbsp;[**Logging**](11_python_api_reference/logging.md) — `LogLevel`, logging 함수

---

## 1. Function index

public 함수 전부입니다. `.`이 붙은 것은 그 class의 method이고, `.`이 없는 것은 모듈 수준의
함수입니다.

**[HandManager](11_python_api_reference/hand_manager.md)**

| 함수 | 역할 |
|---|---|
| [`HandManager()`](11_python_api_reference/hand_manager.md#handmanager__init__) | 빈 manager |
| [`HandManager.create()`](11_python_api_reference/hand_manager.md#handmanagercreate) | config를 검증하고 `Hand` 생성 |
| [`HandManager.destroy()`](11_python_api_reference/hand_manager.md#handmanagerdestroy) | `Hand` 하나를 정리하고 무효화 |
| [`HandManager.destroy_all()`](11_python_api_reference/hand_manager.md#handmanagerdestroy_all) | 소유한 `Hand` 전부 정리 |

**[Hand](11_python_api_reference/hand.md)**

| 함수 | 역할 |
|---|---|
| [`Hand.connect()`](11_python_api_reference/hand.md#handconnect) | CAN 연결 및 state 수신 시작 |
| [`Hand.disconnect()`](11_python_api_reference/hand.md#handdisconnect) | actuator quick stop 후 CAN 연결 종료 |
| [`Hand.reconnect()`](11_python_api_reference/hand.md#handreconnect) | fault 뒤 CAN 재연결 |
| [`Hand.run()`](11_python_api_reference/hand.md#handrun) | actuator enable을 확인하고 제어 시작 |
| [`Hand.stop()`](11_python_api_reference/hand.md#handstop) | actuator quick stop |
| [`Hand.home()`](11_python_api_reference/hand.md#handhome) | homing 완료까지 대기 |
| [`Hand.set_command()`](11_python_api_reference/hand.md#handset_command) | command 적용 |
| [`Hand.set_max_effort()`](11_python_api_reference/hand.md#handset_max_effort) | effort 상한 |
| [`Hand.set_controller_config()`](11_python_api_reference/hand.md#handset_controller_config) | 위치 filter와 impedance gain |
| [`Hand.get_state()`](11_python_api_reference/hand.md#handget_state) | 최신 [`HandState`](11_python_api_reference/state.md#handstate) |
| [`Hand.get_diagnostics()`](11_python_api_reference/hand.md#handget_diagnostics) | 최신 [`Diagnostics`](11_python_api_reference/diagnostics.md#diagnostics) |
| [`Hand.get_command_mode()`](11_python_api_reference/hand.md#handget_command_mode) | 지금 활성인 [`CommandMode`](11_python_api_reference/command.md#enum-commandmode) |

**[Kinematics](11_python_api_reference/kinematics.md)**

| 함수 | 역할 |
|---|---|
| [`fk_actuator_to_joint()`](11_python_api_reference/kinematics.md#fk_actuator_to_joint) | actuator encoder 16개 → joint 21개 |
| [`ik_joint_to_actuator()`](11_python_api_reference/kinematics.md#ik_joint_to_actuator) | active joint 16개 → actuator encoder 16개 |

**[Command](11_python_api_reference/command.md)**

| 함수 | 역할 |
|---|---|
| [`JointPositionCommand.clamp()`](11_python_api_reference/command.md#jointpositioncommandclamp) | target을 도달 범위 안으로 투영 |
| [`JointImpedanceCommand.clamp()`](11_python_api_reference/command.md#jointimpedancecommandclamp) | target을 도달 범위 안으로 투영 |

**[Logging](11_python_api_reference/logging.md)**

| 함수 | 역할 |
|---|---|
| [`set_log_level()`](11_python_api_reference/logging.md#set_log_level) | console sink level |
| [`set_log_to_console()`](11_python_api_reference/logging.md#set_log_to_console) | console sink on/off |
| [`set_log_to_file()`](11_python_api_reference/logging.md#set_log_to_file) | rotating file sink |
| [`set_log_callback()`](11_python_api_reference/logging.md#set_log_callback) | Python 함수로 전달 |
| [`flush_log()`](11_python_api_reference/logging.md#flush_log) | 남은 log 내보내기 |

## 2. Type index

class·enum 전부입니다. 안에 정의된 class는 `.`으로 밝힙니다.

**[HandManager](11_python_api_reference/hand_manager.md) · [Hand](11_python_api_reference/hand.md)**

| 타입 | 종류 | 역할 |
|---|---|---|
| [`HandManager`](11_python_api_reference/hand_manager.md#handmanager) | class | hand 자원을 만들고 소유. `with` 블록 지원 |
| [`Hand`](11_python_api_reference/hand.md#hand) | class | 연결·제어·조회를 하는 handle |

**[Hand description](11_python_api_reference/description.md)**

| 타입 | 종류 | 역할 |
|---|---|---|
| [상수](11_python_api_reference/description.md#constants) | `int` | actuator·joint·tactile 개수 |
| [`HandSide`](11_python_api_reference/description.md#enum-handside) | `enum.Enum` | 왼손 / 오른손 |
| [`Finger`](11_python_api_reference/description.md#enum-finger) | `enum.IntEnum` | tactile finger index |

**[Config](11_python_api_reference/config.md)**

| 타입 | 종류 | 역할 |
|---|---|---|
| [`HandConfig`](11_python_api_reference/config.md#handconfig) | class | [`HandManager.create()`](11_python_api_reference/hand_manager.md#handmanagercreate)에 넘기는 설정 |
| [`ControllerConfig`](11_python_api_reference/config.md#controllerconfig) | class | [`Hand.set_controller_config()`](11_python_api_reference/hand.md#handset_controller_config)로 바꾸는 tuning |
| [`ControllerConfig.JointPositionController`](11_python_api_reference/config.md#controllerconfigjointpositioncontroller) | class | 위치 filter 설정 |
| [`ControllerConfig.JointImpedanceController`](11_python_api_reference/config.md#controllerconfigjointimpedancecontroller) | class | impedance gain 설정 |

**[Command](11_python_api_reference/command.md)**

| 타입 | 종류 | 역할 |
|---|---|---|
| [`CommandMode`](11_python_api_reference/command.md#enum-commandmode) | `enum.Enum` | 활성 controller |
| [`Idle`](11_python_api_reference/command.md#idle) | class | 무토크 |
| [`JointPositionCommand`](11_python_api_reference/command.md#jointpositioncommand) | class | joint 위치 목표 |
| [`JointImpedanceCommand`](11_python_api_reference/command.md#jointimpedancecommand) | class | joint impedance 목표 |
| [`ActuatorPositionCommand`](11_python_api_reference/command.md#actuatorpositioncommand) | class | actuator 위치 목표 |
| [`ActuatorEffortCommand`](11_python_api_reference/command.md#actuatoreffortcommand) | class | actuator effort 목표 |

**[State](11_python_api_reference/state.md)**

| 타입 | 종류 | 역할 |
|---|---|---|
| [`HandLifecycle`](11_python_api_reference/state.md#enum-handlifecycle) | `enum.Enum` | 손의 상태 5개 |
| [`HomingState`](11_python_api_reference/state.md#enum-homingstate) | `enum.Enum` | 원점 상태 |
| [`ActuatorFault`](11_python_api_reference/state.md#enum-actuatorfault) | `enum.IntEnum` | actuator error code |
| [`CommandSource`](11_python_api_reference/state.md#enum-commandsource) | `enum.Enum` | 이번 cycle에 쓴 출처 |
| [`HandState`](11_python_api_reference/state.md#handstate) | class | [`Hand.get_state()`](11_python_api_reference/hand.md#handget_state)가 돌려주는 값 |
| [`ActuatorState`](11_python_api_reference/state.md#actuatorstate) | class | actuator 관측 |
| [`JointState`](11_python_api_reference/state.md#jointstate) | class | FK 결과 |
| [`TactileState`](11_python_api_reference/state.md#tactilestate) | class | 촉각 |
| [`CommandedState`](11_python_api_reference/state.md#commandedstate) | class | 이번 cycle의 command 입력·출력 |
| [`ActuatorPositionSetpoint` / `ActuatorEffortSetpoint`](11_python_api_reference/state.md#actuatorpositionsetpoint--actuatoreffortsetpoint) | class | controller 출력 setpoint |
| [`ActuatorHealth`](11_python_api_reference/state.md#actuatorhealth) | class | actuator별 enable·fault |

**[Diagnostics](11_python_api_reference/diagnostics.md)**

| 타입 | 종류 | 역할 |
|---|---|---|
| [`Diagnostics`](11_python_api_reference/diagnostics.md#diagnostics) | class | [`Hand.get_diagnostics()`](11_python_api_reference/hand.md#handget_diagnostics)가 돌려주는 값 |

**[Errors](11_python_api_reference/error.md)**

| 타입 | 종류 | 역할 |
|---|---|---|
| [`ErrorCode`](11_python_api_reference/error.md#enum-errorcode) | `enum.Enum` | 실패 분류 |
| [`Error`](11_python_api_reference/error.md#error) | exception | SDK 호출이 던지는 예외 |

**[Logging](11_python_api_reference/logging.md)**

| 타입 | 종류 | 역할 |
|---|---|---|
| [`LogLevel`](11_python_api_reference/logging.md#enum-loglevel) | `enum.Enum` | log level |

## 3. Version

```python
ah2.__version__   # "0.7.1", 설치된 패키지의 버전
```

버전은 C++ SDK와 같은 번호 체계입니다. 0.x에서는 minor가 다르면 API가 호환되지 않으므로,
`requirements.txt`에는 `aidin-hand2==0.7.1`처럼 버전을 고정하십시오. C++ SDK와 같은 컴퓨터에서 쓸
때 버전을 맞추는 방법은 [SDK build & install](06_sdk_build_and_install.md#21-install)에 있습니다.
