# C++ API Reference

Public 심볼을 header 파일 단위로 정리했습니다. 함수는 선언과 계약을, struct는 필드 표를
싣습니다. 쓰는 순서와 배경은 [C++ guide](07_cpp_usage_guide.md), 오류 문구별 조치는
[Error messages](15_error_messages.md)에 있습니다.

```cpp
#include <aidin_hand2/aidin_hand2.hpp>   // public 심볼 전부, namespace aidin_hand2
namespace ah2 = aidin_hand2;             // 이 문서의 예제 alias
```

함수 항목은 **Parameters → Returns → Throws → Preconditions → Postconditions → Notes** 순서로
쓰고, 해당 없는 줄은 비웁니다.

## Contents

&nbsp;&nbsp;[**`hand/hand_manager.hpp`**](08_cpp_api_reference/hand_manager.md) — `HandManager`<br>
&nbsp;&nbsp;[**`hand/hand.hpp`**](08_cpp_api_reference/hand.md) — `Hand`<br>
&nbsp;&nbsp;[**`hand/hand_kinematics.hpp`**](08_cpp_api_reference/hand_kinematics.md) — FK / IK<br>
&nbsp;&nbsp;[**`types/description.hpp`**](08_cpp_api_reference/types_description.md) — 상수, `HandSide`, `Finger`, 배열 규약<br>
&nbsp;&nbsp;[**`types/config.hpp`**](08_cpp_api_reference/types_config.md) — `HandConfig`, `ControllerConfig`<br>
&nbsp;&nbsp;[**`types/command.hpp`**](08_cpp_api_reference/types_command.md) — `CommandMode`, command 5종<br>
&nbsp;&nbsp;[**`types/state.hpp`**](08_cpp_api_reference/types_state.md) — `HandLifecycle`, `HomingState`, `HandState`, `ActuatorFault`<br>
&nbsp;&nbsp;[**`types/diagnostics.hpp`**](08_cpp_api_reference/types_diagnostics.md) — `Diagnostics`<br>
&nbsp;&nbsp;[**`types/error.hpp`**](08_cpp_api_reference/types_error.md) — `ErrorCode`, `Exception`<br>
&nbsp;&nbsp;[**`logging/logging.hpp`**](08_cpp_api_reference/logging.md) — `LogLevel`, logging 함수<br>
&nbsp;&nbsp;[**`version.hpp`**](08_cpp_api_reference/version.md) — version 상수와 `find_package`

---

## 함수 인덱스

public 함수 전부입니다. 선언된 파일별로 묶었습니다. 이름은 정규화 표기이며,
`::`가 붙은 것은 그 클래스·struct의 멤버, `::`가 없는 것은 `aidin_hand2` namespace 수준의 자유 함수입니다.

**[`hand/hand_manager.hpp`](08_cpp_api_reference/hand_manager.md)**

| 함수 | 역할 |
|---|---|
| [`HandManager::HandManager()`](08_cpp_api_reference/hand_manager.md#handmanagerhandmanager) | 빈 manager를 만들거나 다른 manager에서 이동 |
| [`HandManager::~HandManager()`](08_cpp_api_reference/hand_manager.md#handmanagerhandmanager-1) | 소유한 hand를 모두 quick stop 후 종료 |
| [`HandManager::operator=`](08_cpp_api_reference/hand_manager.md#handmanageroperator) | 이동 대입 |
| [`HandManager::create()`](08_cpp_api_reference/hand_manager.md#handmanagercreate) | config를 검증하고 `Hand` 생성 |
| [`HandManager::destroy()`](08_cpp_api_reference/hand_manager.md#handmanagerdestroy) | `Hand` 하나를 종료하고 handle 무효화 |
| [`HandManager::destroy_all()`](08_cpp_api_reference/hand_manager.md#handmanagerdestroy_all) | 소유한 `Hand` 전부 종료 |

**[`hand/hand.hpp`](08_cpp_api_reference/hand.md)**

| 함수 | 역할 |
|---|---|
| [`Hand::connect()`](08_cpp_api_reference/hand.md#handconnect) | CAN 연결 및 state 수신 시작 |
| [`Hand::disconnect()`](08_cpp_api_reference/hand.md#handdisconnect) | actuator quick stop 후 CAN 연결 종료 |
| [`Hand::reconnect()`](08_cpp_api_reference/hand.md#handreconnect) | fault 뒤 CAN 재연결 |
| [`Hand::run()`](08_cpp_api_reference/hand.md#handrun) | actuator enable을 확인하고 제어 시작 |
| [`Hand::stop()`](08_cpp_api_reference/hand.md#handstop) | actuator quick stop |
| [`Hand::home()`](08_cpp_api_reference/hand.md#handhome) | homing 완료까지 대기 (blocking) |
| [`Hand::set_command(Idle)`](08_cpp_api_reference/hand.md#handset_command) | 출력 0 |
| [`Hand::set_command(JointPositionCommand)`](08_cpp_api_reference/hand.md#handset_command) | joint 위치 목표 |
| [`Hand::set_command(JointImpedanceCommand)`](08_cpp_api_reference/hand.md#handset_command) | joint impedance 목표 |
| [`Hand::set_command(ActuatorPositionCommand)`](08_cpp_api_reference/hand.md#handset_command) | actuator 위치 목표 |
| [`Hand::set_command(ActuatorEffortCommand)`](08_cpp_api_reference/hand.md#handset_command) | actuator effort 목표 |
| [`Hand::set_max_effort(double)`](08_cpp_api_reference/hand.md#handset_max_effort) | 공통 effort 상한 |
| [`Hand::set_max_effort(array<double, 16>)`](08_cpp_api_reference/hand.md#handset_max_effort) | actuator별 effort 상한 |
| [`Hand::set_controller_config()`](08_cpp_api_reference/hand.md#handset_controller_config) | 위치 filter와 impedance gain |
| [`Hand::get_state()`](08_cpp_api_reference/hand.md#handget_state) | 최신 [`HandState`](08_cpp_api_reference/types_state.md#handstate) |
| [`Hand::get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics) | 최신 [`Diagnostics`](08_cpp_api_reference/types_diagnostics.md#diagnostics) |
| [`Hand::get_command_mode()`](08_cpp_api_reference/hand.md#handget_command_mode) | 지금 활성인 [`CommandMode`](08_cpp_api_reference/types_command.md#enum-commandmode) |

**[`hand/hand_kinematics.hpp`](08_cpp_api_reference/hand_kinematics.md)**

| 함수 | 역할 |
|---|---|
| [`fk_actuator_to_joint()`](08_cpp_api_reference/hand_kinematics.md#fk_actuator_to_joint) | actuator encoder 16개 → joint 21개 |
| [`ik_joint_to_actuator()`](08_cpp_api_reference/hand_kinematics.md#ik_joint_to_actuator) | active joint 16개 → actuator encoder 16개 |

**[`types/config.hpp`](08_cpp_api_reference/types_config.md)**

| 함수 | 역할 |
|---|---|
| [`HandConfig::HandConfig()`](08_cpp_api_reference/types_config.md#handconfighandconfig) | 기본값으로 채운 config |

**[`types/command.hpp`](08_cpp_api_reference/types_command.md)**

| 함수 | 역할 |
|---|---|
| [`JointPositionCommand::clamp()`](08_cpp_api_reference/types_command.md#jointpositioncommandclamp) | target을 joint limit 안으로 조정 |
| [`JointImpedanceCommand::clamp()`](08_cpp_api_reference/types_command.md#jointimpedancecommandclamp) | target을 joint limit 안으로 조정 |
| [`to_command_mode()`](08_cpp_api_reference/types_command.md#to_command_mode) | variant에서 `CommandMode`를 꺼냄 |

**[`types/state.hpp`](08_cpp_api_reference/types_state.md)**

| 함수 | 역할 |
|---|---|
| [`to_string(HandLifecycle)`](08_cpp_api_reference/types_state.md#enum-handlifecycle) | lifecycle 이름 |
| [`to_string(HomingState)`](08_cpp_api_reference/types_state.md#enum-homingstate) | homing 상태 이름 |
| [`to_string(ActuatorFault)`](08_cpp_api_reference/types_state.md#enum-actuatorfault) | actuator error code 이름 |

**[`types/error.hpp`](08_cpp_api_reference/types_error.md)**

| 함수 | 역할 |
|---|---|
| [`to_string(ErrorCode)`](08_cpp_api_reference/types_error.md#enum-errorcode) | error code 이름 |
| [`Exception::Exception()`](08_cpp_api_reference/types_error.md#exceptionexception) | code와 message로 예외 생성 |
| [`Exception::code()`](08_cpp_api_reference/types_error.md#exceptioncode) | `ErrorCode` 반환 |

**[`logging/logging.hpp`](08_cpp_api_reference/logging.md)**

| 함수 | 역할 |
|---|---|
| [`set_log_level()`](08_cpp_api_reference/logging.md#set_log_level) | console sink level |
| [`set_log_to_console()`](08_cpp_api_reference/logging.md#set_log_to_console) | console sink on/off |
| [`set_log_to_file()`](08_cpp_api_reference/logging.md#set_log_to_file) | rotating file sink |
| [`set_log_callback()`](08_cpp_api_reference/logging.md#set_log_callback) | application logger로 중계 |
| [`flush_log()`](08_cpp_api_reference/logging.md#flush_log) | 남은 log 내보내기 |

## 타입 인덱스

enum·struct·class·alias 전부입니다. 선언된 파일별로 묶고, 중첩 타입은 `::`로 밝힙니다.

**[`hand/hand_manager.hpp`](08_cpp_api_reference/hand_manager.md)**

| 타입 | 종류 | 역할 |
|---|---|---|
| [`HandManager`](08_cpp_api_reference/hand_manager.md#handmanager) | class | hand 자원을 만들고 소유하는 move-only 타입 |

**[`hand/hand.hpp`](08_cpp_api_reference/hand.md)**

| 타입 | 종류 | 역할 |
|---|---|---|
| [`Hand`](08_cpp_api_reference/hand.md#hand) | class | 연결·제어·조회를 하는 handle |

**[`types/description.hpp`](08_cpp_api_reference/types_description.md)**

| 타입 | 종류 | 역할 |
|---|---|---|
| [상수](08_cpp_api_reference/types_description.md#constants) | `constexpr` | actuator·joint·tactile 개수 |
| [`HandSide`](08_cpp_api_reference/types_description.md#enum-handside) | enum | 왼손 / 오른손 |
| [`Finger`](08_cpp_api_reference/types_description.md#enum-finger) | enum | tactile finger index |

**[`types/config.hpp`](08_cpp_api_reference/types_config.md)**

| 타입 | 종류 | 역할 |
|---|---|---|
| [`HandConfig`](08_cpp_api_reference/types_config.md#handconfig) | struct | [`HandManager::create()`](08_cpp_api_reference/hand_manager.md#handmanagercreate)에 넘기는 설정 |
| [`ControllerConfig`](08_cpp_api_reference/types_config.md#controllerconfig) | struct | [`Hand::set_controller_config()`](08_cpp_api_reference/hand.md#handset_controller_config)로 바꾸는 tuning |
| [`ControllerConfig::JointPositionController`](08_cpp_api_reference/types_config.md#controllerconfigjointpositioncontroller) | struct | 위치 filter 설정 |
| [`ControllerConfig::JointImpedanceController`](08_cpp_api_reference/types_config.md#controllerconfigjointimpedancecontroller) | struct | impedance gain 설정 |

**[`types/command.hpp`](08_cpp_api_reference/types_command.md)**

| 타입 | 종류 | 역할 |
|---|---|---|
| [`CommandMode`](08_cpp_api_reference/types_command.md#enum-commandmode) | enum | 활성 controller |
| [`Idle`](08_cpp_api_reference/types_command.md#idle) | struct | 무토크 |
| [`JointPositionCommand`](08_cpp_api_reference/types_command.md#jointpositioncommand) | struct | joint 위치 목표 |
| [`JointImpedanceCommand`](08_cpp_api_reference/types_command.md#jointimpedancecommand) | struct | joint impedance 목표 |
| [`ActuatorPositionCommand`](08_cpp_api_reference/types_command.md#actuatorpositioncommand) | struct | actuator 위치 목표 |
| [`ActuatorEffortCommand`](08_cpp_api_reference/types_command.md#actuatoreffortcommand) | struct | actuator effort 목표 |
| [`ControllerCommand`](08_cpp_api_reference/types_command.md#type-aliases) | alias | command 하나를 담는 variant |

**[`types/state.hpp`](08_cpp_api_reference/types_state.md)**

| 타입 | 종류 | 역할 |
|---|---|---|
| [`HandLifecycle`](08_cpp_api_reference/types_state.md#enum-handlifecycle) | enum | 손의 상태 5개 |
| [`HomingState`](08_cpp_api_reference/types_state.md#enum-homingstate) | enum | 원점 상태 |
| [`ActuatorFault`](08_cpp_api_reference/types_state.md#enum-actuatorfault) | enum | actuator error code |
| [`CommandSource`](08_cpp_api_reference/types_state.md#enum-commandsource) | enum | 이번 cycle에 쓴 출처 |
| [`HandState`](08_cpp_api_reference/types_state.md#handstate) | struct | [`Hand::get_state()`](08_cpp_api_reference/hand.md#handget_state)가 돌려주는 값 |
| [`ActuatorState`](08_cpp_api_reference/types_state.md#actuatorstate) | struct | actuator 관측 |
| [`JointState`](08_cpp_api_reference/types_state.md#jointstate) | struct | FK 결과 |
| [`TactileState`](08_cpp_api_reference/types_state.md#tactilestate) | struct | 촉각 |
| [`CommandedState`](08_cpp_api_reference/types_state.md#commandedstate) | struct | 이번 cycle의 command 입력·출력 |
| [`ActuatorPositionSetpoint` / `ActuatorEffortSetpoint`](08_cpp_api_reference/types_state.md#actuatorpositionsetpoint--actuatoreffortsetpoint) | struct | controller 출력 setpoint |
| [`ActuatorHealth`](08_cpp_api_reference/types_state.md#actuatorhealth) | struct | actuator별 enable·fault |
| [`ControllerOutput`](08_cpp_api_reference/types_state.md#type-aliases) | alias | setpoint variant |

**[`types/diagnostics.hpp`](08_cpp_api_reference/types_diagnostics.md)**

| 타입 | 종류 | 역할 |
|---|---|---|
| [`Diagnostics`](08_cpp_api_reference/types_diagnostics.md#diagnostics) | struct | [`Hand::get_diagnostics()`](08_cpp_api_reference/hand.md#handget_diagnostics)가 돌려주는 값 |

**[`types/error.hpp`](08_cpp_api_reference/types_error.md)**

| 타입 | 종류 | 역할 |
|---|---|---|
| [`ErrorCode`](08_cpp_api_reference/types_error.md#enum-errorcode) | enum | 실패 분류 |
| [`Exception`](08_cpp_api_reference/types_error.md#exception--stdruntime_error) | class | public 호출이 던지는 예외 |

**[`logging/logging.hpp`](08_cpp_api_reference/logging.md)**

| 타입 | 종류 | 역할 |
|---|---|---|
| [`LogLevel`](08_cpp_api_reference/logging.md#enum-loglevel) | enum | log level |
| [`LogCallback`](08_cpp_api_reference/logging.md#type-aliases) | alias | callback 형태 |

**[`version.hpp`](08_cpp_api_reference/version.md)**

| 타입 | 종류 | 역할 |
|---|---|---|
| version 상수 | `constexpr` | `kVersionMajor`·`kVersionMinor`·`kVersionPatch`·`kVersionString` |
