// 목적: enum→string / enum-변환 순수 헬퍼의 전수(exhaustive) 값 검증.
//       to_string(ErrorCode) · to_string(ActuatorFault) · to_string(HandLifecycle) ·
//       to_command_mode(ControllerCommand) 의 모든 enum 값/입력을 기대값과 직접 대조.
// 방법: constexpr/noexcept 순수 함수이므로 결정적 값 대조만 수행. enum 이 성장했을 때
//       빠진 case 를 잡는 것이 이 테스트의 가치 — 각 값마다 정확한 문자열/모드를 명시한다.

#include <cstdio>
#include <cstring>

#include <aidin_hand2/types/command.hpp>
#include <aidin_hand2/types/error.hpp>
#include <aidin_hand2/types/state.hpp>

namespace
{

int g_failures = 0;

void check(bool ok, const char* what)
{
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    ++g_failures;
  }
}

// 문자열 동등 비교 헬퍼 (nullptr 안전).
bool str_eq(const char* a, const char* b)
{
  return a != nullptr && b != nullptr && std::strcmp(a, b) == 0;
}

using aidin_hand2::to_string;

void test_error_code()
{
  using aidin_hand2::ErrorCode;
  check(str_eq(to_string(ErrorCode::None), "None"), "ErrorCode::None → \"None\"");
  check(str_eq(to_string(ErrorCode::InvalidArgument), "InvalidArgument"),
        "ErrorCode::InvalidArgument");
  check(str_eq(to_string(ErrorCode::WrongCallOrder), "WrongCallOrder"),
        "ErrorCode::WrongCallOrder");
  check(str_eq(to_string(ErrorCode::InterfaceUnavailable), "InterfaceUnavailable"),
        "ErrorCode::InterfaceUnavailable");
  check(str_eq(to_string(ErrorCode::CommunicationLost), "CommunicationLost"),
        "ErrorCode::CommunicationLost");
  check(str_eq(to_string(ErrorCode::HardwareFault), "HardwareFault"),
        "ErrorCode::HardwareFault");
  check(str_eq(to_string(ErrorCode::ControlLoopFault), "ControlLoopFault"),
        "ErrorCode::ControlLoopFault");
  check(str_eq(to_string(ErrorCode::UnexpectedError), "UnexpectedError"),
        "ErrorCode::UnexpectedError");
}

void test_homing_state()
{
  using aidin_hand2::HomingState;
  std::printf("[to_string] HomingState\n");
  check(str_eq(to_string(HomingState::NotRun), "NotRun"), "HomingState::NotRun");
  check(str_eq(to_string(HomingState::Succeeded), "Succeeded"), "HomingState::Succeeded");
  check(str_eq(to_string(HomingState::InProgress), "InProgress"), "HomingState::InProgress");
  check(str_eq(to_string(HomingState::Failed), "Failed"), "HomingState::Failed");
  check(static_cast<int>(HomingState::Succeeded) == 1, "HomingState::Succeeded == 1");
}

void test_actuator_fault()
{
  using aidin_hand2::ActuatorFault;
  // None 은 로그 노출용으로 빈 문자열 계약.
  check(str_eq(to_string(ActuatorFault::None), ""), "ActuatorFault::None → \"\"");
  check(str_eq(to_string(ActuatorFault::OverCurrentError), "OverCurrentError"),
        "ActuatorFault::OverCurrentError");
  check(str_eq(to_string(ActuatorFault::OverVoltageError), "OverVoltageError"),
        "ActuatorFault::OverVoltageError");
  check(str_eq(to_string(ActuatorFault::UnderVoltageError), "UnderVoltageError"),
        "ActuatorFault::UnderVoltageError");
  check(str_eq(to_string(ActuatorFault::OverTemperatureError), "OverTemperatureError"),
        "ActuatorFault::OverTemperatureError");
  check(str_eq(to_string(ActuatorFault::CurrentDetectionError), "CurrentDetectionError"),
        "ActuatorFault::CurrentDetectionError");
  check(str_eq(to_string(ActuatorFault::SpeedError), "SpeedError"),
        "ActuatorFault::SpeedError");
  check(str_eq(to_string(ActuatorFault::CommunicationError), "CommunicationError"),
        "ActuatorFault::CommunicationError");
  check(str_eq(to_string(ActuatorFault::FollowingError), "FollowingError"),
        "ActuatorFault::FollowingError");
  check(str_eq(to_string(ActuatorFault::HallSensorError), "HallSensorError"),
        "ActuatorFault::HallSensorError");
  check(str_eq(to_string(ActuatorFault::OverLoadError), "OverLoadError"),
        "ActuatorFault::OverLoadError");
  check(str_eq(to_string(ActuatorFault::PositiveLimitSwitchError), "PositiveLimitSwitchError"),
        "ActuatorFault::PositiveLimitSwitchError");
  check(str_eq(to_string(ActuatorFault::NegativeLimitSwitchError), "NegativeLimitSwitchError"),
        "ActuatorFault::NegativeLimitSwitchError");
  check(str_eq(to_string(ActuatorFault::EmergencySwitchError), "EmergencySwitchError"),
        "ActuatorFault::EmergencySwitchError");
  check(str_eq(to_string(ActuatorFault::Sto1Error), "Sto1Error"),
        "ActuatorFault::Sto1Error");
  check(str_eq(to_string(ActuatorFault::Sto2Error), "Sto2Error"),
        "ActuatorFault::Sto2Error");
  check(str_eq(to_string(ActuatorFault::SerialEncoderChannelAError),
               "SerialEncoderChannelAError"),
        "ActuatorFault::SerialEncoderChannelAError");
  check(str_eq(to_string(ActuatorFault::SerialEncoderChannelADisconnectedError),
               "SerialEncoderChannelADisconnectedError"),
        "ActuatorFault::SerialEncoderChannelADisconnectedError");
  check(str_eq(to_string(ActuatorFault::SerialEncoderChannelBError),
               "SerialEncoderChannelBError"),
        "ActuatorFault::SerialEncoderChannelBError");
  check(str_eq(to_string(ActuatorFault::SerialEncoderChannelBDisconnectedError),
               "SerialEncoderChannelBDisconnectedError"),
        "ActuatorFault::SerialEncoderChannelBDisconnectedError");
}

void test_hand_lifecycle()
{
  using aidin_hand2::HandLifecycle;
  check(str_eq(to_string(HandLifecycle::Disconnected), "Disconnected"),
        "HandLifecycle::Disconnected");
  check(str_eq(to_string(HandLifecycle::Connected), "Connected"),
        "HandLifecycle::Connected");
  check(str_eq(to_string(HandLifecycle::Running), "Running"),
        "HandLifecycle::Running");
  check(str_eq(to_string(HandLifecycle::Stopped), "Stopped"),
        "HandLifecycle::Stopped");
  check(str_eq(to_string(HandLifecycle::Faulted), "Faulted"),
        "HandLifecycle::Faulted");
}

void test_to_command_mode()
{
  using namespace aidin_hand2;
  // ControllerCommand variant 의 각 대안 타입 → 대응 CommandMode.
  check(to_command_mode(ControllerCommand{Idle{}}) == CommandMode::Idle,
        "Idle → CommandMode::Idle");
  check(to_command_mode(ControllerCommand{JointPositionCommand{}}) == CommandMode::JointPosition,
        "JointPositionCommand → CommandMode::JointPosition");
  check(to_command_mode(ControllerCommand{JointImpedanceCommand{}}) == CommandMode::JointImpedance,
        "JointImpedanceCommand → CommandMode::JointImpedance");
  check(to_command_mode(ControllerCommand{ActuatorPositionCommand{}}) == CommandMode::ActuatorPosition,
        "ActuatorPositionCommand → CommandMode::ActuatorPosition");
  check(to_command_mode(ControllerCommand{ActuatorEffortCommand{}}) == CommandMode::ActuatorEffort,
        "ActuatorEffortCommand → CommandMode::ActuatorEffort");
}

// enum 이 성장하면 컴파일러가 -Wswitch 로 to_string 의 빠진 case 를 잡도록,
// 그리고 이 테스트가 함께 갱신되도록 (전수 검증 계약 명시).
void test_distinctness()
{
  using aidin_hand2::ErrorCode;
  // 대표적으로 ErrorCode 8 값이 서로 다른 문자열임을 교차 확인.
  const char* names[] = {
      to_string(ErrorCode::None),
      to_string(ErrorCode::InvalidArgument),
      to_string(ErrorCode::WrongCallOrder),
      to_string(ErrorCode::InterfaceUnavailable),
      to_string(ErrorCode::CommunicationLost),
      to_string(ErrorCode::HardwareFault),
      to_string(ErrorCode::ControlLoopFault),
      to_string(ErrorCode::UnexpectedError),
  };
  constexpr int kN = sizeof(names) / sizeof(names[0]);
  for (int i = 0; i < kN; ++i) {
    check(names[i] != nullptr && names[i][0] != '\0', "ErrorCode 이름 비어있지 않음");
    for (int j = i + 1; j < kN; ++j) {
      check(!str_eq(names[i], names[j]), "ErrorCode 이름이 서로 구별됨");
    }
  }
}

}  // namespace

int main()
{
  test_error_code();
  test_homing_state();
  test_actuator_fault();
  test_hand_lifecycle();
  test_to_command_mode();
  test_distinctness();
  if (g_failures == 0) {
    std::printf("to_string_test: PASS\n");
  }
  return g_failures;
}
