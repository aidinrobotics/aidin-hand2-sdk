// 목적: lifecycle 전이 표(check_action_allowed)와 원인 렌더 헬퍼를 검증.
//   (1) destroyed gate — 모든 action 거부(WrongCallOrder)
//   (2) 전이 표 — (HandAction × HandLifecycle) 전 셀의 허용/거부 + 거부 시 ErrorCode
//   (3) Faulted 거부 ErrorCode 파생 — 통신 사건 → CommunicationLost, 그 외 → ControlLoopFault
//   (4) SetCommand homed gate — 미원점 거부, 원점 통과
//   (5) 원인 문구 — stop_cause_phrase / receive_anomaly_suffix / transmit_errno_suffix
//   (6) 요구/사실 분리 — 판정은 요구 기준, 관측 Faulted 가 요구를 지배
// 방법: 공개 계약(Status.ok()/code, 문구의 의미 있는 substring)만 assert — 내부 구현/정확한 문장에
//       비의존(리팩터 안전). 전이 표는 소스(action_check.cpp)의 셀 전수와 1:1.

#include <cstdio>
#include <string>

#include <linux/can/error.h>  // CAN_ERR_* class 비트

#include "hand_core/comms/canfd/protocol.hpp"   // canfd::DecodeResult
#include "hand_core/lifecycle/action_check.hpp"

namespace
{
using namespace aidin_hand2;

int g_failures = 0;
void check(bool ok, const char* what)
{
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    ++g_failures;
  }
}

bool contains(const std::string& s, const char* sub) { return s.find(sub) != std::string::npos; }

// 기본 원인 = 루프 예외(파생 시 ControlLoopFault). 통신 원인은 개별 케이스에서 지정.
StopCause loop_cause() { return StopCause{RealtimeEventKind::ControlLoopFailure, 0, 0, 0}; }

// 한 셀 판정: 허용 여부 + (거부면) ErrorCode 까지.
void cell(HandAction a, HandLifecycle lc, const StopCause& cause, bool homed,
          bool want_ok, ErrorCode want_code, const char* tag)
{
  // 정착 상태는 요구와 사실이 같고, Faulted 는 Running 을 요구하던 중 관측된 사실이다.
  const HandLifecycle requested =
      lc == HandLifecycle::Faulted ? HandLifecycle::Running : lc;
  const Status s = check_action_allowed(a, requested, lc, /*destroyed=*/false, cause, homed);
  check(s.ok() == want_ok, tag);
  if (!want_ok) check(s.code == want_code, tag);
}

void test_destroyed_gate()
{
  const HandAction actions[] = {HandAction::Connect,  HandAction::Disconnect, HandAction::Run,
                                HandAction::Stop,      HandAction::Home,       HandAction::Reconnect,
                                HandAction::SetCommand};
  for (HandAction a : actions) {
    // lifecycle/homed 와 무관하게 파기면 WrongCallOrder 거부.
    const Status s = check_action_allowed(a, HandLifecycle::Connected, HandLifecycle::Connected,
                                          /*destroyed=*/true, loop_cause(), /*homed=*/true);
    check(!s.ok() && s.code == ErrorCode::WrongCallOrder, "destroyed → 모든 action 거부(WrongCallOrder)");
  }
}

void test_transition_table()
{
  const StopCause lf = loop_cause();  // Faulted 파생 → ControlLoopFault
  const bool H = true;                // homed (SetCommand 외엔 무관)

  // ── Connect: Disconnected 허용, Connected 는 postcondition 을 이미 만족해 no-op ──
  cell(HandAction::Connect, HandLifecycle::Disconnected, lf, H, true,  ErrorCode::None, "Connect@Disconnected 허용");
  cell(HandAction::Connect, HandLifecycle::Connected,    lf, H, true,  ErrorCode::None, "Connect@Connected 허용(no-op은 메서드 몫)");
  cell(HandAction::Connect, HandLifecycle::Running,      lf, H, false, ErrorCode::WrongCallOrder, "Connect@Running 거부");
  cell(HandAction::Connect, HandLifecycle::Stopped,      lf, H, false, ErrorCode::WrongCallOrder, "Connect@Stopped 거부");
  cell(HandAction::Connect, HandLifecycle::Faulted,      lf, H, false, ErrorCode::ControlLoopFault, "Connect@Faulted 거부(파생)");

  // ── Disconnect: Faulted 외 전부 허용, Disconnected 는 no-op ──
  cell(HandAction::Disconnect, HandLifecycle::Disconnected, lf, H, true,  ErrorCode::None, "Disconnect@Disconnected 허용(no-op은 메서드 몫)");
  cell(HandAction::Disconnect, HandLifecycle::Connected,    lf, H, true,  ErrorCode::None, "Disconnect@Connected 허용");
  cell(HandAction::Disconnect, HandLifecycle::Running,      lf, H, true,  ErrorCode::None, "Disconnect@Running 허용(확인은 메서드 몫)");
  cell(HandAction::Disconnect, HandLifecycle::Stopped,      lf, H, true,  ErrorCode::None, "Disconnect@Stopped 허용");
  cell(HandAction::Disconnect, HandLifecycle::Faulted,      lf, H, false, ErrorCode::ControlLoopFault, "Disconnect@Faulted 거부(파생)");

  // ── Run: Connected/Running/Stopped 허용 ──
  cell(HandAction::Run, HandLifecycle::Disconnected, lf, H, false, ErrorCode::WrongCallOrder, "Run@Disconnected 거부");
  cell(HandAction::Run, HandLifecycle::Connected,    lf, H, true,  ErrorCode::None, "Run@Connected 허용");
  cell(HandAction::Run, HandLifecycle::Running,      lf, H, true,  ErrorCode::None, "Run@Running 허용(no-op은 메서드 몫)");
  cell(HandAction::Run, HandLifecycle::Stopped,      lf, H, true,  ErrorCode::None, "Run@Stopped 허용");
  cell(HandAction::Run, HandLifecycle::Faulted,      lf, H, false, ErrorCode::ControlLoopFault, "Run@Faulted 거부(파생)");

  // ── Stop: Running/Stopped 허용 · Connected 거부 ──
  cell(HandAction::Stop, HandLifecycle::Disconnected, lf, H, false, ErrorCode::WrongCallOrder, "Stop@Disconnected 거부");
  cell(HandAction::Stop, HandLifecycle::Connected,    lf, H, false, ErrorCode::WrongCallOrder, "Stop@Connected 거부(미실행)");
  cell(HandAction::Stop, HandLifecycle::Running,      lf, H, true,  ErrorCode::None, "Stop@Running 허용");
  cell(HandAction::Stop, HandLifecycle::Stopped,      lf, H, true,  ErrorCode::None, "Stop@Stopped 허용(no-op은 메서드 몫)");
  cell(HandAction::Stop, HandLifecycle::Faulted,      lf, H, false, ErrorCode::ControlLoopFault, "Stop@Faulted 거부(파생)");

  // ── Home: Connected/Running/Stopped 허용 ──
  cell(HandAction::Home, HandLifecycle::Disconnected, lf, H, false, ErrorCode::WrongCallOrder, "Home@Disconnected 거부");
  cell(HandAction::Home, HandLifecycle::Connected,    lf, H, true,  ErrorCode::None, "Home@Connected 허용");
  cell(HandAction::Home, HandLifecycle::Running,      lf, H, true,  ErrorCode::None, "Home@Running 허용");
  cell(HandAction::Home, HandLifecycle::Stopped,      lf, H, true,  ErrorCode::None, "Home@Stopped 허용");
  cell(HandAction::Home, HandLifecycle::Faulted,      lf, H, false, ErrorCode::ControlLoopFault, "Home@Faulted 거부(파생)");

  // ── Reconnect: Faulted 만 허용 (유일 복구 경로) ──
  cell(HandAction::Reconnect, HandLifecycle::Disconnected, lf, H, false, ErrorCode::WrongCallOrder, "Reconnect@Disconnected 거부");
  cell(HandAction::Reconnect, HandLifecycle::Connected,    lf, H, false, ErrorCode::WrongCallOrder, "Reconnect@Connected 거부(fault 아님)");
  cell(HandAction::Reconnect, HandLifecycle::Running,      lf, H, false, ErrorCode::WrongCallOrder, "Reconnect@Running 거부(fault 아님)");
  cell(HandAction::Reconnect, HandLifecycle::Stopped,      lf, H, false, ErrorCode::WrongCallOrder, "Reconnect@Stopped 거부(fault 아님)");
  cell(HandAction::Reconnect, HandLifecycle::Faulted,      lf, H, true,  ErrorCode::None, "Reconnect@Faulted 허용");

  // ── SetCommand(homed=true): Running 만 허용 ──
  cell(HandAction::SetCommand, HandLifecycle::Disconnected, lf, H, false, ErrorCode::WrongCallOrder, "SetCommand@Disconnected 거부");
  cell(HandAction::SetCommand, HandLifecycle::Connected,    lf, H, false, ErrorCode::WrongCallOrder, "SetCommand@Connected 거부(TX 없음)");
  cell(HandAction::SetCommand, HandLifecycle::Running,      lf, H, true,  ErrorCode::None, "SetCommand@Running 허용");
  cell(HandAction::SetCommand, HandLifecycle::Stopped,      lf, H, false, ErrorCode::WrongCallOrder, "SetCommand@Stopped 거부(run 먼저)");
  cell(HandAction::SetCommand, HandLifecycle::Faulted,      lf, H, false, ErrorCode::ControlLoopFault, "SetCommand@Faulted 거부(파생)");
}

void test_faulted_error_code_derivation()
{
  // Faulted 거부 시 ErrorCode 는 stop_cause.reason 에서 파생. Run@Faulted 로 관찰.
  auto code_for = [](RealtimeEventKind reason) {
    return check_action_allowed(HandAction::Run, HandLifecycle::Running, HandLifecycle::Faulted,
                                false, StopCause{reason, 0, 0, 0}, true).code;
  };
  check(code_for(RealtimeEventKind::ReceiveSilence) == ErrorCode::CommunicationLost,
        "Faulted 파생: ReceiveSilence → CommunicationLost");
  check(code_for(RealtimeEventKind::TransmitFailure) == ErrorCode::CommunicationLost,
        "Faulted 파생: TransmitFailure → CommunicationLost");
  check(code_for(RealtimeEventKind::ControlLoopFailure) == ErrorCode::ControlLoopFault,
        "Faulted 파생: ControlLoopFailure → ControlLoopFault");
  check(code_for(RealtimeEventKind::StopConfirmed) == ErrorCode::ControlLoopFault,
        "Faulted 파생: 그 외 → ControlLoopFault");
}

void test_setcommand_homed_gate()
{
  const StopCause lf = loop_cause();
  // 미원점 → 거부(WrongCallOrder), 원점 → 허용. (Faulted 아닌 상태에서)
  cell(HandAction::SetCommand, HandLifecycle::Running, lf, /*homed=*/false, false,
       ErrorCode::WrongCallOrder, "SetCommand 미원점 거부");
  cell(HandAction::SetCommand, HandLifecycle::Running, lf, /*homed=*/true, true,
       ErrorCode::None, "SetCommand 원점 허용");
  // Faulted 는 homed 여부와 무관하게 먼저 거부(부검만 허용).
  cell(HandAction::SetCommand, HandLifecycle::Faulted, lf, /*homed=*/false, false,
       ErrorCode::ControlLoopFault, "SetCommand@Faulted 는 homed 이전에 거부");
}

void test_cause_phrases()
{
  // stop_cause_phrase — reason 별 의미 substring + cycle 번호 포함.
  check(contains(stop_cause_phrase(StopCause{RealtimeEventKind::ReceiveSilence, 528594, 0, 0}),
                 "no RX since cycle 528594"), "phrase: ReceiveSilence");
  check(contains(stop_cause_phrase(StopCause{RealtimeEventKind::TransmitFailure, 42, 0, 0}),
                 "CAN TX failed"), "phrase: TransmitFailure");
  check(contains(stop_cause_phrase(StopCause{RealtimeEventKind::ControlLoopFailure, 7, 0, 0}),
                 "control loop terminated"), "phrase: ControlLoopFailure");
  check(contains(stop_cause_phrase(StopCause{RealtimeEventKind::StopConfirmed, 0, 0, 0}),
                 "cause not recorded"), "phrase: 그 외 → not recorded");

  // transmit_errno_suffix — 0/음수 생략, 양수는 errno 포함.
  check(transmit_errno_suffix(0).empty(), "errno suffix: 0 → 빈 문자열");
  check(contains(transmit_errno_suffix(6), "errno 6"), "errno suffix: 6 포함");

  // receive_anomaly_suffix — DecodeResult 별 분기 + error frame class 세분.
  using canfd::DecodeResult;
  check(receive_anomaly_suffix(static_cast<int>(DecodeResult::Decoded), 0).empty(),
        "anomaly: Decoded → 빈 문자열");
  check(contains(receive_anomaly_suffix(static_cast<int>(DecodeResult::LengthError), 0), "malformed"),
        "anomaly: LengthError → malformed");
  check(contains(receive_anomaly_suffix(static_cast<int>(DecodeResult::Unknown), 0), "unknown"),
        "anomaly: Unknown");
  check(contains(receive_anomaly_suffix(static_cast<int>(DecodeResult::ConflictingCommand), 0), "conflicting"),
        "anomaly: ConflictingCommand");
  check(contains(receive_anomaly_suffix(static_cast<int>(DecodeResult::Error), CAN_ERR_BUSOFF), "bus off"),
        "anomaly: Error + BUSOFF → bus off");
  check(contains(receive_anomaly_suffix(static_cast<int>(DecodeResult::Error), CAN_ERR_ACK), "no ACK"),
        "anomaly: Error + ACK → no ACK");
  check(contains(receive_anomaly_suffix(static_cast<int>(DecodeResult::Error), 0), "bus error"),
        "anomaly: Error + class 0 → bus error");
}

// 요구와 사실이 어긋난 구간: 정지를 요구했지만 drive 가 확인 전(관측 Running)인 상태.
void test_requested_versus_observed()
{
  const StopCause lf = loop_cause();
  auto allowed = [&lf](HandAction a, HandLifecycle requested, HandLifecycle observed) {
    return check_action_allowed(a, requested, observed, /*destroyed=*/false, lf, /*homed=*/true).ok();
  };

  // 미확인 정지: 판정은 요구 기준이므로 command 는 거부되고, stop() 재호출은 통해야 한다.
  check(!allowed(HandAction::SetCommand, HandLifecycle::Stopped, HandLifecycle::Running),
        "요구 Stopped·관측 Running → SetCommand 거부");
  check(allowed(HandAction::Stop, HandLifecycle::Stopped, HandLifecycle::Running),
        "요구 Stopped·관측 Running → stop() 재호출 허용(재확인)");

  // enable 확인 전: 요구가 Running 이어도 command 는 거부된다 — 확인된 enable 만 command 를 싣는다.
  check(!allowed(HandAction::SetCommand, HandLifecycle::Running, HandLifecycle::Connected),
        "요구 Running·관측 Connected → SetCommand 거부(enable 미확인)");
  check(allowed(HandAction::SetCommand, HandLifecycle::Running, HandLifecycle::Running),
        "요구·관측 모두 Running → SetCommand 허용");

  // 관측 Faulted 는 요구와 무관하게 지배하고, Faulted 를 나열한 action 만 통과한다.
  check(allowed(HandAction::Reconnect, HandLifecycle::Running, HandLifecycle::Faulted),
        "관측 Faulted → reconnect() 허용");
  check(!allowed(HandAction::Run, HandLifecycle::Running, HandLifecycle::Faulted),
        "관측 Faulted → run() 거부");
}

}  // namespace

int main()
{
  test_destroyed_gate();
  test_transition_table();
  test_faulted_error_code_derivation();
  test_setcommand_homed_gate();
  test_cause_phrases();
  test_requested_versus_observed();
  if (g_failures == 0) {
    std::printf("action_check_test: PASS\n");
  }
  return g_failures;
}
