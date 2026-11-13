// 목적: lifecycle 전이 표(check_action_allowed)와 원인 렌더 헬퍼를 검증.
//   (1) destroyed gate — 모든 action 거부(WrongCallOrder)
//   (2) 전이 표 — (HandAction × HandLifecycle) 전 셀의 허용/거부 + 거부 시 ErrorKind
//   (3) Faulted 거부 — 원인과 상관없이 E601, 원인 번호는 stop_cause_code(E404 / E405 / E702, 기록 없음 0)
//   (4) SetCommand homed gate — 미원점 거부, 원점 통과
//   (5) 원인 문구 — stop_cause_phrase / receive_anomaly_suffix / transmit_errno_suffix
//   (6) 요구/사실 분리 — 판정은 요구 기준, 관측 Faulted 가 요구를 지배
//   (7) 거부 문구 — 원인만, 조치 안내 없이 정확한 문장과 번호
// 방법: (1)~(6)은 공개 계약(Status.ok()/kind, 문구의 의미 있는 substring)만 assert. 전이 표는
//       소스(action_check.cpp)의 셀 전수와 1:1. (7)은 문서가 인용하는 문장이라 정확히 비교한다.

#include <cstdio>
#include <string>

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

// 기본 원인 = 루프 예외. Faulted 거부는 원인과 상관없이 HandFaulted. 통신 원인은 개별 케이스에서 지정.
StopCause loop_cause() { return StopCause{RealtimeEventKind::ControlLoopFailure, 0, 0}; }

// 한 셀 판정: 허용 여부 + (거부면) ErrorKind 까지.
void cell(HandAction a, HandLifecycle lc, const StopCause& cause, bool homed,
          bool want_ok, ErrorKind want_kind, const char* tag)
{
  // 정착 상태는 요구와 사실이 같고, Faulted 는 Running 을 요구하던 중 관측된 사실이다.
  const HandLifecycle requested =
      lc == HandLifecycle::Faulted ? HandLifecycle::Running : lc;
  const Status s = check_action_allowed(a, requested, lc, /*destroyed=*/false, cause, homed);
  check(s.ok() == want_ok, tag);
  if (!want_ok) check(s.kind() == want_kind, tag);
}

void test_destroyed_gate()
{
  const HandAction actions[] = {HandAction::Connect,  HandAction::Disconnect, HandAction::Run,
                                HandAction::Stop,      HandAction::Home,       HandAction::Reconnect,
                                HandAction::SetCommand, HandAction::SetTactileBias,
                                HandAction::ResetTactileBias};
  for (HandAction a : actions) {
    // lifecycle/homed 와 무관하게 파기면 WrongCallOrder 거부.
    const Status s = check_action_allowed(a, HandLifecycle::Connected, HandLifecycle::Connected,
                                          /*destroyed=*/true, loop_cause(), /*homed=*/true);
    check(!s.ok() && s.kind() == ErrorKind::WrongCallOrder, "destroyed → 모든 action 거부(WrongCallOrder)");
  }
}

void test_transition_table()
{
  const StopCause lf = loop_cause();  // Faulted → HandFaulted
  const bool H = true;                // homed (SetCommand 외엔 무관)

  // ── Connect: Disconnected 허용, Connected 는 postcondition 을 이미 만족해 no-op ──
  cell(HandAction::Connect, HandLifecycle::Disconnected, lf, H, true,  ErrorKind::None, "Connect@Disconnected 허용");
  cell(HandAction::Connect, HandLifecycle::Connected,    lf, H, true,  ErrorKind::None, "Connect@Connected 허용(no-op은 메서드 몫)");
  cell(HandAction::Connect, HandLifecycle::Running,      lf, H, false, ErrorKind::WrongCallOrder, "Connect@Running 거부");
  cell(HandAction::Connect, HandLifecycle::Stopped,      lf, H, false, ErrorKind::WrongCallOrder, "Connect@Stopped 거부");
  cell(HandAction::Connect, HandLifecycle::Faulted,      lf, H, false, ErrorKind::HandFaulted, "Connect@Faulted 거부(HandFaulted)");

  // ── Disconnect: Faulted 외 전부 허용, Disconnected 는 no-op ──
  cell(HandAction::Disconnect, HandLifecycle::Disconnected, lf, H, true,  ErrorKind::None, "Disconnect@Disconnected 허용(no-op은 메서드 몫)");
  cell(HandAction::Disconnect, HandLifecycle::Connected,    lf, H, true,  ErrorKind::None, "Disconnect@Connected 허용");
  cell(HandAction::Disconnect, HandLifecycle::Running,      lf, H, true,  ErrorKind::None, "Disconnect@Running 허용(확인은 메서드 몫)");
  cell(HandAction::Disconnect, HandLifecycle::Stopped,      lf, H, true,  ErrorKind::None, "Disconnect@Stopped 허용");
  cell(HandAction::Disconnect, HandLifecycle::Faulted,      lf, H, false, ErrorKind::HandFaulted, "Disconnect@Faulted 거부(HandFaulted)");

  // ── Run: Connected/Running/Stopped 허용 ──
  cell(HandAction::Run, HandLifecycle::Disconnected, lf, H, false, ErrorKind::WrongCallOrder, "Run@Disconnected 거부");
  cell(HandAction::Run, HandLifecycle::Connected,    lf, H, true,  ErrorKind::None, "Run@Connected 허용");
  cell(HandAction::Run, HandLifecycle::Running,      lf, H, true,  ErrorKind::None, "Run@Running 허용(no-op은 메서드 몫)");
  cell(HandAction::Run, HandLifecycle::Stopped,      lf, H, true,  ErrorKind::None, "Run@Stopped 허용");
  cell(HandAction::Run, HandLifecycle::Faulted,      lf, H, false, ErrorKind::HandFaulted, "Run@Faulted 거부(HandFaulted)");

  // ── Stop: Running/Stopped 허용 · Connected 거부 ──
  cell(HandAction::Stop, HandLifecycle::Disconnected, lf, H, false, ErrorKind::WrongCallOrder, "Stop@Disconnected 거부");
  cell(HandAction::Stop, HandLifecycle::Connected,    lf, H, false, ErrorKind::WrongCallOrder, "Stop@Connected 거부(미실행)");
  cell(HandAction::Stop, HandLifecycle::Running,      lf, H, true,  ErrorKind::None, "Stop@Running 허용");
  cell(HandAction::Stop, HandLifecycle::Stopped,      lf, H, true,  ErrorKind::None, "Stop@Stopped 허용(no-op은 메서드 몫)");
  cell(HandAction::Stop, HandLifecycle::Faulted,      lf, H, false, ErrorKind::HandFaulted, "Stop@Faulted 거부(HandFaulted)");

  // ── Home: Connected/Running/Stopped 허용 ──
  cell(HandAction::Home, HandLifecycle::Disconnected, lf, H, false, ErrorKind::WrongCallOrder, "Home@Disconnected 거부");
  cell(HandAction::Home, HandLifecycle::Connected,    lf, H, true,  ErrorKind::None, "Home@Connected 허용");
  cell(HandAction::Home, HandLifecycle::Running,      lf, H, true,  ErrorKind::None, "Home@Running 허용");
  cell(HandAction::Home, HandLifecycle::Stopped,      lf, H, true,  ErrorKind::None, "Home@Stopped 허용");
  cell(HandAction::Home, HandLifecycle::Faulted,      lf, H, false, ErrorKind::HandFaulted, "Home@Faulted 거부(HandFaulted)");

  // ── Reconnect: Faulted 만 허용 (유일 복구 경로) ──
  cell(HandAction::Reconnect, HandLifecycle::Disconnected, lf, H, false, ErrorKind::WrongCallOrder, "Reconnect@Disconnected 거부");
  cell(HandAction::Reconnect, HandLifecycle::Connected,    lf, H, false, ErrorKind::WrongCallOrder, "Reconnect@Connected 거부(fault 아님)");
  cell(HandAction::Reconnect, HandLifecycle::Running,      lf, H, false, ErrorKind::WrongCallOrder, "Reconnect@Running 거부(fault 아님)");
  cell(HandAction::Reconnect, HandLifecycle::Stopped,      lf, H, false, ErrorKind::WrongCallOrder, "Reconnect@Stopped 거부(fault 아님)");
  cell(HandAction::Reconnect, HandLifecycle::Faulted,      lf, H, true,  ErrorKind::None, "Reconnect@Faulted 허용");

  // ── SetCommand(homed=true): Running 만 허용 ──
  cell(HandAction::SetCommand, HandLifecycle::Disconnected, lf, H, false, ErrorKind::WrongCallOrder, "SetCommand@Disconnected 거부");
  cell(HandAction::SetCommand, HandLifecycle::Connected,    lf, H, false, ErrorKind::WrongCallOrder, "SetCommand@Connected 거부(TX 없음)");
  cell(HandAction::SetCommand, HandLifecycle::Running,      lf, H, true,  ErrorKind::None, "SetCommand@Running 허용");
  cell(HandAction::SetCommand, HandLifecycle::Stopped,      lf, H, false, ErrorKind::WrongCallOrder, "SetCommand@Stopped 거부(run 먼저)");
  cell(HandAction::SetCommand, HandLifecycle::Faulted,      lf, H, false, ErrorKind::HandFaulted, "SetCommand@Faulted 거부(HandFaulted)");

  // ── SetTactileBias / ResetTactileBias: RT 루프가 도는 Connected/Running/Stopped 허용 ──
  for (HandAction a : {HandAction::SetTactileBias, HandAction::ResetTactileBias}) {
    cell(a, HandLifecycle::Disconnected, lf, H, false, ErrorKind::WrongCallOrder, "TactileBias@Disconnected 거부");
    cell(a, HandLifecycle::Connected,    lf, H, true,  ErrorKind::None, "TactileBias@Connected 허용(토크 불필요)");
    cell(a, HandLifecycle::Running,      lf, H, true,  ErrorKind::None, "TactileBias@Running 허용");
    cell(a, HandLifecycle::Stopped,      lf, H, true,  ErrorKind::None, "TactileBias@Stopped 허용");
    cell(a, HandLifecycle::Faulted,      lf, H, false, ErrorKind::HandFaulted, "TactileBias@Faulted 거부(HandFaulted)");
  }
}

void test_faulted_rejection_and_cause_code()
{
  // Faulted 거부의 번호는 원인과 상관없이 E601 이다. Run@Faulted 로 관찰.
  auto rejection_for = [](RealtimeEventKind reason) {
    return check_action_allowed(HandAction::Run, HandLifecycle::Running, HandLifecycle::Faulted,
                                false, StopCause{reason, 0, 0}, true).code;
  };
  check(rejection_for(RealtimeEventKind::ReceiveSilence) == 601, "Faulted 거부: ReceiveSilence → E601");
  check(rejection_for(RealtimeEventKind::TransmitFailure) == 601, "Faulted 거부: TransmitFailure → E601");
  check(rejection_for(RealtimeEventKind::ControlLoopFailure) == 601, "Faulted 거부: ControlLoopFailure → E601");
  check(rejection_for(RealtimeEventKind::HomingFailed) == 601, "Faulted 거부: 기록 없는 원인 → E601");

  // 원인 번호는 RT log 가 같은 원인에 붙이는 번호와 같고, Diagnostics::fault_code 가 이 값을 쓴다.
  auto code_for = [](RealtimeEventKind reason) { return stop_cause_code(StopCause{reason, 0, 0}); };
  check(code_for(RealtimeEventKind::ReceiveSilence) == 404, "원인 번호: ReceiveSilence → 404");
  check(code_for(RealtimeEventKind::TransmitFailure) == 405, "원인 번호: TransmitFailure → 405");
  check(code_for(RealtimeEventKind::ControlLoopFailure) == 702, "원인 번호: ControlLoopFailure → 702");
  check(code_for(RealtimeEventKind::HomingFailed) == 0, "원인 번호: 그 외 → 0");
}

void test_setcommand_homed_gate()
{
  const StopCause lf = loop_cause();
  // 미원점 → 거부(WrongCallOrder), 원점 → 허용. (Faulted 아닌 상태에서)
  cell(HandAction::SetCommand, HandLifecycle::Running, lf, /*homed=*/false, false,
       ErrorKind::WrongCallOrder, "SetCommand 미원점 거부");
  cell(HandAction::SetCommand, HandLifecycle::Running, lf, /*homed=*/true, true,
       ErrorKind::None, "SetCommand 원점 허용");
  // Faulted 는 homed 여부와 무관하게 먼저 거부(부검만 허용).
  cell(HandAction::SetCommand, HandLifecycle::Faulted, lf, /*homed=*/false, false,
       ErrorKind::HandFaulted, "SetCommand@Faulted 는 homed 이전에 거부");
}

void test_cause_phrases()
{
  // stop_cause_phrase — "E<번호> <사건>". 사건은 RT log 줄과 같은 말이고, 세부(errno·수신 이상)는 log 줄에만.
  check(stop_cause_phrase(StopCause{RealtimeEventKind::ReceiveSilence, 528594,
                                    static_cast<int>(canfd::DecodeResult::ConflictingCommand)}) ==
            "E404 communication lost", "phrase: ReceiveSilence");
  check(stop_cause_phrase(StopCause{RealtimeEventKind::TransmitFailure, 42, 6}) == "E405 CAN TX failed",
        "phrase: TransmitFailure");
  check(stop_cause_phrase(StopCause{RealtimeEventKind::ControlLoopFailure, 7, 0}) ==
            "E702 control loop terminated", "phrase: ControlLoopFailure");
  check(stop_cause_phrase(StopCause{RealtimeEventKind::HomingFailed, 0, 0}).empty(),
        "phrase: 기록 없는 원인 → 빈 문자열");

  // transmit_errno_suffix — 0/음수 생략, 양수는 errno 포함.
  check(transmit_errno_suffix(0).empty(), "errno suffix: 0 → 빈 문자열");
  check(contains(transmit_errno_suffix(6), "errno 6"), "errno suffix: 6 포함");

  // receive_anomaly_suffix — 다른 master 의 command 만 덧붙는다. 그 밖의 수신 이상은 원인을 특정하지 못한다.
  using canfd::DecodeResult;
  check(receive_anomaly_suffix(static_cast<int>(DecodeResult::Decoded)).empty(),
        "anomaly: Decoded → 빈 문자열");
  check(receive_anomaly_suffix(static_cast<int>(DecodeResult::ConflictingCommand)) ==
            ", last RX error: conflicting command",
        "anomaly: ConflictingCommand");
  check(receive_anomaly_suffix(static_cast<int>(DecodeResult::LengthError)).empty(),
        "anomaly: LengthError → 빈 문자열");
  check(receive_anomaly_suffix(static_cast<int>(DecodeResult::Unknown)).empty(),
        "anomaly: Unknown → 빈 문자열");
  check(receive_anomaly_suffix(static_cast<int>(DecodeResult::Error)).empty(),
        "anomaly: Error → 빈 문자열");
}

// 거부 문구 — 원인만 담고 조치 안내(" — ...")는 붙지 않는다.
void test_rejection_messages()
{
  const StopCause lf = loop_cause();
  auto status = [&lf](HandAction a, HandLifecycle requested, HandLifecycle observed, bool homed = true) {
    return check_action_allowed(a, requested, observed, /*destroyed=*/false, lf, homed);
  };
  auto expect = [](const Status& got, unsigned code, const char* message, const char* what) {
    if (got.code != code || got.message != message) {
      std::printf("FAIL: %s\n  got  E%u %s\n  want E%u %s\n", what, unsigned{got.code},
                  got.message.c_str(), code, message);
      ++g_failures;
    }
  };
  using L = HandLifecycle;
  expect(status(HandAction::Connect, L::Running, L::Running), 201, "cannot connect hand: already connected",
         "Connect@Running");
  expect(status(HandAction::Run, L::Disconnected, L::Disconnected), 202, "cannot run hand: not connected",
         "Run@Disconnected");
  expect(status(HandAction::SetCommand, L::Disconnected, L::Disconnected), 202,
         "cannot set command: not connected", "SetCommand@Disconnected");
  expect(status(HandAction::Reconnect, L::Disconnected, L::Disconnected), 202,
         "cannot reconnect hand: not connected", "Reconnect@Disconnected");
  expect(status(HandAction::Stop, L::Connected, L::Connected), 204, "cannot stop hand: not running",
         "Stop@Connected");
  expect(status(HandAction::SetCommand, L::Connected, L::Connected), 204, "cannot set command: not running",
         "SetCommand@Connected");
  expect(status(HandAction::SetCommand, L::Running, L::Running, /*homed=*/false), 205,
         "cannot set command: not homed", "SetCommand unhomed");
  expect(status(HandAction::Reconnect, L::Running, L::Running), 206, "cannot reconnect hand: not faulted",
         "Reconnect@Running");
  expect(check_action_allowed(HandAction::Connect, L::Connected, L::Connected, /*destroyed=*/true, lf, true),
         207, "cannot use hand: hand is destroyed", "destroyed");
  expect(check_action_allowed(HandAction::Run, L::Running, L::Faulted, false,
                              StopCause{RealtimeEventKind::ReceiveSilence, 5, 0}, true),
         601, "cannot run hand: hand faulted (E404 communication lost)", "Run@Faulted");
  expect(check_action_allowed(HandAction::Run, L::Running, L::Faulted, false,
                              StopCause{RealtimeEventKind::HomingFailed, 0, 0}, true),
         601, "cannot run hand: hand faulted", "Run@Faulted, cause not recorded");
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
  test_faulted_rejection_and_cause_code();
  test_setcommand_homed_gate();
  test_cause_phrases();
  test_requested_versus_observed();
  test_rejection_messages();
  if (g_failures == 0) {
    std::printf("action_check_test: PASS\n");
  }
  return g_failures;
}
