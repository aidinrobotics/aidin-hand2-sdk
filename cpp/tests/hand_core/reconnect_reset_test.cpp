// 목적(Tier 1, HW 무관): Running 을 벗어난 뒤 저장된 command 가 남지 않는지, 그리고 수동
// reconnect() 가 자동 재연결과 같은 복구 상태를 만드는지 점검한다. set_command() 는 Running 에서만
// 받으므로 Running 밖의 command 는 무효이고, 남아 있으면 다시 Running 이 된 첫 cycle 에 송신된다.
//   (a) reconnect(): fault 직전 command 가 남아 있으면 reconnect() 뒤 run() 을 부른
//       첫 cycle 에 그것이 그대로 송신된다. enable_control() 은 Stopped 에서 올 때만 command 를
//       비우고, reconnect() 는 Connected 로 끝나므로 그 분기를 타지 않는다. 자동 재연결은
//       try_reconnect_once() 에서 reset_command_to_idle() 을 부른다 — 수동 경로도 같아야 한다.
//   (b) disconnect(): Running 을 떠나므로 저장된 command 를 비운다. quick stop 확인이 실패해
//       예외로 끝나는 경로에서도 남아야 하므로, 확인을 기다리기 전에 비운다.
//   (c) connect(): RealtimeBuffer 는 값을 한 번만 건네고 RT 스레드는 connect() 마다 새로 뜨므로,
//       마지막 ControllerConfig 를 다시 싣지 않으면 재연결 뒤 기본값으로 돌아간다.
//   (d) 멈춘 원인 보존: reset_fault_state() 가 last_stop_cause_ 를 비운 뒤 connect() 가 실패하면
//       lifecycle 은 Faulted 로 돌아간다. RealtimeEventKind 의 첫 값이 ReceiveSilence 이므로
//       기본값 StopCause 는 "cause not recorded" 가 아니라 "no RX since cycle 0" 으로 렌더된다 —
//       일어나지 않은 사건이 보고된다. 실패 시 초기화 전 값을 되돌려야 한다.
//
// 소켓을 열지 않는다. 존재하지 않는 interface 이름을 써서 connect() 가 반드시 실패하게 하고,
// lifecycle 과 원인은 test peer 로 직접 세운다.

#include <cstdio>
#include <string>
#include <variant>

#include <aidin_hand2/types/command.hpp>
#include <aidin_hand2/types/config.hpp>

#include "hand_core/hand_core.hpp"
#include "hand_core/lifecycle/action_check.hpp"

using namespace aidin_hand2;

namespace aidin_hand2::test
{
struct HandCoreTestPeer {
  static void force_faulted(HandCore& core, const StopCause& cause)
  {
    core.last_stop_cause_ = cause;
    core.lifecycle_.store(HandLifecycle::Faulted);
  }
  // disconnect() 가 quick stop 확인 분기를 타려면 요청과 관측이 모두 Running 이어야 한다
  static void force_running(HandCore& core)
  {
    core.requested_lifecycle_.store(HandLifecycle::Running);
    core.lifecycle_.store(HandLifecycle::Running);
  }
  static void stage_position_command(HandCore& core)
  {
    ActuatorPositionCommand command{};
    command.target.fill(0.5);
    core.staging_command_.controller = command;
    core.command_buffer_.write(core.staging_command_);
  }
  static const HandCommand& staged(HandCore& core) { return core.staging_command_; }
  static void republish(HandCore& core) { core.republish_staged(); }
  static bool read_config(HandCore& core, ControllerConfig& out)
  {
    return core.controller_config_buffer_.read(out);
  }
  static StopCause stop_cause(HandCore& core) { return core.last_stop_cause_; }
  static HandLifecycle lifecycle(HandCore& core) { return core.lifecycle_.load(); }
};
}  // namespace aidin_hand2::test

namespace
{
using test::HandCoreTestPeer;

int g_failures = 0;

void check(bool ok, const char* what)
{
  if (!ok) { std::printf("  FAIL: %s\n", what); ++g_failures; }
}

// 열리지 않는 이름이어야 한다 — connect() 가 성공하면 이 테스트의 전제가 무너진다.
HandConfig make_config()
{
  HandConfig config{"ah2_no_such_iface", HandSide::Right};
  config.control_rate = 500;
  config.auto_reconnect = false;
  return config;
}

// (a) fault 직전 command 가 reconnect() 뒤에 남아 있지 않다.
void test_reconnect_clears_staged_command()
{
  HandCore core{make_config()};
  HandCoreTestPeer::stage_position_command(core);
  check(std::holds_alternative<ActuatorPositionCommand>(HandCoreTestPeer::staged(core).controller),
        "전제: position command 가 저장돼 있다");

  HandCoreTestPeer::force_faulted(core, StopCause{RealtimeEventKind::ReceiveSilence, 4210, 0, 0});
  const Status result = core.reconnect();

  check(!result.ok(), "전제: 없는 interface 라 reconnect() 는 실패한다");
  check(std::holds_alternative<Idle>(HandCoreTestPeer::staged(core).controller),
        "reconnect() 는 저장된 command 를 Idle 로 되돌린다");
}

// (b) Running 에서 부른 disconnect() 는 quick stop 확인이 실패해도 저장된 command 를 비운다.
void test_disconnect_clears_staged_command()
{
  HandCore core{make_config()};
  HandCoreTestPeer::stage_position_command(core);
  HandCoreTestPeer::force_running(core);

  const Status result = core.disconnect();

  check(!result.ok(), "전제: RT 루프가 없어 quick stop 확인은 실패한다");
  check(std::holds_alternative<Idle>(HandCoreTestPeer::staged(core).controller),
        "disconnect() 는 확인 실패와 무관하게 저장된 command 를 Idle 로 되돌린다");
}

// (c) connect() 는 마지막 ControllerConfig 를 RT 로 다시 싣는다.
//     RealtimeBuffer 는 값을 한 번만 건네고, RT 스레드는 connect() 마다 새로 뜨면서 기본값에서
//     시작한다. 다시 싣지 않으면 set_controller_config() 로 넣은 gain 이 재연결 뒤 조용히 사라진다.
void test_connect_republishes_staged_config()
{
  HandCore core{make_config()};

  ControllerConfig config{};
  config.joint_position_controller.cutoff_freq = 42.0;
  check(core.set_controller_config(config).ok(), "전제: config 가 받아들여진다");

  // RT 루프가 한 번 읽어 간 상태를 흉내낸다
  ControllerConfig consumed{};
  check(HandCoreTestPeer::read_config(core, consumed), "전제: 첫 read 가 값을 받는다");
  check(!HandCoreTestPeer::read_config(core, consumed), "전제: 같은 값을 두 번 건네지 않는다");

  HandCoreTestPeer::republish(core);

  ControllerConfig after{};
  check(HandCoreTestPeer::read_config(core, after), "connect() 가 부르는 republish 가 staged config 를 다시 싣는다");
  check(after.joint_position_controller.cutoff_freq == 42.0, "다시 실린 값이 마지막 설정과 같다");
}

// (d) reconnect() 가 실패해도 멈춘 원인이 남는다.
void test_failed_reconnect_keeps_stop_cause()
{
  HandCore core{make_config()};
  const StopCause original{RealtimeEventKind::TransmitFailure, 777, 32, 0};
  HandCoreTestPeer::force_faulted(core, original);

  const Status result = core.reconnect();
  check(!result.ok(), "전제: 없는 interface 라 reconnect() 는 실패한다");
  check(HandCoreTestPeer::lifecycle(core) == HandLifecycle::Faulted,
        "실패하면 Faulted 로 남아 재시도가 다시 reconnect() 다");

  const StopCause kept = HandCoreTestPeer::stop_cause(core);
  check(kept.reason == original.reason && kept.cycle == original.cycle,
        "실패해도 초기화 전 원인이 남는다");

  const std::string phrase = stop_cause_phrase(kept);
  check(phrase.find("CAN TX failed at cycle 777") != std::string::npos,
        "원인 문구가 실제 사건을 가리킨다");
  check(phrase.find("no RX since cycle 0") == std::string::npos,
        "기본값이 렌더돼 일어나지 않은 사건을 보고하지 않는다");
}

}  // namespace

int main()
{
  std::printf("reconnect_reset_test\n");
  test_reconnect_clears_staged_command();
  test_disconnect_clears_staged_command();
  test_connect_republishes_staged_config();
  test_failed_reconnect_keeps_stop_cause();
  if (g_failures != 0) {
    std::printf("  %d 건 실패\n", g_failures);
    return 1;
  }
  std::printf("  통과\n");
  return 0;
}
