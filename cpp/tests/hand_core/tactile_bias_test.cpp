// 목적(Tier 1, HW 무관): tactile bias 가 RT 루프의 한 주기 안에서 어떻게 적용되는지 점검한다.
//   (a) 요청이 없으면 raw 가 그대로 지나가고 bias 는 0 이다.
//   (b) set 요청을 받은 주기는 그 주기의 raw 를 bias 로 잡아 0 을 낸다.
//   (c) 다음 주기부터는 raw − bias 이고, 음수 차이(−65535 까지)도 그대로 나온다.
//   (d) 요청은 한 번만 소비되고, 소비한 주기에만 event ring 에 log 용 event 가 들어간다.
//   (e) reset 요청 뒤에는 raw 로 돌아가고 bias 는 0 이다.
//   (f) RT 루프가 없는 Disconnected 에서 set/reset 은 WrongCallOrder 로 거부된다.

#include <cstdio>

#include <aidin_hand2/types/config.hpp>
#include <aidin_hand2/types/state.hpp>

#include "hand_core/hand_core.hpp"

using namespace aidin_hand2;

namespace aidin_hand2::test
{
struct HandCoreTestPeer {
  static void request_set(HandCore& core)
  {
    core.request_tactile_bias_.store(HandCore::TactileBiasRequest::Set);
  }
  static void request_reset(HandCore& core)
  {
    core.request_tactile_bias_.store(HandCore::TactileBiasRequest::Reset);
  }
  static void apply(HandCore& core, HandState& state, std::uint64_t cycle)
  {
    core.apply_tactile_bias(state, cycle);
  }
  // 이번 주기에 들어간 event 를 하나 꺼낸다. 없으면 false.
  static bool pop_event(HandCore& core, RealtimeEvent& out) { return core.event_ring_.pop(out); }
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

// 소켓을 열지 않는다. 생성만 한다.
HandConfig make_config()
{
  HandConfig config{"ah2_no_such_iface", HandSide::Right};
  config.control_rate = 500;
  config.auto_reconnect = false;
  return config;
}

// taxel 마다 다른 값이어야 index 가 섞여도 드러난다. 끝 값 0 과 65535 를 포함한다.
HandState raw_state()
{
  HandState state{};
  for (std::size_t f = 0; f < kFingerCount; ++f)
    for (std::size_t t = 0; t < kTactileTaxelsPerFinger; ++t)
      state.tactile.fingers[f][t] = static_cast<std::int32_t>(1000 * (f + 1) + t);
  for (std::size_t t = 0; t < kPalmTactileCount; ++t)
    state.tactile.palm[t] = static_cast<std::int32_t>(20000 + t);
  state.tactile.fingers[0][0] = 0;
  state.tactile.palm[kPalmTactileCount - 1] = 65535;
  return state;
}

bool tactile_equal(const TactileState& a, const TactileState& b)
{
  return a.fingers == b.fingers && a.palm == b.palm;
}

bool tactile_all(const TactileState& s, std::int32_t value)
{
  for (const auto& finger : s.fingers)
    for (std::int32_t taxel : finger)
      if (taxel != value) return false;
  for (std::int32_t taxel : s.palm)
    if (taxel != value) return false;
  return true;
}

// (a)
void test_no_request_passes_raw()
{
  HandCore core{make_config()};
  HandState state = raw_state();
  HandCoreTestPeer::apply(core, state, 1);
  RealtimeEvent event{};
  check(!HandCoreTestPeer::pop_event(core, event), "(a) 요청이 없으면 event 도 없다");
  check(tactile_equal(state.tactile, raw_state().tactile), "(a) 요청이 없으면 raw 가 그대로 지나간다");
  check(tactile_all(core.tactile_bias(), 0), "(a) bias 는 0 이다");
}

// (b)(c)(d)(e)
void test_set_then_reset()
{
  HandCore core{make_config()};

  // (b) set 주기
  HandCoreTestPeer::request_set(core);
  HandState state = raw_state();
  HandCoreTestPeer::apply(core, state, 100);
  RealtimeEvent event{};
  check(HandCoreTestPeer::pop_event(core, event) && event.kind == RealtimeEventKind::TactileBiasSet &&
            event.cycle == 100,
        "(b) set 을 처리한 주기 번호로 TactileBiasSet event 가 들어간다");
  check(tactile_all(state.tactile, 0), "(b) set 주기의 tactile 은 0 이다");
  check(tactile_equal(core.tactile_bias(), raw_state().tactile), "(b) bias 는 그 주기의 raw 다");
  check(tactile_equal(core.tactile_bias(), raw_state().tactile), "(b) bias 는 다시 읽어도 같다");

  // (c)(d) 다음 주기: 요청 없이 raw 가 변한다
  HandState next = raw_state();
  next.tactile.fingers[2][5] += 37;
  next.tactile.fingers[4][16] -= 120;
  next.tactile.palm[3] -= 20003;                     // raw 0, bias 20003
  next.tactile.palm[kPalmTactileCount - 1] = 0;      // raw 0, bias 65535
  HandCoreTestPeer::apply(core, next, 101);
  check(!HandCoreTestPeer::pop_event(core, event), "(d) 요청은 한 번만 소비되어 다음 주기에는 event 가 없다");
  check(tactile_equal(core.tactile_bias(), raw_state().tactile), "(d) 다음 주기에도 bias 는 그대로다");
  check(next.tactile.fingers[2][5] == 37, "(c) 양의 차이");
  check(next.tactile.fingers[4][16] == -120, "(c) 음의 차이");
  check(next.tactile.palm[3] == -20003, "(c) palm 의 음의 차이");
  check(next.tactile.palm[kPalmTactileCount - 1] == -65535, "(c) 가장 큰 음의 차이 −65535");
  check(next.tactile.fingers[1][0] == 0, "(c) 변하지 않은 taxel 은 0");

  // (e) reset 주기
  HandCoreTestPeer::request_reset(core);
  HandState after = raw_state();
  HandCoreTestPeer::apply(core, after, 200);
  check(HandCoreTestPeer::pop_event(core, event) && event.kind == RealtimeEventKind::TactileBiasReset &&
            event.cycle == 200,
        "(e) reset 을 처리한 주기 번호로 TactileBiasReset event 가 들어간다");
  check(tactile_equal(after.tactile, raw_state().tactile), "(e) reset 주기부터 raw 가 그대로 나온다");
  check(tactile_all(core.tactile_bias(), 0), "(e) reset 뒤 bias 는 0 이다");
}

// (f)
void test_refused_without_loop()
{
  HandCore core{make_config()};
  const Status set = core.set_tactile_bias();
  check(!set.ok() && set.code == ErrorCode::WrongCallOrder, "(f) Disconnected 에서 set 은 WrongCallOrder");
  const Status reset = core.reset_tactile_bias();
  check(!reset.ok() && reset.code == ErrorCode::WrongCallOrder, "(f) Disconnected 에서 reset 은 WrongCallOrder");
}

}  // namespace

int main()
{
  std::printf("tactile_bias_test\n");
  test_no_request_passes_raw();
  test_set_then_reset();
  test_refused_without_loop();
  if (g_failures != 0) {
    std::printf("  %d 건 실패\n", g_failures);
    return 1;
  }
  std::printf("  통과\n");
  return 0;
}
