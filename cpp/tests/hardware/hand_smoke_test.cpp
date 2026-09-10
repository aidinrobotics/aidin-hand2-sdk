// 하드웨어 통합 스모크 — 실기가 있을 때만 실행된다(ctest label "hardware").
//
// 게이팅: 환경변수 HAND_TEST_CAN 이 없으면 SKIP(통과 처리)한다. 그래서 트리에 있어도 CI/개발
// 머신에서 안전하다. 실기에서는:  HAND_TEST_CAN=can0 [HAND_TEST_SIDE=left|right] ctest -L hardware
//
// 안전: 기본은 **listen-only** — connect(관측만, 무토크) → 상태 조회 → disconnect → destroy.
// run()/home()/set_command(모션)은 하지 않는다(실제 손이 움직이므로 작업자 감독 하에 별도로).
//
// 이 스모크는 실기 RX 수립과 **깨끗한 종료**를 검증한다: 전체를 watchdog 로 감싸, connect/
// disconnect/destroy 가 유한 시간에 반환되지 않으면(교착/무종료) 테스트를 실패시킨다.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include <aidin_hand2/aidin_hand2.hpp>

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

constexpr int kWatchdogSeconds = 15;

void listen_only_smoke(const char* iface, aidin_hand2::HandSide side)
{
  namespace ah2 = aidin_hand2;
  ah2::HandManager manager;
  ah2::HandConfig config{iface, side};
  config.auto_home = false;  // run/home 안 함 — 관측만

  ah2::Hand hand = manager.create(config);
  try {
    hand.connect();  // 첫 수신 확인까지 blocking (~300ms) · 무토크
    // 실기 RX 관측 — 몇 번 상태를 읽어 예외 없이 관측되는지 확인.
    for (int i = 0; i < 5; ++i) {
      (void)hand.get_state();
      (void)hand.get_diagnostics();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    hand.disconnect();  // (제어 세션 아님) 통신 내리고 Disconnected
  } catch (const ah2::Exception& e) {
    check(false, "hardware smoke: connect/observe/disconnect 중 예외");
    std::printf("  (%s: %s)\n", ah2::to_string(e.code()), e.what());
  }
  manager.destroy(hand);  // 정지 확인 + 종료 + 자원 파기
}

}  // namespace

int main()
{
  const char* iface = std::getenv("HAND_TEST_CAN");
  if (iface == nullptr || iface[0] == '\0') {
    std::printf("hand_smoke_test: SKIPPED — set HAND_TEST_CAN=can0 (and optionally "
                "HAND_TEST_SIDE=left|right) to run against real hardware.\n");
    return 0;  // 하드웨어 없음 → 통과 처리(트리에 있어도 안전)
  }
  const char* side_env = std::getenv("HAND_TEST_SIDE");
  const aidin_hand2::HandSide side =
      (side_env != nullptr && std::string(side_env) == "right") ? aidin_hand2::HandSide::Right
                                                                : aidin_hand2::HandSide::Left;

  // watchdog: 스모크가 유한 시간에 끝나지 않으면 교착으로 보고 강제 실패 종료.
  std::atomic<bool> done{false};
  std::thread watchdog([&done] {
    for (int i = 0; i < kWatchdogSeconds * 10 && !done.load(std::memory_order_acquire); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!done.load(std::memory_order_acquire)) {
      std::fprintf(stderr, "FAIL: hardware smoke timed out after %ds — connect/disconnect/destroy "
                           "did not return (hang?).\n", kWatchdogSeconds);
      std::fflush(stderr);
      std::_Exit(1);
    }
  });
  watchdog.detach();

  std::printf("hand_smoke_test: running against %s (%s)\n", iface,
              side == aidin_hand2::HandSide::Right ? "right" : "left");
  listen_only_smoke(iface, side);
  done.store(true, std::memory_order_release);

  if (g_failures == 0) {
    std::printf("hand_smoke_test: PASS\n");
  }
  return g_failures;
}
