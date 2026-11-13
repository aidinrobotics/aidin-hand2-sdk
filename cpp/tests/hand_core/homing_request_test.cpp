// home() leaves homing_state as it was when control cannot be requested
#include <cstdio>

#include <aidin_hand2/types/config.hpp>

#include "hand_core/hand_core.hpp"

using namespace aidin_hand2;

namespace aidin_hand2::test
{
// Reaches the lifecycle, the RX confirmation and homing_state, a friend in hand_core.hpp
struct HandCoreTestPeer {
  // Connected with no RX confirmed, so request_control() refuses
  static void connected_without_rx(HandCore& core)
  {
    core.requested_lifecycle_.store(HandLifecycle::Connected);
    core.lifecycle_.store(HandLifecycle::Connected);
    core.check_communication_.store(false);
  }
  static void set_homing_state(HandCore& core, HomingState state) { core.homing_state_.store(state); }
  static HomingState homing_state(const HandCore& core) { return core.homing_state_.load(); }
  // Back to Disconnected, so the destructor has nothing to stop
  static void disconnected(HandCore& core)
  {
    core.requested_lifecycle_.store(HandLifecycle::Disconnected);
    core.lifecycle_.store(HandLifecycle::Disconnected);
  }
};
}  // namespace aidin_hand2::test

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

// The interface does not exist, so request_control() reports it gone
void test_state_kept(HomingState before, const char* what)
{
  HandCore core(HandConfig{"aidin_no_if", HandSide::Left, HandModel::Gen2});
  test::HandCoreTestPeer::connected_without_rx(core);
  test::HandCoreTestPeer::set_homing_state(core, before);

  const Status status = core.home();
  check(status.kind() == ErrorKind::CommunicationLost, what);
  check(test::HandCoreTestPeer::homing_state(core) == before, what);

  test::HandCoreTestPeer::disconnected(core);
}

}  // namespace

int main()
{
  test_state_kept(HomingState::NotRun, "NotRun stays after a refused home()");
  test_state_kept(HomingState::Succeeded, "Succeeded stays after a refused home()");
  test_state_kept(HomingState::Failed, "Failed stays after a refused home()");

  if (g_failures == 0) std::printf("homing_request: all checks passed\n");
  return g_failures == 0 ? 0 : 1;
}
