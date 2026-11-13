// The messages HandCore returns for each failure, and the error code that goes with them
#include <array>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>

#include <aidin_hand2/hand/hand_manager.hpp>
#include <aidin_hand2/logging/logging.hpp>
#include <aidin_hand2/types/config.hpp>

#include "hand_core/hand_core.hpp"
#include "types/status.hpp"

using namespace aidin_hand2;

namespace aidin_hand2::test
{
// Reaches the lifecycle, the link liveness, the stop cause and the private waits, a friend in hand_core.hpp
struct HandCoreTestPeer {
  static void set_lifecycle(HandCore& core, HandLifecycle requested, HandLifecycle observed)
  {
    core.requested_lifecycle_.store(requested);
    core.lifecycle_.store(observed);
  }
  static void set_link(HandCore& core, bool alive) { core.check_communication_.store(alive); }
  static void set_stop_cause(HandCore& core, const StopCause& cause) { core.last_stop_cause_ = cause; }
  static Status enable_control(HandCore& core) { return core.enable_control(); }
  static Status wait_first_frame(HandCore& core) { return core.wait_first_frame("connect hand"); }
  // What destroy() raises first, so a call from another thread meets a destroy in progress
  static void set_destroying(HandCore& core) { core.request_destroy_.store(true); }
  // Back to Disconnected, so the destructor has nothing to stop
  static void disconnected(HandCore& core) { set_lifecycle(core, HandLifecycle::Disconnected, HandLifecycle::Disconnected); }
};
}  // namespace aidin_hand2::test

namespace
{

using Peer = test::HandCoreTestPeer;

int g_failures = 0;

void check(bool ok, const char* what)
{
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    ++g_failures;
  }
}

// code is the number the error table gives the cause
void check_status(const Status& status, std::uint16_t code, const std::string& message, const char* what)
{
  if (status.code != code || status.message != message) {
    std::printf("FAIL: %s\n  got  E%u: %s\n  want E%u: %s\n", what, unsigned{status.code},
                status.message.c_str(), unsigned{code}, message.c_str());
    ++g_failures;
  }
}

HandConfig config_without_auto_home()
{
  HandConfig config{"aidin_no_if", HandSide::Left, HandModel::Gen2};
  config.auto_home = false;
  return config;
}

// A link that is not alive refuses run(), home() and set_tactile_bias() with one message
void test_link_not_alive()
{
  HandCore core(config_without_auto_home());
  Peer::set_lifecycle(core, HandLifecycle::Connected, HandLifecycle::Connected);
  Peer::set_link(core, false);

  check_status(core.run(), 402, "cannot run hand: CAN communication failed",
               "run() without a live link");
  check_status(core.home(), 402, "cannot home hand: CAN communication failed",
               "home() without a live link");
  check_status(core.set_tactile_bias(), 402,
               "cannot set tactile bias: CAN communication failed", "set_tactile_bias() without a live link");
  Peer::disconnected(core);
}

// A fault during the enable wait reports the fault, not a drive that did not follow
void test_fault_during_enable()
{
  HandCore core(config_without_auto_home());
  Peer::set_lifecycle(core, HandLifecycle::Connected, HandLifecycle::Faulted);
  Peer::set_link(core, true);
  Peer::set_stop_cause(core, StopCause{RealtimeEventKind::ControlLoopFailure, 7, 0});
  check_status(Peer::enable_control(core), 601,
               "cannot run hand: hand faulted (E702 control loop terminated)",
               "loop failure during the enable wait");

  Peer::set_lifecycle(core, HandLifecycle::Connected, HandLifecycle::Faulted);
  Peer::set_link(core, true);
  Peer::set_stop_cause(core, StopCause{RealtimeEventKind::TransmitFailure, 9, 0});
  check_status(Peer::enable_control(core), 601,
               "cannot run hand: hand faulted (E405 CAN TX failed)",
               "TX failure during the enable wait");
  Peer::disconnected(core);
}

// Each case waits out the 500 ms stop budget, since no loop is there to confirm the stop
void test_stop_not_confirmed()
{
  HandCore core(config_without_auto_home());

  Peer::set_lifecycle(core, HandLifecycle::Running, HandLifecycle::Running);
  Peer::set_link(core, false);
  check_status(core.stop(), 403,
               "cannot stop hand: quick stop not confirmed (CAN communication failed)",
               "stop() without a live link");

  Peer::set_lifecycle(core, HandLifecycle::Running, HandLifecycle::Running);
  Peer::set_link(core, true);
  check_status(core.stop(), 503,
               "cannot stop hand: drives did not reach quick stop within 500 ms",
               "stop() with a live link");

  Peer::set_lifecycle(core, HandLifecycle::Running, HandLifecycle::Running);
  Peer::set_link(core, false);
  check_status(core.disconnect(), 403,
               "cannot disconnect hand: quick stop not confirmed (CAN communication failed)",
               "disconnect() without a live link");

  Peer::set_lifecycle(core, HandLifecycle::Running, HandLifecycle::Running);
  Peer::set_link(core, true);
  check_status(core.disconnect(), 503,
               "cannot disconnect hand: drives did not reach quick stop within 500 ms",
               "disconnect() with a live link");
  Peer::disconnected(core);
}

void test_no_first_frame()
{
  HandCore core(config_without_auto_home());
  check_status(Peer::wait_first_frame(core), 401,
               "cannot connect hand: no data received from aidin_no_if within 300 ms",
               "no frame within the connect wait");
}

// A destroyed hand refuses every call through its handle
void test_destroyed_handle()
{
  HandManager manager;
  Hand hand = manager.create(config_without_auto_home());
  manager.destroy(hand);
  try {
    hand.connect();
    check(false, "connect() on a destroyed hand throws");
  } catch (const Exception& error) {
    check(error.code() == 207, "destroyed hand is E207");
    check(error.kind() == ErrorKind::WrongCallOrder, "destroyed hand is WrongCallOrder");
    check(std::string(error.what()) == "[left] [E207] cannot use hand: hand is destroyed", "destroyed hand message");
  }
}

// what() names the hand before the code, so a printed what() tells two hands apart
void test_side_in_what()
{
  HandManager manager;
  Hand hand = manager.create(config_without_auto_home());
  try {
    hand.run();
    check(false, "run() before connect() throws");
  } catch (const Exception& error) {
    check(std::string(error.what()) == "[left] [E202] cannot run hand: not connected", "refused call message");
  }
  try {
    hand.connect();
    check(false, "connect() on a missing interface throws");
  } catch (const Exception& error) {
    check(std::string(error.what()) == "[left] [E301] cannot connect hand: CAN interface 'aidin_no_if' not found",
          "interface failure message");
  }
  manager.destroy(hand);
}

// Anything other than an Exception becomes E901 with the call it escaped from
void test_unexpected_failure()
{
  try {
    guard_boundary(HandSide::Right, "run hand", []() -> void { throw std::runtime_error("boom"); });
    check(false, "an unexpected exception is rethrown");
  } catch (const Exception& error) {
    check(std::string(error.what()) == "[right] [E901] cannot run hand: unexpected failure (boom)",
          "unexpected failure message");
  }
}

// A call that meets a destroy in progress reads the same as one through a handle that outlived its core
void test_destroy_in_progress()
{
  HandCore core(config_without_auto_home());
  Peer::set_destroying(core);

  check_status(core.set_max_effort(std::array<double, kActuatorCount>{}), 207,
               "cannot use hand: hand is destroyed", "set_max_effort() during destroy");
  check_status(core.set_controller_config(ControllerConfig{}), 207,
               "cannot use hand: hand is destroyed", "set_controller_config() during destroy");
  check_status(core.run(), 207, "cannot use hand: hand is destroyed", "run() during destroy");
  try {
    (void)core.diagnostics();
    check(false, "diagnostics() during destroy throws");
  } catch (const Exception& error) {
    check(std::string(error.what()) == "[left] [E207] cannot use hand: hand is destroyed",
          "diagnostics() during destroy message");
  }
}

// A thrown failure goes to the caller only, while a warning with no caller is logged with its side
void test_log_lines()
{
  std::string logged;
  set_log_callback([&logged](LogLevel, const std::string& message) { logged += message + "\n"; });

  HandManager manager;
  Hand hand = manager.create(config_without_auto_home());
  manager.destroy(hand);
  try {
    hand.connect();
  } catch (const Exception&) {
  }
  HandConfig out_of_range = config_without_auto_home();
  out_of_range.disabled_actuators = {16};
  Hand second = manager.create(out_of_range);
  manager.destroy(second);
  set_log_callback(nullptr);

  check(logged.find("[E207]") == std::string::npos, "a thrown failure is not logged");
  check(logged.find("[left] disabled_actuators: ignoring out-of-range index 16\n") != std::string::npos,
        "create() logs the side of its config");
}

}  // namespace

int main()
{
  test_link_not_alive();
  test_fault_during_enable();
  test_stop_not_confirmed();
  test_no_first_frame();
  test_destroyed_handle();
  test_side_in_what();
  test_unexpected_failure();
  test_destroy_in_progress();
  test_log_lines();

  if (g_failures == 0) std::printf("failure_messages: all checks passed\n");
  return g_failures == 0 ? 0 : 1;
}
