// connect() logs a Connection Info block on the move to Connected, checked line for line
#include <cstdio>
#include <string>

#include <aidin_hand2/logging/logging.hpp>
#include <aidin_hand2/types/config.hpp>
#include <aidin_hand2/version.hpp>

#include "hand_core/hand_core.hpp"

using namespace aidin_hand2;

namespace aidin_hand2::test
{
// Reaches the realtime report, the diagnostics buffer and transition_to, a friend in hand_core.hpp
struct HandCoreTestPeer {
  static void set_realtime(HandCore& core, int fifo_priority, int running_cpu, bool memory_locked)
  {
    core.realtime_report_.fifo_priority = fifo_priority;
    core.realtime_report_.running_cpu   = running_cpu;
    core.realtime_report_.memory_locked = memory_locked;
  }
  static void set_fault(HandCore& core, std::size_t actuator)
  {
    Diagnostics diagnostics{};
    diagnostics.actuator_health.fault[actuator] = ActuatorFault::OverCurrentError;
    core.diagnostics_buffer_.write(diagnostics);
  }
  // Back to Disconnected afterwards, so the destructor has nothing to stop
  static void log_connected(HandCore& core)
  {
    core.transition_to(HandLifecycle::Connected);
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

// A whole line, from its first column
bool has_line(const std::string& text, const std::string& line)
{
  return text.find("\n" + line + "\n") != std::string::npos;
}

HandConfig left_gen2()
{
  HandConfig config{"can0", HandSide::Left, HandModel::Gen2};
  config.auto_home = true;
  return config;
}

// The message the connect log hands to a log callback
std::string connect_log(const HandConfig& config, int fifo_priority, int running_cpu, bool memory_locked,
                        int faulted_actuator)
{
  std::string captured;
  set_log_callback([&captured](LogLevel, const std::string& message) {
    if (message.find("connect() succeeded") != std::string::npos) captured = message;
  });
  {
    HandCore core(config);
    test::HandCoreTestPeer::set_realtime(core, fifo_priority, running_cpu, memory_locked);
    if (faulted_actuator >= 0) test::HandCoreTestPeer::set_fault(core, static_cast<std::size_t>(faulted_actuator));
    test::HandCoreTestPeer::log_connected(core);
  }
  set_log_callback(nullptr);
  return captured;
}

void test_block()
{
  const std::string expected =
      std::string("[left] connect() succeeded\n") +
      std::string(26, '-') + " Connection Info " + std::string(25, '-') + "\n" +
      "side        left\n" +
      "model       gen2\n" +
      "interface   can0\n" +
      "rate        500 Hz\n" +
      "auto_home   on\n" +
      "scheduler   SCHED_FIFO 90\n" +
      "cpu         3\n" +
      "affinity    none\n" +
      "memory      locked\n" +
      "actuators   16\n" +
      "disabled    0\n" +
      "faulted     0\n" +
      "sdk         " + kVersionString + "\n" +
      std::string(68, '-');
  const std::string text = connect_log(left_gen2(), 90, 3, true, -1);
  check(text == expected, "the block for a left Gen2 hand under SCHED_FIFO");
  if (text != expected) std::printf("got:\n%s\nexpected:\n%s\n", text.c_str(), expected.c_str());
}

void test_variants()
{
  HandConfig config = left_gen2();
  config.hand_side          = HandSide::Right;
  config.hand_model         = HandModel::Gen2B;
  config.auto_home          = false;
  config.rt_cpu_affinity    = 2;
  config.disabled_actuators = {3, 7};
  const std::string text = connect_log(config, 0, 3, false, 5);
  check(has_line(text, "side        right"), "right side");
  check(has_line(text, "model       gen2b"), "Gen2B as gen2b");
  check(has_line(text, "auto_home   off"), "auto_home off");
  check(has_line(text, "scheduler   SCHED_OTHER"), "no SCHED_FIFO");
  check(has_line(text, "affinity    2"), "an affinity");
  check(has_line(text, "memory      not locked"), "an unlocked loop");
  check(has_line(text, "disabled    2"), "disabled count");
  check(has_line(text, "faulted     1"), "faulted count");
}

}  // namespace

int main()
{
  test_block();
  test_variants();
  std::printf("connection_info_test: %s\n", g_failures == 0 ? "PASS" : "FAIL");
  return g_failures;
}
