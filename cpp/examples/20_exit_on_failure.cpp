// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, a program that runs once and gives up on the first failure
//
// logging -> create -> connect -> run (homes) -> grip and release five times -> exit
// One try around the whole thing, because there is nothing to recover for: a calibration run,
// a diagnostic readout or a rehearsal fails, someone fixes the cause, and it runs again
//
// Run as: 20_exit_on_failure [right|left] [interface=can0]
// Needs a real hand. auto_home is on, so run() drives every finger into its hard stop:
// clear the space around the hand before running

#include <atomic>
#include <chrono>
#include <cstdio>
#include <csignal>
#include <string>
#include <thread>

#include <aidin_hand2/aidin_hand2.hpp>

using namespace aidin_hand2;

namespace
{
// SIGINT only asks the cycle loop to stop. The HandManager destructor is what confirms the
// quick stop and closes the link, so the point is to leave the loop and reach that destructor
std::atomic<bool> g_shutdown{false};
}  // namespace

int main(int argc, char** argv)
{
  std::signal(SIGINT, [](int) { g_shutdown.store(true); });

  set_log_level(LogLevel::Info);
  set_log_to_console(true);

  const HandSide side = (argc > 1 && std::string(argv[1]) == "left") ? HandSide::Left : HandSide::Right;

  // Declared before the try so the destructor still runs on the failure path, where it
  // confirms the quick stop and closes the link
  HandManager manager;
  int exit_code = 0;

  try {
    HandConfig config{argc > 2 ? argv[2] : "can0", side};
    config.control_rate = 500;    // Hz
    config.auto_home    = true;   // run() homes before it enables control

    Hand hand = manager.create(config);
    hand.connect();
    hand.set_max_effort(600.0);   // percent of rated current
    hand.run();                   // returns once homing is done and the drives are enabled

    JointPositionCommand open;
    open.target.fill(0.0);

    JointPositionCommand close;
    close.target = {
        0.30, 0.50, 0.00, 0.40,   // thumb   j0 j1 [j2=abduction] j3
        0.00, 0.60, 0.50,         // index   [j1=abduction] j2 j3
        0.00, 0.60, 0.50,         // middle
        0.00, 0.60, 0.50,         // ring
        0.00, 0.60, 0.50};        // baby

    for (int cycle = 0; cycle < 5 && !g_shutdown.load(); ++cycle) {
      std::printf("cycle %d/5\n", cycle + 1);
      hand.set_command(close);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      hand.set_command(open);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Ctrl-C is not a failure, so the exit code stays 0; the run simply ended early
    if (g_shutdown.load()) std::printf("interrupted, leaving through the destructor\n");
  } catch (const Exception&) {
    exit_code = 1;   // the SDK logged the cause before throwing
  }

  flush_log();
  return exit_code;
}
