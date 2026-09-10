// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, homing and the reason every command waits for it
//
// Until homing succeeds there is no absolute reference, so the SDK refuses every command but
// Idle. This example sends one before homing to see that refusal, then homes, then sends the
// same command again — the printed WrongCallOrder and the printed success are the same call.
//
// home() blocks until it finishes, so the encoder counts are printed before and after rather
// than during. The travel to the hard stops is what moves the zero, and the counts show it.
//
// Run as: 04_homing [right|left] [interface=can0]
// Needs a real hand. home() drives every finger into its hard stop, so clear the space around
// the hand and keep your hands off it until the run finishes.

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <csignal>
#include <string>
#include <thread>

#include <aidin_hand2/aidin_hand2.hpp>

using namespace aidin_hand2;

namespace
{
// SIGINT only asks the wait loop to stop. It cannot cut home() short — that call blocks inside
// the SDK — so Ctrl-C during homing takes effect once homing returns.
std::atomic<bool> g_shutdown{false};
}  // namespace

int main(int argc, char** argv)
{
  std::signal(SIGINT, [](int) { g_shutdown.store(true); });

  const HandSide side = (argc > 1 && std::string(argv[1]) == "left") ? HandSide::Left : HandSide::Right;

  // Four actuators spread across the array, one per printed row
  constexpr std::array<std::size_t, 4> probe = {0, 4, 8, 12};

  try {
    HandConfig config{argc > 2 ? argv[2] : "can0", side};
    config.control_rate = 500;    // Hz
    config.auto_home    = false;  // home() is called below, so run() must not do it as well

    HandManager manager;
    Hand hand = manager.create(config);
    hand.connect();

    // 1) Cap the current before anything moves. Homing pushes into a hard stop on purpose, and
    //    this is the cap it pushes with.
    hand.set_max_effort(600.0);  // % of rated current × 10, so 60 %

    std::printf("homing_state = %s\n\n", to_string(hand.get_diagnostics().homing_state));

    // 2) A joint target without a reference. The SDK refuses it, and the code says why:
    //    WrongCallOrder is the call sequence being wrong, not the target.
    JointPositionCommand zero_pose;
    zero_pose.target.fill(0.0);  // rad
    try {
      hand.set_command(zero_pose);
      std::printf("the command was accepted, which the SDK is not supposed to allow yet\n");
    } catch (const Exception& error) {
      std::printf("before homing — %s: %s\n\n", to_string(error.code()), error.what());
    }

    // 3) Idle is the exception. It needs no reference because it sends an effort of 0.
    //    It still needs the drives on, so run() comes first.
    hand.run();
    hand.set_command(Idle{});
    std::printf("Idle was accepted before homing, since it asks for no position\n\n");

    // 4) The counts before homing. They are whatever the drives happened to power up with.
    const HandState before = hand.get_state();
    std::printf("encoder count before homing:");
    for (const std::size_t a : probe) std::printf("  a%zu %8.0f", a, before.actuators.position_count[a]);
    std::printf("\n\n");

    // 5) home() blocks until every finger has reached its hard stop and the zero has been
    //    written. It takes a few seconds, and it enables the drives itself when they are off,
    //    so the run() above was for the Idle command rather than for this call.
    std::printf("homing — every finger travels to its hard stop\n");
    hand.home();
    std::printf("home() returned, homing_state = %s\n\n", to_string(hand.get_diagnostics().homing_state));

    // 6) The counts after homing. They are now measured from the hard stops, which is what
    //    makes a joint target mean the same thing on every run.
    const HandState after = hand.get_state();
    std::printf("encoder count after homing: ");
    for (const std::size_t a : probe) std::printf("  a%zu %8.0f", a, after.actuators.position_count[a]);
    std::printf("\n\n");

    // 7) The same command as step 2, unchanged. Homing is the only thing that happened in
    //    between, and homing also left a command holding the zero pose, so this replaces it.
    hand.set_command(zero_pose);
    std::printf("after homing — the same command was accepted\n\n");

    // 8) Hold for 2 s so the zero pose is visible, then finish. These are joint indices rather
    //    than the command's active-joint indices: joint 0 is the thumb's first and joint 5 is
    //    the index finger's first, because joint 4 is the thumb's passive one.
    for (int i = 0; i < 20 && !g_shutdown.load(); ++i) {
      const HandState state = hand.get_state();
      std::printf("\r\033[Kholding the zero pose:  j0 %+.4f rad  j5 %+.4f rad",
                  state.joints.position_rad[0], state.joints.position_rad[5]);
      std::fflush(stdout);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::printf("\n\n");

    // 9) A failed homing is a HardwareFault whose message names the actuator, and a fault later
    //    on costs the reference: reconnect() puts homing_state back to NotRun, so a recovered
    //    hand has to be homed again before it takes a command.
    hand.stop();
    std::printf("done — homing_state stays %s until a reconnect() clears it\n",
                to_string(hand.get_diagnostics().homing_state));
  } catch (const Exception& error) {
    // A HardwareFault here is most likely homing itself failing, and the message names the
    // actuator that did not reach its stop.
    std::printf("\nfailed: %s: %s\n", to_string(error.code()), error.what());
    return 1;
  }

  return 0;
}
