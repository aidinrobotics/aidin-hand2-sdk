// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, the current cap and what it does to a blocked finger
//
// set_max_effort() caps what the drives are allowed to draw, in units of 0.1 % of rated
// current. Rated is 400 mA on every motor, so 1000 is 100 % and 400 mA.
//
// The cap is raised in three steps while the same command is held. Hold a finger with your
// hand and the measured current climbs with each step, because the cap is what the controller
// was running into. Leave the hand alone and the current stays near 0 at every step, because
// nothing was asking for torque in the first place.
//
// Run as: 05_max_effort [right|left] [interface=can0]
// Needs a real hand. home() drives every finger into its hard stop, and the fingers then flex
// against whatever is in the way — including your hand, gently, which is the point.

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
// SIGINT only asks the loops to stop. The HandManager destructor is what confirms the quick
// stop and closes the link, so the point is to leave the loop and reach that destructor.
std::atomic<bool> g_shutdown{false};
}  // namespace

int main(int argc, char** argv)
{
  std::signal(SIGINT, [](int) { g_shutdown.store(true); });

  const HandSide side = (argc > 1 && std::string(argv[1]) == "left") ? HandSide::Left : HandSide::Right;

  try {
    HandConfig config{argc > 2 ? argv[2] : "can0", side};
    config.control_rate = 500;    // Hz
    config.auto_home    = false;

    HandManager manager;
    Hand hand = manager.create(config);
    hand.connect();

    // 1) One value for every actuator. The cap can be set at any point in the lifecycle, and
    //    this one is set before connect() has anything to drive on purpose: a low cap in place
    //    before the first command means a mistyped target cannot push hard.
    hand.set_max_effort(300.0);
    std::printf("cap set to 300 (30 %% of rated, about 120 mA)\n");

    // 2) The applied value is readable, so there is no need to track it in the application.
    //    It appears in HandState rather than in Diagnostics, because it is part of the command
    //    path rather than of the loop's health.
    const HandState staged = hand.get_state();
    std::printf("state.commanded.max_effort_pct[0] = %.0f\n\n", staged.commanded.max_effort_pct[0]);

    // 3) Out of range does not throw. Anything outside [0, 2000] is pulled to the nearest end,
    //    so a bad number becomes a safe number rather than an exception to handle.
    hand.set_max_effort(5000.0);
    std::printf("asked for 5000, applied %.0f — the range is [0, 2000]\n",
                hand.get_state().commanded.max_effort_pct[0]);
    hand.set_max_effort(-100.0);
    std::printf("asked for -100, applied %.0f\n\n", hand.get_state().commanded.max_effort_pct[0]);

    // 4) A per-actuator array for the other overload. The thumb carries the grasp, so it keeps
    //    the higher cap while the long fingers are held lower.
    const std::array<double, kActuatorCount> per_actuator = {
        600.0, 600.0, 600.0, 600.0,   // thumb   a0 a1 a2 a3
        400.0, 400.0, 400.0,          // index   a1 a2 a3
        400.0, 400.0, 400.0,          // middle
        400.0, 400.0, 400.0,          // ring
        400.0, 400.0, 400.0};         // baby
    hand.set_max_effort(per_actuator);
    const HandState mixed = hand.get_state();
    std::printf("per-actuator cap: thumb a0 %.0f, index a1 %.0f\n\n",
                mixed.commanded.max_effort_pct[0], mixed.commanded.max_effort_pct[4]);

    // 5) Bring the hand under control and home it, then flex every finger. From here on the
    //    controller is asking for a position it may not be able to reach, which is the
    //    situation in which the cap is what decides the current.
    hand.run();
    hand.home();

    JointPositionCommand flex;
    flex.target.fill(0.0);
    // Active joint index: T[0..3] I[4..6] M[7..9] R[10..12] B[13..15]. The abduction joints
    // stay at 0, so only the flexion joints below are asked to move.
    for (const std::size_t j : {0u, 1u, 3u, 5u, 6u, 8u, 9u, 11u, 12u, 14u, 15u}) {
      flex.target[j] = 40.0 * 3.14159265358979323846 / 180.0;  // rad
    }
    hand.set_command(flex);
    std::printf("flexing to 40 deg — hold a finger to feel each step\n\n");

    // 6) Raise the cap in three steps, holding the same command throughout. Only the cap
    //    changes between the rows, so the measured current is answering for the cap alone.
    for (const double cap : {200.0, 600.0, 1000.0}) {
      if (g_shutdown.load()) break;
      hand.set_max_effort(cap);

      // Let the drives settle at the new cap before reading, since the change takes effect on
      // the next cycle but the current needs a moment to get there.
      for (int i = 0; i < 25 && !g_shutdown.load(); ++i) {
        const HandState state = hand.get_state();
        std::printf("\r\033[Kcap %6.0f  |  measured mA: a0 %7.1f  a4 %7.1f  a7 %7.1f",
                    cap, state.actuators.current_mA[0], state.actuators.current_mA[4],
                    state.actuators.current_mA[7]);
        std::fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      std::printf("\n");
    }
    std::printf("\n");

    // 7) The cap does not survive a destroy, but it does survive a reconnect: connect()
    //    reissues the staged config, so a recovered hand comes back with this cap in place
    //    rather than with the default 1000.
    hand.set_command(Idle{});
    hand.stop();
    std::printf("done — the cap is reissued by connect(), so a reconnect() keeps it\n");
  } catch (const Exception& error) {
    std::printf("\nfailed: %s: %s\n", to_string(error.code()), error.what());
    return 1;
  }

  return 0;
}
