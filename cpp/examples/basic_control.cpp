// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, a basic control example using only the public API
//
// create -> connect -> set_max_effort -> run -> home -> joint sweep -> stop -> destroy
// One header to include and one library to link, and every failure arrives as an Exception
//
// Run as: basic_control [right|left] [interface=can0]
// Needs a real hand, and the fingers move during home() and the sweep

#include <array>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include <aidin_hand2/aidin_hand2.hpp>

using namespace aidin_hand2;

int main(int argc, char** argv)
{
  const HandSide side = (argc > 1 && std::string(argv[1]) == "left") ? HandSide::Left : HandSide::Right;
  HandConfig config{argc > 2 ? argv[2] : "can0", side};
  config.control_rate   = 500;  // Hz

  try {
    // 1) HandManager owns the resources and hands back a non-owning Hand
    HandManager manager;
    Hand hand = manager.create(config);
    hand.connect();

    // 2) Effort limit, percent of rated current
    hand.set_max_effort(1000.0);

    // 3) Enable the drives, then home
    hand.run();
    hand.home();

    // 4) Move one joint at a time to 50 deg, hold, then return to 0
    //    Active joint index: T[0..3] I[4..6] M[7..9] R[10..12] B[13..15]
    //    The abduction joints are skipped, being the coupled axes of the parallel wrist
    constexpr std::array<bool, kActiveJointCount> is_abduction = {
        false, false, true,  false,   // thumb:  j0 j1 [j2=abd] j3
        true,  false, false,          // index:  [j1=abd] j2 j3
        true,  false, false,          // middle: [j1=abd] j2 j3
        true,  false, false,          // ring:   [j1=abd] j2 j3
        true,  false, false};         // baby:   [j1=abd] j2 j3
    constexpr double kTargetRad = 50.0 * 3.14159265358979323846 / 180.0;

    JointPositionCommand cmd;

    for (std::size_t j = 0; j < kActiveJointCount; ++j) {
      if (is_abduction[j]) continue;

      cmd.target.fill(0.0);
      cmd.target[j] = kTargetRad;
      // A target outside the workspace is clamped for you
      hand.set_command(cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));

      cmd.target.fill(0.0);
      hand.set_command(cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));

      const HandState state = hand.get_state();
      std::printf("joint %2zu: 50deg hold -> 0 | actuator %.0f cnt, %.0f mA\n",
                  j, state.actuators.position_count[j], state.actuators.current_mA[j]);
    }

    // 5) Read the diagnostics, stop the motion, then end the session
    const Diagnostics diag = hand.get_diagnostics();
    unsigned faulted = 0;
    for (std::size_t i = 0; i < kActuatorCount; ++i)
      if (diag.actuator_health.fault[i] != ActuatorFault::None) ++faulted;
    std::printf("diagnostics: lifecycle=%s faulted=%u cycles=%llu misses=%llu period=%.3fms\n",
                to_string(diag.lifecycle), faulted,
                static_cast<unsigned long long>(diag.control_cycles),
                static_cast<unsigned long long>(diag.deadline_misses), diag.last_period_ms);

    // hand.run() turns control back on
    hand.stop();

    manager.destroy(hand);

    return 0;
  } catch (const Exception&) {
    // The SDK already logged the failure, so only the exit code is left to decide
    flush_log();
    return 1;
  }
}
