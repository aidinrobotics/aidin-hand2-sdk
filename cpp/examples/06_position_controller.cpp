// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, what the joint position controller does to a target before it ships
//
// A JointPositionCommand carries the target and nothing else. The deadband, the low-pass
// filter and the inverse kinematics that turn it into an actuator position all live in
// ControllerConfig, and this example changes them one at a time under the same step command.
//
// Every row is the setpoint the controller actually produced, read back from
// state.commanded.controller_output. With the filter off, the setpoint jumps to the target on
// the first cycle. With it on, the same step is spread over cycles, and a lower cutoff spreads
// it further. The deadband pass then shows targets small enough to be dropped entirely.
//
// Run as: 06_position_controller [right|left] [interface=can0]
// Needs a real hand. home() drives every finger into its hard stop, and the index finger flexes
// repeatedly afterwards, so clear the space around the hand.

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <csignal>
#include <string>
#include <thread>
#include <variant>

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

  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

  // The index finger. kJoint indexes the command's active-joint array and kActuator indexes
  // the setpoint array, and the two spaces are separate: an actuator index and a joint index
  // of the same number are not the same axis. Actuator 5 is one of this finger's three, so a
  // step in this joint shows up in its setpoint, which is all this example needs.
  constexpr std::size_t kJoint    = 5;
  constexpr std::size_t kActuator = 5;

  try {
    HandConfig config{argc > 2 ? argv[2] : "can0", side};
    config.control_rate = 500;    // Hz
    config.auto_home    = false;

    HandManager manager;
    Hand hand = manager.create(config);
    hand.connect();
    hand.set_max_effort(600.0);
    hand.run();
    hand.home();

    JointPositionCommand command;
    command.target.fill(0.0);

    // 1) Three settings for the same 30 deg step. filter_enabled false skips both the deadband
    //    and the filter, so it is the raw inverse kinematics result.
    struct Setting {
      const char* label;
      bool        filter_enabled;
      double      cutoff_freq;  // Hz
    };
    const std::array<Setting, 3> settings = {{
        {"filter off       ", false, 0.0},
        {"filter on, 40 Hz ", true, 40.0},
        {"filter on, 5 Hz  ", true, 5.0},
    }};

    for (const Setting& setting : settings) {
      if (g_shutdown.load()) break;

      ControllerConfig controller;
      controller.joint_position_controller.filter_enabled = setting.filter_enabled;
      controller.joint_position_controller.cutoff_freq    = setting.cutoff_freq;
      controller.joint_position_controller.deadband       = 0.0;  // rad, out of the way for now
      hand.set_controller_config(controller);

      // Back to 0 and let it settle, so every row below starts from the same place.
      command.target[kJoint] = 0.0;
      hand.set_command(command);
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));

      // 2) The step. Sampling every 40 ms is coarser than the 500 Hz loop, so the numbers are
      //    a sketch of the response rather than every cycle of it — enough to tell a jump
      //    from a ramp.
      command.target[kJoint] = 30.0 * kDegToRad;
      hand.set_command(command);

      std::printf("%s setpoint cnt:", setting.label);
      for (int i = 0; i < 12 && !g_shutdown.load(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
        const HandState state = hand.get_state();
        // controller_output is a variant. A position command produces ActuatorPositionSetpoint,
        // and get_if returns nullptr when it holds something else.
        if (const auto* setpoint =
                std::get_if<ActuatorPositionSetpoint>(&state.commanded.controller_output)) {
          std::printf(" %6.0f", setpoint->target_position_cnt[kActuator]);
        } else {
          std::printf("      -");
        }
      }
      std::printf("\n");
    }
    std::printf("\n");

    // 3) The deadband, on its own. The filter is off so that nothing but the deadband decides
    //    whether the setpoint moves.
    ControllerConfig deadband_only;
    deadband_only.joint_position_controller.filter_enabled = true;
    deadband_only.joint_position_controller.cutoff_freq    = 0.0;              // Hz, skipped
    deadband_only.joint_position_controller.deadband       = 1.0 * kDegToRad;  // rad, exaggerated
    hand.set_controller_config(deadband_only);

    command.target[kJoint] = 0.0;
    hand.set_command(command);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    // 4) Walk the target up in 0.3 deg steps against a 1.0 deg deadband. Each step alone is
    //    too small to pass, and the comparison is against the last target that did pass, so
    //    the setpoint holds for three steps and then moves once.
    std::printf("deadband 1.0 deg, target in 0.3 deg steps\n");
    for (int step = 1; step <= 8 && !g_shutdown.load(); ++step) {
      command.target[kJoint] = step * 0.3 * kDegToRad;
      hand.set_command(command);
      std::this_thread::sleep_for(std::chrono::milliseconds(300));

      const HandState state = hand.get_state();
      const auto* setpoint =
          std::get_if<ActuatorPositionSetpoint>(&state.commanded.controller_output);
      std::printf("  target %4.1f deg  ->  setpoint %6.0f cnt\n", step * 0.3,
                  setpoint != nullptr ? setpoint->target_position_cnt[kActuator] : 0.0);
    }
    std::printf("\n");

    // 5) The deadband exists because of the gear ratio: a change too small to matter at the
    //    joint is still a reversal at the motor, and a reversal crosses the backlash. Pick it
    //    from how much noise the input carries, not from how precise the joint has to be.
    hand.set_command(Idle{});
    hand.stop();
    std::printf("done — the config stays until the next set_controller_config()\n");
  } catch (const Exception& error) {
    // An InvalidArgument here is a negative or non-finite field in the config, and the previous
    // config stays in force when that happens.
    std::printf("\nfailed: %s: %s\n", to_string(error.code()), error.what());
    return 1;
  }

  return 0;
}
