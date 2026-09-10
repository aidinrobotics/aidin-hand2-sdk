// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, the joint impedance controller and the two gains that shape it
//
// The command looks like a position command, but the controller turns it into effort instead of
// position: the target becomes an actuator position through the inverse kinematics, and the
// effort follows from how far the actuator is from it.
//
//   effort = stiffness × position_error − damping × velocity
//
// Both gains are per actuator, not per joint. This example holds one target while the
// stiffness is raised, so the printed effort setpoint is answering for the gain alone. Hold
// the index finger away from its target and the effort grows; let go and it drops back.
//
// The controller is still in development. It works, but its behaviour may change in a later
// version, so do not build on it yet.
//
// Run as: 07_impedance_controller [right|left] [interface=can0]
// Needs a real hand. home() drives every finger into its hard stop, and the index finger then
// pushes toward a flexed target with a rising current cap.

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
  // both the gain arrays and the setpoint array, and the two spaces are separate: an actuator
  // index and a joint index of the same number are not the same axis. The gains are what make
  // this distinction matter here — they are per actuator, so kActuator is what selects one.
  constexpr std::size_t kJoint    = 5;
  constexpr std::size_t kActuator = 5;

  try {
    HandConfig config{argc > 2 ? argv[2] : "can0", side};
    config.control_rate = 500;    // Hz
    config.auto_home    = false;

    HandManager manager;
    Hand hand = manager.create(config);
    hand.connect();

    // 1) The cap matters more here than under a position command. An impedance controller with
    //    a large error asks for a large effort, and the cap is what keeps that bounded.
    hand.set_max_effort(500.0);

    hand.run();
    hand.home();

    // 2) The target. It is a joint angle in rad, the same as a position command takes, and the
    //    controller is chosen by the command type rather than by a mode setting.
    JointImpedanceCommand command;
    command.target.fill(0.0);
    command.target[kJoint] = 35.0 * kDegToRad;

    // 3) Raise the stiffness on this one actuator across three passes. Every other actuator
    //    keeps the default, so nothing else changes underneath the comparison.
    for (const double stiffness : {0.005, 0.02, 0.05}) {
      if (g_shutdown.load()) break;

      ControllerConfig controller;  // constructed fresh, so the other 15 stay at the default
      controller.joint_impedance_controller.stiffness[kActuator] = stiffness;
      controller.joint_impedance_controller.damping[kActuator]   = 1e-5;
      hand.set_controller_config(controller);

      // Resending the command is not required — the config change lands on the next cycle by
      // itself — but it makes each pass start the same way.
      hand.set_command(command);

      std::printf("stiffness %.3f — hold the index finger back and watch the effort\n", stiffness);
      for (int i = 0; i < 30 && !g_shutdown.load(); ++i) {
        const HandState state = hand.get_state();

        // 4) An impedance command produces ActuatorEffortSetpoint, where a position command
        //    produces ActuatorPositionSetpoint. The variant is how the difference shows up.
        const auto* setpoint =
            std::get_if<ActuatorEffortSetpoint>(&state.commanded.controller_output);

        std::printf("\r\033[K  pos %7.0f cnt  vel %7.1f rpm  ->  effort %7.1f  |  measured %6.1f mA",
                    state.actuators.position_count[kActuator],
                    state.actuators.velocity_rpm[kActuator],
                    setpoint != nullptr ? setpoint->target_effort_pct[kActuator] : 0.0,
                    state.actuators.current_mA[kActuator]);
        std::fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      std::printf("\n\n");
    }

    // 5) Sending Idle switches controllers as much as it changes the target: the effort goes to
    //    0 while the drives stay enabled, so the finger becomes back-driveable but not limp.
    hand.set_command(Idle{});
    std::printf("Idle — effort 0 with the drives still on\n");
    for (int i = 0; i < 20 && !g_shutdown.load(); ++i) {
      const HandState state = hand.get_state();
      const auto* setpoint = std::get_if<ActuatorEffortSetpoint>(&state.commanded.controller_output);
      std::printf("\r\033[K  effort %7.1f  |  measured %6.1f mA",
                  setpoint != nullptr ? setpoint->target_effort_pct[kActuator] : 0.0,
                  state.actuators.current_mA[kActuator]);
      std::fflush(stdout);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::printf("\n\n");

    // 6) The effort setpoint is in 0.1 % of rated current and the measurement is in mA, so the
    //    two columns above cannot be subtracted from one another. Rated is 400 mA, which makes
    //    1000 the whole of it, but the loop between them is the drive's rather than the SDK's.
    hand.stop();
    std::printf("done — gains are per actuator, so joint index and gain index are not the same\n");
  } catch (const Exception& error) {
    std::printf("\nfailed: %s: %s\n", to_string(error.code()), error.what());
    return 1;
  }

  return 0;
}
