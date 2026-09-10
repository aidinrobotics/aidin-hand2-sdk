// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, JointImpedanceCommand, the same target through a different controller
//
// JointImpedanceCommand and JointPositionCommand carry exactly the same thing: 16 joint angles
// in rad. The command type is what picks the controller, and the controller decides what
// reaches the drives — a position for one, an effort for the other.
//
// So this example sends one target both ways and prints the controller output each time. The
// variant that comes back is the difference, and it is the whole difference: the target
// arrays are identical.
//
// Under impedance the pose is a suggestion. The finger stops wherever the effort balances
// against what is holding it, so grasping an object gives a force that follows the
// displacement rather than a position that has to be reached.
//
// The controller is still in development. It works, but its behaviour may change in a later
// version, so do not build on it yet.
//
// Run as: 10_joint_impedance [right|left] [interface=can0]
// Needs a real hand. home() drives every finger into its hard stop, and the fingers then flex
// toward a grasp pose. Put a soft object in the hand for the impedance pass.

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

  // The index finger, watched through three different index spaces. A command is indexed by
  // active joint and state.joints by joint index, and the two differ because each finger's
  // passive joint takes the last slot of its group — so active 5 is joint 6. Actuator 5 belongs
  // to the same finger, but an actuator index and a joint index of the same number are not the
  // same axis, which is why all three are named separately here.
  constexpr std::size_t kActive   = 5;  // active joint index, what the command carries
  constexpr std::size_t kJoint    = 6;  // the same axis in state.joints
  constexpr std::size_t kActuator = 5;  // one of this finger's three actuators

  // One target, used by both command types below
  const std::array<double, kActiveJointCount> target = {
      0.20, 0.35, 0.10, 0.25,   // thumb   j0 j1 j2 j3
      0.08, 0.45, 0.30,         // index   j1 j2 j3
      0.04, 0.55, 0.40,         // middle
      -0.04, 0.65, 0.50,        // ring
      -0.08, 0.75, 0.60};       // baby

  try {
    HandConfig config{argc > 2 ? argv[2] : "can0", side};
    config.control_rate = 500;    // Hz
    config.auto_home    = false;

    HandManager manager;
    Hand hand = manager.create(config);
    hand.connect();
    hand.set_max_effort(500.0);
    hand.run();
    hand.home();

    // 1) The position command first, so there is something to compare against. The output is
    //    an actuator position, and the finger drives to it however hard the cap allows.
    JointPositionCommand position;
    position.target = target;
    hand.set_command(position);
    std::printf("JointPositionCommand, mode = %s\n",
                hand.get_command_mode() == CommandMode::JointPosition ? "JointPosition" : "other");

    for (int i = 0; i < 20 && !g_shutdown.load(); ++i) {
      const HandState state = hand.get_state();
      const auto* as_position =
          std::get_if<ActuatorPositionSetpoint>(&state.commanded.controller_output);
      const auto* as_effort =
          std::get_if<ActuatorEffortSetpoint>(&state.commanded.controller_output);
      std::printf("\r\033[K  output %-9s  actuator %zu %8.0f  |  active %zu reached %+.3f rad  %6.1f mA",
                  as_position != nullptr ? "position" : as_effort != nullptr ? "effort" : "none",
                  kActuator,
                  as_position != nullptr ? as_position->target_position_cnt[kActuator]
                  : as_effort != nullptr ? as_effort->target_effort_pct[kActuator]
                                         : 0.0,
                  kActive, state.joints.position_rad[kJoint],
                  state.actuators.current_mA[kActuator]);
      std::fflush(stdout);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::printf("\n\n");

    // 2) Back to the open pose between the two passes, so the impedance pass starts from a
    //    position error rather than from where the position controller already parked.
    JointPositionCommand open;
    open.target.fill(0.0);
    hand.set_command(open);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 3) Gains before the command. They are per actuator rather than per joint, and the
    //    defaults are the ones the guide lists — set here explicitly so this pass does not
    //    depend on whatever ran before it.
    ControllerConfig controller;
    controller.joint_impedance_controller.stiffness = {
        0.02, 0.02, 0.02, 0.02,   // thumb   a0 a1 a2 a3
        0.01, 0.01, 0.02,         // index   a1 a2 a3
        0.01, 0.01, 0.02,         // middle
        0.01, 0.01, 0.02,         // ring
        0.01, 0.01, 0.02};        // baby
    controller.joint_impedance_controller.damping.fill(1e-5);
    hand.set_controller_config(controller);

    // 4) The same 16 numbers, assigned to the other struct. Nothing else changed.
    JointImpedanceCommand impedance;
    impedance.target = target;
    hand.set_command(impedance);
    std::printf("JointImpedanceCommand with the identical target, mode = %s\n",
                hand.get_command_mode() == CommandMode::JointImpedance ? "JointImpedance" : "other");

    for (int i = 0; i < 30 && !g_shutdown.load(); ++i) {
      const HandState state = hand.get_state();
      const auto* as_position =
          std::get_if<ActuatorPositionSetpoint>(&state.commanded.controller_output);
      const auto* as_effort =
          std::get_if<ActuatorEffortSetpoint>(&state.commanded.controller_output);
      std::printf("\r\033[K  output %-9s  actuator %zu %8.0f  |  active %zu reached %+.3f rad  %6.1f mA",
                  as_position != nullptr ? "position" : as_effort != nullptr ? "effort" : "none",
                  kActuator,
                  as_position != nullptr ? as_position->target_position_cnt[kActuator]
                  : as_effort != nullptr ? as_effort->target_effort_pct[kActuator]
                                         : 0.0,
                  kActive, state.joints.position_rad[kJoint],
                  state.actuators.current_mA[kActuator]);
      std::fflush(stdout);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::printf("\n\n");

    // 5) The reached angle is the interesting column. Under position control it converges on
    //    the target; under impedance it settles short of it, by however much friction and the
    //    object take, and the effort is what stayed constant instead.
    std::printf("the target was the same both times, so the difference is the controller\n\n");

    // 6) Both command types are clamped and validated the same way — the projection into the
    //    reachable workspace applies to a joint command whichever controller consumes it.
    hand.set_command(Idle{});
    hand.stop();
    std::printf("done — the command struct picks the controller, not a mode setting\n");
  } catch (const Exception& error) {
    std::printf("\nfailed: %s: %s\n", to_string(error.code()), error.what());
    return 1;
  }

  return 0;
}
