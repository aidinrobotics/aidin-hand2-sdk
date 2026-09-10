// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, ActuatorPositionCommand, the target that goes straight to the drives
//
// This command is 16 encoder counts, and no controller touches them: no deadband, no filter,
// no inverse kinematics, and — the part worth pausing on — no workspace clamp. A joint command
// past a joint limit is quietly projected back into reach. An actuator count past the same
// limit is sent as written.
//
// So the targets here are derived from the measured counts rather than typed in, which is the
// safe way to use this command: read where the actuator is, offset from there. The example
// then sends a count past the int32 range to show the one value check that does apply.
//
// Run as: 11_actuator_position [right|left] [interface=can0]
// Needs a real hand. home() drives every finger into its hard stop, and the index finger then
// steps away from its homed count and back.

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
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

  // One of the index finger's three actuators, which are 4, 5 and 6. Everything this example
  // prints is in actuator space, so no index mapping comes up.
  constexpr std::size_t kActuator = 5;

  try {
    HandConfig config{argc > 2 ? argv[2] : "can0", side};
    config.control_rate = 500;    // Hz
    config.auto_home    = false;

    HandManager manager;
    Hand hand = manager.create(config);
    hand.connect();

    // 1) The cap earns its keep here. Nothing between this command and the drives checks
    //    whether the count is reachable, so the cap is the one thing that keeps a count past
    //    the mechanical stop from being pushed for as hard as the motor can manage.
    hand.set_max_effort(400.0);

    hand.run();
    hand.home();

    // 2) Read where the actuators are. After homing these are measured from the hard stops,
    //    so they are a reference the next run will agree with.
    const HandState homed = hand.get_state();
    std::printf("counts after homing:  a0 %8.0f  a5 %8.0f  a8 %8.0f\n\n",
                homed.actuators.position_count[0], homed.actuators.position_count[kActuator],
                homed.actuators.position_count[8]);

    // 3) Build the command from those counts. Every actuator is told to stay where it is, and
    //    then one of them is offset — the rest have to be filled in, because the command is
    //    the whole 16-entry array every time rather than a change to some of it.
    ActuatorPositionCommand command;
    for (std::size_t a = 0; a < kActuatorCount; ++a) {
      command.target[a] = homed.actuators.position_count[a];
    }

    // 4) Step the one actuator out and back, twice. A count is a fraction of a millimetre of
    //    screw travel rather than an angle, and on a long finger 8192 of them make 1 mm, so
    //    these two offsets are 1 mm and 2 mm. The thumb screws have a finer lead and take
    //    16384 counts per millimetre, which is why an offset means different travel there.
    const double base = command.target[kActuator];
    for (const double offset : {8192.0, 0.0, 16384.0, 0.0}) {
      if (g_shutdown.load()) break;
      command.target[kActuator] = base + offset;
      hand.set_command(command);

      for (int i = 0; i < 15 && !g_shutdown.load(); ++i) {
        const HandState state = hand.get_state();
        const auto* setpoint =
            std::get_if<ActuatorPositionSetpoint>(&state.commanded.controller_output);
        std::printf("\r\033[K  asked %9.0f  ->  setpoint %9.0f  reached %9.0f cnt  |  %6.1f mA",
                    command.target[kActuator],
                    setpoint != nullptr ? setpoint->target_position_cnt[kActuator] : 0.0,
                    state.actuators.position_count[kActuator],
                    state.actuators.current_mA[kActuator]);
        std::fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      std::printf("\n");
    }
    std::printf("\n");

    // 5) The setpoint column matched the asked column on the first cycle, with none of the
    //    ramping a joint position command shows. That is the absent controller: what you send
    //    is what the drive is told.
    std::printf("the setpoint equals the target immediately — no filter sits in between\n\n");

    // 6) The one check that does apply. A count outside the int32 range cannot be put in a CAN
    //    frame, so the command is dropped and the previous one keeps going out. Like a NaN, it
    //    does not throw: the counter is the only report.
    const std::uint64_t before = hand.get_diagnostics().nan_command_count;

    ActuatorPositionCommand too_far = command;
    too_far.target[kActuator] = 3.0e9;  // past INT32_MAX, which is about 2.1e9
    hand.set_command(too_far);          // returns normally
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    std::printf("sent 3.0e9 counts: no exception, nan_command_count %llu -> %llu\n",
                static_cast<unsigned long long>(before),
                static_cast<unsigned long long>(hand.get_diagnostics().nan_command_count));
    std::printf("still holding %9.0f cnt, which is the previous command\n\n",
                hand.get_state().actuators.position_count[kActuator]);

    // 7) A count inside the int32 range but outside the mechanical travel passes every check
    //    there is. Convert a joint target with ik_joint_to_actuator() when you want the
    //    workspace model applied, or keep the offsets small and measured, as above.
    hand.set_command(Idle{});
    hand.stop();
    std::printf("done — this command is not workspace-clamped, so the limits are yours to hold\n");
  } catch (const Exception& error) {
    std::printf("\nfailed: %s: %s\n", to_string(error.code()), error.what());
    return 1;
  }

  return 0;
}
