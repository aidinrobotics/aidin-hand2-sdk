// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, ActuatorEffortCommand, torque with no position in it at all
//
// The target is 16 currents in units of 0.1 % of rated, and the sign is the direction. Rated
// is 400 mA on every motor, so 300 means 30 % and about 120 mA of pull.
//
// Nothing here is a position. There is no target to converge on and no error to close, so the
// finger keeps moving in the commanded direction until something stops it — the mechanical
// stop, an object, or your hand. Watch the velocity column: under a position command it
// returns to 0 when the target is reached, and under this one it returns to 0 only when the
// finger is blocked.
//
// The example then asks for more than the cap allows, to show which of the two wins.
//
// Run as: 12_actuator_effort [right|left] [interface=can0]
// Needs a real hand. home() drives every finger into its hard stop, and the index finger then
// pulls in each direction until it reaches its own stops. Keep the cap low, as it is here.

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

    // 1) Set the cap first and keep it low. An effort command runs until something stops it,
    //    so the cap is what decides how hard it arrives there.
    hand.set_max_effort(400.0);

    hand.run();
    hand.home();

    // 2) Every actuator gets an entry, and the ones that should not move get 0. An effort of 0
    //    is not a brake: the actuator is free, and gravity or a nudge will move it.
    ActuatorEffortCommand command;
    command.target.fill(0.0);

    // 3) Pull one way, then the other, then release. The sign is the only thing that changes
    //    direction — there is no separate field for it.
    for (const double effort : {250.0, 0.0, -250.0, 0.0}) {
      if (g_shutdown.load()) break;
      command.target[kActuator] = effort;
      hand.set_command(command);

      for (int i = 0; i < 20 && !g_shutdown.load(); ++i) {
        const HandState state = hand.get_state();
        const auto* setpoint =
            std::get_if<ActuatorEffortSetpoint>(&state.commanded.controller_output);
        std::printf("\r\033[K  asked %+7.1f  ->  setpoint %+7.1f  |  %8.0f cnt  %+7.1f rpm  %+7.1f mA",
                    effort, setpoint != nullptr ? setpoint->target_effort_pct[kActuator] : 0.0,
                    state.actuators.position_count[kActuator],
                    state.actuators.velocity_rpm[kActuator],
                    state.actuators.current_mA[kActuator]);
        std::fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      std::printf("\n");
    }
    std::printf("\n");

    // 4) The cap against the command. 1500 is asked for with a cap of 400, and the setpoint
    //    column shows which one reached the drive.
    command.target[kActuator] = 1500.0;
    hand.set_command(command);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    const HandState capped = hand.get_state();
    const auto* capped_setpoint =
        std::get_if<ActuatorEffortSetpoint>(&capped.commanded.controller_output);
    std::printf("asked for 1500 under a cap of %.0f, setpoint %+.1f\n",
                capped.commanded.max_effort_pct[kActuator],
                capped_setpoint != nullptr ? capped_setpoint->target_effort_pct[kActuator] : 0.0);

    // 5) Raise the cap and nothing else. The command has not been resent, so this is the cap
    //    alone deciding what the same stored command produces.
    hand.set_max_effort(800.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    const HandState raised = hand.get_state();
    const auto* raised_setpoint =
        std::get_if<ActuatorEffortSetpoint>(&raised.commanded.controller_output);
    std::printf("same command, cap raised to %.0f, setpoint %+.1f\n\n",
                raised.commanded.max_effort_pct[kActuator],
                raised_setpoint != nullptr ? raised_setpoint->target_effort_pct[kActuator] : 0.0);

    // 6) Release before finishing. Idle is the same zero effort as filling the array with 0,
    //    and it says so more plainly.
    hand.set_max_effort(400.0);
    hand.set_command(Idle{});
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 7) The setpoint is in 0.1 % of rated and the measurement is in mA, so the two columns
    //    above are not comparable term by term. The current loop that connects them belongs to
    //    the drive, and a blocked finger draws its cap while a free one draws much less.
    hand.stop();
    std::printf("done — the cap bounds the command, and the command carries the sign\n");
  } catch (const Exception& error) {
    std::printf("\nfailed: %s: %s\n", to_string(error.code()), error.what());
    return 1;
  }

  return 0;
}
