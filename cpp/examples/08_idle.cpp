// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, Idle, and how long a command lasts
//
// Idle is the empty command: no target field, no controller, an actuator effort of 0 sent every
// cycle. The drives stay enabled through it, so a finger moved by hand still meets resistance.
// That is what separates it from stop(), which takes the torque away altogether, and the two
// are printed one after the other here so the difference is a number rather than a claim.
//
// Idle is also where the SDK lands on its own. A command only lives while the lifecycle is
// Running, and the last part of this example walks the two paths back into Running to print
// which command each one starts from.
//
// Run as: 08_idle [right|left] [interface=can0]
// Needs a real hand. home() drives every finger into its hard stop. Push a finger around
// during the Idle passes to feel the resistance the printed numbers are describing.

#include <array>
#include <atomic>
#include <chrono>
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

  try {
    HandConfig config{argc > 2 ? argv[2] : "can0", side};
    config.control_rate = 500;    // Hz
    config.auto_home    = false;

    HandManager manager;
    Hand hand = manager.create(config);
    hand.connect();
    hand.set_max_effort(600.0);

    // 1) Idle is refused outside Running like every other command: the drives have to be on
    //    for an effort of 0 to mean anything.
    try {
      hand.set_command(Idle{});
      std::printf("Idle was accepted in Connected, which the SDK is not supposed to allow\n");
    } catch (const Exception& error) {
      std::printf("in Connected — %s: %s\n\n", to_string(error.code()), error.what());
    }

    // 2) In Running it goes through even before homing. Every other command needs
    //    homing_state to be Succeeded first, because it needs an absolute reference; Idle
    //    asks for no position, so it needs none.
    hand.run();
    hand.set_command(Idle{});
    std::printf("in Running before homing — accepted, mode = %s\n\n",
                hand.get_command_mode() == CommandMode::Idle ? "Idle" : "not Idle");

    // 3) Idle produces an effort setpoint of 0 rather than no setpoint at all. The distinction
    //    matters when reading state back: the variant holds ActuatorEffortSetpoint, and
    //    selected_source stays Controller, so the SDK is still commanding the hand.
    std::printf("Idle with the drives on — push a finger and it resists\n");
    for (int i = 0; i < 25 && !g_shutdown.load(); ++i) {
      const HandState state = hand.get_state();
      const auto* setpoint = std::get_if<ActuatorEffortSetpoint>(&state.commanded.controller_output);
      std::printf("\r\033[K  source %-10s  effort a0 %6.1f  |  measured a0 %6.1f mA  vel %6.1f rpm",
                  state.commanded.selected_source == CommandSource::Controller ? "Controller" : "other",
                  setpoint != nullptr ? setpoint->target_effort_pct[0] : 0.0,
                  state.actuators.current_mA[0], state.actuators.velocity_rpm[0]);
      std::fflush(stdout);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::printf("\n\n");

    // 4) stop() is the other way to get to zero torque, and it is not the same thing. The
    //    drives go to quick stop, so nothing is being commanded any more: selected_source
    //    changes and the finger goes slack.
    hand.stop();
    std::printf("after stop() — push the same finger and it moves freely\n");
    for (int i = 0; i < 25 && !g_shutdown.load(); ++i) {
      const HandState state = hand.get_state();
      std::printf("\r\033[K  source %-10s  |  measured a0 %6.1f mA  vel %6.1f rpm",
                  state.commanded.selected_source == CommandSource::QuickStop ? "QuickStop"
                  : state.commanded.selected_source == CommandSource::Controller ? "Controller"
                                                                                 : "other",
                  state.actuators.current_mA[0], state.actuators.velocity_rpm[0]);
      std::fflush(stdout);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::printf("\n\n");

    // 5) Command lifetime. A command lives until the next set_command(), but leaving Running
    //    ends it, because Running is the only state that accepts one. Coming back, the SDK
    //    fills in a first command rather than leaving the drives uncommanded.
    //
    //    Homing is one such path, and it lands on an ActuatorPositionCommand of all zeros —
    //    the pose it just finished defining.
    hand.home();
    std::printf("back in Running by homing, mode = %s\n",
                hand.get_command_mode() == CommandMode::ActuatorPosition ? "ActuatorPosition"
                                                                         : "something else");

    // 6) A joint target, so there is something other than the SDK's fill-in to lose.
    JointPositionCommand flex;
    flex.target.fill(0.0);
    // Active joint 5, one of the index finger's flexion joints. The command's index space is
    // the active-joint one: 4 entries for the thumb, then 3 per long finger.
    flex.target[5] = 30.0 * 3.14159265358979323846 / 180.0;  // rad
    hand.set_command(flex);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    std::printf("sent a joint target, mode = %s\n",
                hand.get_command_mode() == CommandMode::JointPosition ? "JointPosition"
                                                                      : "something else");

    // 7) stop() then run() is the other path. It holds the pose it resumed at rather than
    //    reinstating the joint target, so the flexed pose stays but the command behind it does
    //    not — send the pose you want again after every return to Running.
    hand.stop();
    hand.run();
    std::printf("stopped and ran again, mode = %s — the joint target is gone\n\n",
                hand.get_command_mode() == CommandMode::ActuatorPosition ? "ActuatorPosition"
                : hand.get_command_mode() == CommandMode::Idle           ? "Idle"
                                                                         : "something else");

    // 8) Everything else — a reconnect() out of Faulted, for one — lands on Idle, which is
    //    zero effort with the drives on rather than a pose to hold.
    hand.set_command(Idle{});
    hand.stop();
    std::printf("done — Idle keeps the drives on, stop() does not\n");
  } catch (const Exception& error) {
    std::printf("\nfailed: %s: %s\n", to_string(error.code()), error.what());
    return 1;
  }

  return 0;
}
