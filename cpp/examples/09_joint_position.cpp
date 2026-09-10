// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, JointPositionCommand, and the three checks a command passes
//
// The command is 16 joint angles in rad and nothing else. What happens to them is worth
// knowing before trusting them: set_command() checks the state, checks the values, and
// projects the target into the reachable workspace, and only the first of those three throws.
//
// So this example sends a good target, then a target past every joint limit, then a target
// with a NaN in it. The first moves the hand, the second moves it to the clamped pose without
// complaining, and the third is dropped in silence — the only trace is a counter, which is
// printed. An application that only catches exceptions never learns about the third one.
//
// Run as: 09_joint_position [right|left] [interface=can0]
// Needs a real hand. home() drives every finger into its hard stop, and the fingers then flex
// and extend repeatedly, so clear the space around the hand.

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
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

    // 1) A grasp pose. The 16 entries are the active joints, in the order the whole SDK uses:
    //    the thumb's four first, then three per long finger. The first entry of each long
    //    finger group is its abduction joint.
    JointPositionCommand grasp;
    grasp.target = {
        0.20, 0.35, 0.10, 0.25,   // thumb   j0 j1 j2 j3
        0.08, 0.45, 0.30,         // index   j1 j2 j3
        0.04, 0.55, 0.40,         // middle
        -0.04, 0.65, 0.50,        // ring
        -0.08, 0.75, 0.60};       // baby

    JointPositionCommand open;
    open.target.fill(0.0);

    // 2) Reading the result back means crossing index spaces. The command is indexed by active
    //    joint and state.joints is indexed by joint, and the two differ because the passive
    //    joint of each finger takes the last slot of its group. So active 5 is joint 6, and
    //    comparing entry 5 against entry 5 would be comparing two different axes.
    constexpr std::array<std::size_t, kActiveJointCount> active_to_joint = {
        0,  1,  2,  3,   // thumb   active 0-3  -> joint 0-3
        5,  6,  7,       // index   active 4-6  -> joint 5-7
        9,  10, 11,      // middle  active 7-9  -> joint 9-11
        13, 14, 15,      // ring    active 10-12 -> joint 13-15
        17, 18, 19};     // baby    active 13-15 -> joint 17-19

    // 3) Alternate between the two poses twice, printing how the joints follow. The reached
    //    column is the forward kinematics result the SDK computed from the encoders, so it is
    //    where the hand got to rather than what was asked for.
    for (int cycle = 0; cycle < 2 && !g_shutdown.load(); ++cycle) {
      for (const JointPositionCommand* pose : {&grasp, &open}) {
        if (g_shutdown.load()) break;
        hand.set_command(*pose);
        for (int i = 0; i < 15 && !g_shutdown.load(); ++i) {
          const HandState state = hand.get_state();
          std::printf("\r\033[K  active 5 asked %+.3f reached %+.3f  |  active 8 asked %+.3f reached %+.3f rad",
                      pose->target[5], state.joints.position_rad[active_to_joint[5]],
                      pose->target[8], state.joints.position_rad[active_to_joint[8]]);
          std::fflush(stdout);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
    }
    std::printf("\n\n");

    // 4) The workspace clamp. clamp() applies the projection to your own copy, which is the
    //    only way to see it before sending: set_command() runs the same projection internally
    //    and does not report what it changed.
    JointPositionCommand over;
    over.target.fill(3.0);  // rad, far past every joint limit
    const std::array<double, kActiveJointCount> asked = over.target;
    over.clamp();
    std::printf("clamp() on a target of 3.0 rad everywhere, first 8 active joints:\n ");
    for (std::size_t j = 0; j < 8; ++j) std::printf("  %zu %+.3f", j, over.target[j]);
    std::printf("\n  asked for %+.3f, and no exception was thrown\n\n", asked[0]);

    // 5) Sending the unclamped target does the same thing without telling you. The pose the
    //    hand takes is the clamped one, so a target past the limits is not an error to handle
    //    but a target you did not choose.
    JointPositionCommand unclamped;
    unclamped.target.fill(3.0);
    hand.set_command(unclamped);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    const HandState clamped_state = hand.get_state();
    std::printf("sent 3.0 rad unclamped, active 5 reached %+.3f rad\n\n",
                clamped_state.joints.position_rad[active_to_joint[5]]);

    // 6) The value check. A NaN in the target is not clamped and does not throw: the command is
    //    dropped and the previous one keeps going out every cycle. The hand holds the clamped
    //    pose from step 4 rather than doing anything sudden.
    const std::uint64_t before = hand.get_diagnostics().nan_command_count;

    JointPositionCommand broken;
    broken.target.fill(0.0);
    broken.target[5] = std::nan("");
    hand.set_command(broken);  // returns normally
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    const Diagnostics diagnostics = hand.get_diagnostics();
    std::printf("sent a NaN target: no exception, nan_command_count %llu -> %llu\n",
                static_cast<unsigned long long>(before),
                static_cast<unsigned long long>(diagnostics.nan_command_count));
    std::printf("the hand still holds the previous pose, active 5 at %+.3f rad\n",
                hand.get_state().joints.position_rad[active_to_joint[5]]);
    std::printf("so check a computed target for finiteness, or watch that counter\n\n");

    // 7) The clamp is a reachability check and nothing more. Self-collision, the objects
    //    around the hand, the cable and the payload are all the application's to check.
    hand.set_command(open);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    hand.set_command(Idle{});
    hand.stop();
    std::printf("done — of the three checks, only the state check throws\n");
  } catch (const Exception& error) {
    // A WrongCallOrder here is the state check: not Running, or not homed.
    std::printf("\nfailed: %s: %s\n", to_string(error.code()), error.what());
    return 1;
  }

  return 0;
}
