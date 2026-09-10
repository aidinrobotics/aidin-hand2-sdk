// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, converting between joint angles and encoder counts
//
// The two conversion functions are free functions. No Hand, no connection, no lifecycle: they
// are arithmetic on arrays, so the first half of this example runs with no hardware present
// and prints the same numbers either way.
//
// The sizes are the thing to watch. Inverse kinematics takes the 16 active joints and forward
// kinematics returns all 21, the five passive ones included, so a forward result cannot be
// assigned to a command's target — it is the wrong length, and the entries do not line up.
// The round trip below prints both arrays so the mismatch is visible rather than described.
//
// The second half connects, if a hand is there, to show that get_state().joints.position_rad
// is nothing more than fk_actuator_to_joint() applied to the encoder counts.
//
// Run as: 13_kinematics [right|left] [interface=can0]
// Runs without a hand: the conversion half needs no connection, and the comparison half says
// so and stops if there is nothing to connect to. Nothing moves either way.

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <csignal>
#include <string>

#include <aidin_hand2/aidin_hand2.hpp>

using namespace aidin_hand2;

namespace
{
// SIGINT is caught so that Ctrl-C leaves through the destructors below rather than killing the
// process mid-call. Nothing here loops for long, so the flag is only read on the way out.
std::atomic<bool> g_shutdown{false};
}  // namespace

int main(int argc, char** argv)
{
  std::signal(SIGINT, [](int) { g_shutdown.store(true); });

  const HandSide side = (argc > 1 && std::string(argv[1]) == "left") ? HandSide::Left : HandSide::Right;

  // 1) A pose in joint space, the same 16 active joints a command carries.
  const std::array<double, kActiveJointCount> pose = {
      0.20, 0.35, 0.10, 0.25,   // thumb   j0 j1 j2 j3
      0.08, 0.45, 0.30,         // index   j1 j2 j3
      0.04, 0.55, 0.40,         // middle
      -0.04, 0.65, 0.50,        // ring
      -0.08, 0.75, 0.60};       // baby

  // 2) Joint angles to encoder counts. The result is integer counts, because that is what goes
  //    into a CAN frame, and there are 16 of them — one per actuator.
  const std::array<int, kActuatorCount> encoder = ik_joint_to_actuator(pose);

  std::printf("ik_joint_to_actuator, %zu counts:\n ", kActuatorCount);
  for (std::size_t a = 0; a < kActuatorCount; ++a) {
    std::printf(" a%zu %7d", a, encoder[a]);
    if (a == 3 || a == 6 || a == 9 || a == 12) std::printf("\n ");
  }
  std::printf("\n\n");

  // 3) Back the other way. This returns 21 entries rather than 16: the four-bar coupled joint
  //    of each digit comes out too, and it was never an input.
  const std::array<double, kJointCount> joints = fk_actuator_to_joint(encoder);

  std::printf("fk_actuator_to_joint, %zu angles in rad:\n ", kJointCount);
  for (std::size_t j = 0; j < kJointCount; ++j) {
    std::printf(" j%zu %+.4f", j, joints[j]);
    if (j % 5 == 4) std::printf("\n ");
  }
  std::printf("\n\n");

  // 4) The two index spaces do not line up. The passive joint is the last slot of each finger
  //    group, so the 21-entry array carries one extra entry per digit and every active joint
  //    after the thumb sits further along than its own index. This is the mapping between them.
  constexpr std::array<std::size_t, kActiveJointCount> active_to_joint = {
      0,  1,  2,  3,   // thumb   active 0-3  -> joint 0-3,   joint 4  is passive
      5,  6,  7,       // index   active 4-6  -> joint 5-7,   joint 8  is passive
      9,  10, 11,      // middle  active 7-9  -> joint 9-11,  joint 12 is passive
      13, 14, 15,      // ring    active 10-12 -> joint 13-15, joint 16 is passive
      17, 18, 19};     // baby    active 13-15 -> joint 17-19, joint 20 is passive

  // 5) The round trip, read through that mapping. The active angles come back to where they
  //    started, and what bounds the deviation is the rounding to integer counts.
  double worst = 0.0;
  for (std::size_t j = 0; j < kActiveJointCount; ++j) {
    worst = std::fmax(worst, std::fabs(joints[active_to_joint[j]] - pose[j]));
  }
  std::printf("round trip through the mapping, worst deviation %.2e rad\n", worst);

  // 6) The same comparison made the tempting way, against the first 16 entries. It is wrong
  //    from the index finger onwards, and the number says how wrong: this is what truncating
  //    a forward result costs, and it is far too large to mistake for rounding.
  double truncated_worst = 0.0;
  for (std::size_t j = 0; j < kActiveJointCount; ++j) {
    truncated_worst = std::fmax(truncated_worst, std::fabs(joints[j] - pose[j]));
  }
  std::printf("round trip against the first %zu entries, worst deviation %.2e rad — wrong\n\n",
              kActiveJointCount, truncated_worst);

  // 7) The passive angles, which have no input to come back to. They follow from the four-bar
  //    linkage of each digit, so they are output only.
  std::printf("passive joints:");
  for (const std::size_t j : {4u, 8u, 12u, 16u, 20u}) std::printf("  j%zu %+.4f", j, joints[j]);
  std::printf("\n\n");

  // 8) So a forward result cannot fill a command target. The compiler stops the assignment
  //    because 21 and 16 differ, and copying the first 16 compiles while shifting every joint
  //    after the thumb — pick the entries out by index, as step 5 did.
  std::printf("drop the passive entries by index, do not truncate the array\n\n");

  // 9) Now the same conversion inside the SDK, if a hand is there to ask. This half needs a
  //    connection but still moves nothing: no run(), no home(), no set_command().
  if (g_shutdown.load()) return 0;

  try {
    HandConfig config{argc > 2 ? argv[2] : "can0", side};
    config.control_rate = 500;    // Hz
    config.auto_home    = false;

    HandManager manager;
    Hand hand = manager.create(config);
    hand.connect();

    const HandState state = hand.get_state();

    // 10) position_count is double, because it is a measurement, and the conversion takes int.
    //     Round rather than truncate, so a count of 8191.6 does not become 8191.
    std::array<int, kActuatorCount> measured{};
    for (std::size_t a = 0; a < kActuatorCount; ++a) {
      measured[a] = static_cast<int>(std::lround(state.actuators.position_count[a]));
    }

    const std::array<double, kJointCount> computed = fk_actuator_to_joint(measured);

    // 11) The SDK filled state.joints with this call, so the two columns agree except where a
    //     cycle landed between the read and the rounding.
    std::printf("SDK joints against the same call made here:\n");
    for (std::size_t j = 0; j < 5; ++j) {
      std::printf("  j%zu  state %+.6f   computed %+.6f   diff %+.2e rad\n",
                  j, state.joints.position_rad[j], computed[j],
                  computed[j] - state.joints.position_rad[j]);
    }
    std::printf("\ndone — the functions are the same ones the control loop uses\n");
  } catch (const Exception& error) {
    // No hand attached is the ordinary case for this half, so it is reported and not treated
    // as a failure of the example.
    std::printf("no hand to compare against — %s: %s\n", to_string(error.code()), error.what());
    std::printf("the conversion half above needed no connection\n");
  }

  return 0;
}
