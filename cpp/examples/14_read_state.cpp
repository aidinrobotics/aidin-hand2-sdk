// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, reading HandState and knowing what each field is worth
//
// get_state() returns one snapshot: the actuator measurements, the joint angles derived from
// them, the tactile readings, and the command as it was applied this cycle. It is a copy, so
// it does not change underneath you, and reading it does not need a command to be running.
//
// Three things about it are easy to get wrong, so the example prints each one:
//   - two fields are still placeholders, and they read 0 rather than reading nothing;
//   - the command echo is a variant that holds nothing at all outside a control session;
//   - get_state() and get_diagnostics() are separate reads, so pairing values across the two
//     means reading each once and reusing the result.
//
// Run as: 14_read_state [right|left] [interface=can0]
// Needs a real hand. home() drives every finger into its hard stop, and the index finger then
// flexes so the command echo has something in it.

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

  // The index finger, named once per index space. commanded.controller_input is indexed by
  // active joint, joints is indexed by joint, and actuators and controller_output are indexed
  // by actuator. The first two differ because each finger's passive joint takes the last slot
  // of its group, so active 5 is joint 6; the third is a separate space altogether.
  constexpr std::size_t kActive   = 5;
  constexpr std::size_t kJoint    = 6;
  constexpr std::size_t kActuator = 5;

  try {
    HandConfig config{argc > 2 ? argv[2] : "can0", side};
    config.control_rate = 500;    // Hz
    config.auto_home    = false;

    HandManager manager;
    Hand hand = manager.create(config);

    // 1) Before connect() there is no loop, so the snapshot is the default one. timestamp is 0,
    //    which is how "no cycle has stamped this yet" is spelled — there is no separate flag.
    const HandState fresh = hand.get_state();
    std::printf("before connect: timestamp %lld, a0 %.0f cnt\n\n",
                static_cast<long long>(fresh.timestamp), fresh.actuators.position_count[0]);

    hand.connect();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 2) Connected is a read-only state: the loop is reading frames and no torque is applied.
    //    The measurements are live, so a finger moved by hand shows up here.
    const HandState observing = hand.get_state();
    std::printf("connected: timestamp %lld ns since the epoch\n", static_cast<long long>(observing.timestamp));

    // 3) The command echo is empty in this state. controller_output is a variant, and outside a
    //    control session it holds std::monostate — not a setpoint of zeros, but no setpoint.
    //    Testing for it with get_if is what separates the two cases.
    std::printf("controller_output holds %s, selected_source = %s\n\n",
                std::holds_alternative<std::monostate>(observing.commanded.controller_output)
                    ? "monostate (nothing)" : "a setpoint",
                observing.commanded.selected_source == CommandSource::None ? "None" : "something");

    hand.set_max_effort(600.0);
    hand.run();
    hand.home();

    JointPositionCommand flex;
    flex.target.fill(0.0);
    flex.target[kActive] = 35.0 * 3.14159265358979323846 / 180.0;  // rad, index finger flexion
    hand.set_command(flex);

    // 4) The measurements, live. actuators is what the drives reported and joints is the
    //    forward kinematics of it, which is why one array is 16 long and the other is 21.
    std::printf("actuators (%zu) and joints (%zu), live:\n", kActuatorCount, kJointCount);
    for (int i = 0; i < 25 && !g_shutdown.load(); ++i) {
      const HandState state = hand.get_state();
      std::printf("\r\033[K  actuator %zu %8.0f cnt  %+7.1f rpm  %+7.1f mA   |   joint %zu %+.4f rad",
                  kActuator, state.actuators.position_count[kActuator],
                  state.actuators.velocity_rpm[kActuator], state.actuators.current_mA[kActuator],
                  kJoint, state.joints.position_rad[kJoint]);
      std::fflush(stdout);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::printf("\n\n");

    // 5) The two placeholder fields. They are 0 because nothing computes them yet, not because
    //    the joint is still — the actuator velocity above was not 0 while this one was. Do not
    //    record them as measurements.
    const HandState settled = hand.get_state();
    std::printf("joints.velocity_rad_s[%zu] = %.1f and joints.effort_Nm[%zu] = %.1f — placeholders\n",
                kJoint, settled.joints.velocity_rad_s[kJoint],
                kJoint, settled.joints.effort_Nm[kJoint]);
    std::printf("actuators.velocity_rpm[%zu] = %+.1f is the measurement to use instead\n\n",
                kActuator, settled.actuators.velocity_rpm[kActuator]);

    // 6) The command echo, now that a command is running. The three fields are the stages of
    //    one cycle: what was asked for, what came out of the controller, and which path
    //    actually reached the motors.
    if (const auto* input = std::get_if<JointPositionCommand>(&settled.commanded.controller_input)) {
      std::printf("controller_input: JointPositionCommand, active %zu target %+.4f rad\n",
                  kActive, input->target[kActive]);
    }
    if (const auto* output =
            std::get_if<ActuatorPositionSetpoint>(&settled.commanded.controller_output)) {
      // Compare like against like: this setpoint is in counts, so its partner is
      // actuators.position_count, and subtracting the rad target from it would mean nothing.
      std::printf("controller_output: actuator %zu target %8.0f cnt, measured %8.0f cnt, error %+.0f\n",
                  kActuator, output->target_position_cnt[kActuator],
                  settled.actuators.position_count[kActuator],
                  output->target_position_cnt[kActuator] - settled.actuators.position_count[kActuator]);
    }
    std::printf("selected_source = %s, max_effort_pct[%zu] = %.0f\n\n",
                settled.commanded.selected_source == CommandSource::Controller ? "Controller"
                : settled.commanded.selected_source == CommandSource::Homing   ? "Homing"
                : settled.commanded.selected_source == CommandSource::QuickStop ? "QuickStop"
                                                                                : "None",
                kActuator, settled.commanded.max_effort_pct[kActuator]);

    // 7) A setpoint being present does not mean it reached the motors. Homing and the quick
    //    stop both take the path over, so check selected_source before reading the output as
    //    what the hand is doing.
    std::printf("a setpoint with selected_source other than Controller did not reach the motors\n\n");

    // 8) The two reads are separate. Each returns the latest of its own buffer, so a cycle can
    //    land between them: read each once, then work from those two copies rather than
    //    calling again inside the arithmetic.
    const HandState      state       = hand.get_state();
    const Diagnostics    diagnostics = hand.get_diagnostics();
    const std::chrono::steady_clock::time_point read_at = std::chrono::steady_clock::now();

    std::printf("one read of each: %llu cycles, last period %.3f ms, timestamp %lld\n",
                static_cast<unsigned long long>(diagnostics.control_cycles),
                diagnostics.last_period_ms, static_cast<long long>(state.timestamp));

    // 9) timestamp is CLOCK_REALTIME, for lining up with a recording. How stale a snapshot is
    //    is the application's to track, with its own monotonic clock, as here: nothing in
    //    HandState answers that question.
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - read_at);
    std::printf("that snapshot is now %lld ms old by our own clock\n\n",
                static_cast<long long>(age.count()));

    hand.set_command(Idle{});
    hand.stop();
    std::printf("done — the snapshot is a copy, so it stays put while the loop moves on\n");
  } catch (const Exception& error) {
    std::printf("\nfailed: %s: %s\n", to_string(error.code()), error.what());
    return 1;
  }

  return 0;
}
