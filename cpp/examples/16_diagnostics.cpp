// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, a monitor that reads Diagnostics and prints it
//
// This example only observes. It connects and reads, and it never enables the drives: no
// run(), no home(), no set_command(). Leave it running alongside another program, or on its
// own, and it reports what the control and communication loop is doing.
//
// The live display refreshes twice a second and covers the four loop fields, the deadline miss
// rate computed from their differences rather than from the totals, and any actuator fault.
// The last section prints the diagnostic record the guide asks for — the whole set of values
// worth keeping when something needs explaining afterwards.
//
// Pull the CAN cable while it runs and the lifecycle turns to Faulted about 100 ms later, at
// which point the cycle counter and the timestamp both stop advancing. That pair stopping
// together is the signal; neither one alone tells you frames are still arriving.
//
// Run as: 16_diagnostics [right|left] [interface=can0]
// Needs a real hand, and moves nothing. Ctrl-C prints the record and finishes.

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
// SIGINT only asks the display loop to stop, so the record below still gets printed and the
// HandManager destructor still closes the link.
std::atomic<bool> g_shutdown{false};
}  // namespace

int main(int argc, char** argv)
{
  std::signal(SIGINT, [](int) { g_shutdown.store(true); });

  const HandSide side = (argc > 1 && std::string(argv[1]) == "left") ? HandSide::Left : HandSide::Right;

  try {
    HandConfig config{argc > 2 ? argv[2] : "can0", side};
    config.control_rate = 500;    // Hz, so the loop period to compare against is 2 ms
    config.auto_home    = false;  // this example never leaves Connected

    HandManager manager;
    Hand hand = manager.create(config);
    hand.connect();

    std::printf("monitoring — pull the CAN cable to watch the lifecycle fault. Ctrl-C to finish.\n\n");

    // Previous totals, so the miss rate below is over the last interval rather than over the
    // whole session. A rate computed from the totals hides a burst that has since stopped.
    std::uint64_t previous_cycles = 0;
    std::uint64_t previous_misses = 0;

    while (!g_shutdown.load()) {
      // 1) One read of each, in that order, and everything below works from these two copies.
      //    Calling again mid-line would mix two cycles into one row.
      const Diagnostics diagnostics = hand.get_diagnostics();
      const HandState   state       = hand.get_state();

      // 2) The miss rate over this interval. deadline_misses counts cycles whose processing ran
      //    past the 2 ms period, and what matters is the trend rather than any single spike.
      const std::uint64_t delta_cycles = diagnostics.control_cycles - previous_cycles;
      const std::uint64_t delta_misses = diagnostics.deadline_misses - previous_misses;
      const double        miss_rate    = delta_cycles > 0
                                             ? static_cast<double>(delta_misses) / static_cast<double>(delta_cycles)
                                             : 0.0;
      previous_cycles = diagnostics.control_cycles;
      previous_misses = diagnostics.deadline_misses;

      // 3) The loop's four fields, plus the timestamp that should be advancing with them.
      std::printf("\r\033[K%-12s %-10s | cycles %10llu  miss %8llu (%.2f%% now)"
                  "  period %5.3f ms  compute %5.3f ms  ts %s",
                  to_string(diagnostics.lifecycle), to_string(diagnostics.homing_state),
                  static_cast<unsigned long long>(diagnostics.control_cycles),
                  static_cast<unsigned long long>(diagnostics.deadline_misses),
                  miss_rate * 100.0, diagnostics.last_period_ms, diagnostics.last_compute_ms,
                  state.timestamp == 0 ? "unset" : "advancing");
      std::fflush(stdout);

      // 4) A fault on any actuator, printed on its own line so it scrolls into the history
      //    instead of being overwritten. The loop keeps trying to reset a faulted actuator,
      //    and a faulted or disabled one is masked out of the command: its position setpoint
      //    is replaced by the measured position and its effort setpoint becomes 0.
      for (std::size_t a = 0; a < kActuatorCount; ++a) {
        const ActuatorFault fault = diagnostics.actuator_health.fault[a];
        if (fault != ActuatorFault::None) {
          std::printf("\n  actuator %2zu: %s, enabled = %s\n", a, to_string(fault),
                      diagnostics.actuator_health.enabled[a] ? "yes" : "no");
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    std::printf("\n\n");

    // 5) The diagnostic record. Read each buffer once, take a monotonic timestamp alongside
    //    them, and keep the lot together. The cycle counter is not an index the two reads
    //    share, so lining records up afterwards is done by these two clocks.
    const Diagnostics diagnostics = hand.get_diagnostics();
    const HandState   state       = hand.get_state();
    const auto        monotonic_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();

    std::printf("diagnostic record\n");
    std::printf("  monotonic_ns     %lld\n", static_cast<long long>(monotonic_ns));
    std::printf("  state.timestamp  %lld\n", static_cast<long long>(state.timestamp));
    std::printf("  lifecycle        %s\n", to_string(diagnostics.lifecycle));
    std::printf("  homing_state     %s\n", to_string(diagnostics.homing_state));
    std::printf("  nan_commands     %llu\n",
                static_cast<unsigned long long>(diagnostics.nan_command_count));
    std::printf("  control_cycles   %llu\n",
                static_cast<unsigned long long>(diagnostics.control_cycles));
    std::printf("  deadline_misses  %llu\n",
                static_cast<unsigned long long>(diagnostics.deadline_misses));
    std::printf("  last_period_ms   %.3f\n", diagnostics.last_period_ms);
    std::printf("  last_compute_ms  %.3f\n", diagnostics.last_compute_ms);

    // 6) The measurements, per actuator, with the joint angle beside them for the first 16.
    std::printf("  actuators        idx  position_cnt  velocity_rpm   current_mA  en fault\n");
    for (std::size_t a = 0; a < kActuatorCount; ++a) {
      std::printf("                   %3zu  %12.0f  %12.1f  %11.1f  %2s %s\n", a,
                  state.actuators.position_count[a], state.actuators.velocity_rpm[a],
                  state.actuators.current_mA[a],
                  diagnostics.actuator_health.enabled[a] ? "y" : "n",
                  to_string(diagnostics.actuator_health.fault[a]));
    }

    // 7) Joint positions, all 21 of them. The velocity and effort arrays are placeholders, so
    //    they are left out of the record rather than written down as measurements.
    std::printf("  joints rad      ");
    for (std::size_t j = 0; j < kJointCount; ++j) {
      std::printf(" %+.4f", state.joints.position_rad[j]);
      if (j % 7 == 6) std::printf("\n                  ");
    }
    std::printf("\n");

    // 8) The command echo. monostate has to be recorded as its own case: writing it down as 0
    //    makes "outside a control session" and "commanded zero torque" the same entry, and
    //    they are not the same thing at all.
    std::printf("  selected_source  %s\n",
                state.commanded.selected_source == CommandSource::Controller ? "Controller"
                : state.commanded.selected_source == CommandSource::QuickStop ? "QuickStop"
                : state.commanded.selected_source == CommandSource::Homing    ? "Homing"
                                                                              : "None");
    if (std::holds_alternative<std::monostate>(state.commanded.controller_output)) {
      std::printf("  controller_output monostate — no setpoint, which is not a setpoint of 0\n");
    } else if (const auto* position =
                   std::get_if<ActuatorPositionSetpoint>(&state.commanded.controller_output)) {
      std::printf("  controller_output position, a0 %.0f cnt\n", position->target_position_cnt[0]);
    } else if (const auto* effort =
                   std::get_if<ActuatorEffortSetpoint>(&state.commanded.controller_output)) {
      std::printf("  controller_output effort, a0 %.1f\n", effort->target_effort_pct[0]);
    }
    std::printf("  max_effort_pct   a0 %.0f\n", state.commanded.max_effort_pct[0]);

    // 9) ControllerConfig has no getter, so the application keeps its own copy of whatever it
    //    last set and writes that into the record. Nothing here has set one, so there is
    //    nothing to write, and the defaults are what were in force.
    std::printf("  controller_config not set by this program, so the defaults applied\n");

    // 10) disabled_actuators belongs in the record too: an actuator on that list reports no
    //     fault because it is not reported on at all, which reads exactly like a healthy one.
    std::printf("  disabled_actuators none\n");
    std::printf("\ndone\n");
  } catch (const Exception& error) {
    std::printf("\nfailed: %s: %s\n", to_string(error.code()), error.what());
    return 1;
  }

  return 0;
}
