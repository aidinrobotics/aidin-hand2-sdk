// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, homing and the reason every command waits for it
//
// Until homing succeeds there is no absolute reference, so the SDK refuses every command but
// Idle. This example sends one before homing to see that refusal, then homes, then sends the
// same command again — the printed WrongCallOrder and the printed success are the same call.
//
// home() blocks until it finishes, so the encoder counts are printed on either side of it
// rather than during. What they show is the reference being built: nothing meaningful before,
// the hard stop at -1000 straight after, and 0 once the fingers back off it.
//
// Run as: 04_homing [right|left] [interface=can0]
// Needs a real hand. home() drives every finger into its hard stop, so clear the space around
// the hand and keep your hands off it until the run finishes.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <csignal>
#include <string>
#include <thread>

#include <aidin_hand2/aidin_hand2.hpp>

using namespace aidin_hand2;

namespace
{
// SIGINT only asks the wait loop to stop. It cannot cut home() short — that call blocks inside
// the SDK — so Ctrl-C during homing takes effect once homing returns.
std::atomic<bool> g_shutdown{false};

// All 16 actuators are printed, eight to a row, because one row of sixteen would wrap on an
// 80-column terminal and a wrapped row is worse than a split one. Every column is the same
// width, so an index sits directly above its count.
constexpr int kFieldWidth    = 7;
constexpr int kColumnsPerRow = 8;
constexpr int kRuleWidth     = kFieldWidth * kColumnsPerRow;

void print_rule(char fill)
{
  for (int i = 0; i < kRuleWidth; ++i) std::putchar(fill);
  std::putchar('\n');
}

void print_section(const char* title)
{
  std::putchar('\n');
  print_rule('=');
  std::printf("%s\n", title);
  print_rule('-');
}

// position_count is a double because it is a measurement, but what the drive reports is a whole
// count, so the table prints it as one. A dotted rule separates the a0-a7 block from the
// a8-a15 one, so the two index rows do not read as one wrapped row.
void print_count_table(const HandState& state)
{
  for (std::size_t base = 0; base < kActuatorCount; base += kColumnsPerRow) {
    const std::size_t end = base + kColumnsPerRow;
    if (base > 0) print_rule('.');
    for (std::size_t a = base; a < end; ++a) {
      char index_label[8];
      std::snprintf(index_label, sizeof(index_label), "a%zu", a);
      std::printf("%*s", kFieldWidth, index_label);
    }
    std::putchar('\n');
    for (std::size_t a = base; a < end; ++a) {
      std::printf("%*lld", kFieldWidth,
                  static_cast<long long>(std::lround(state.actuators.position_count[a])));
    }
    std::putchar('\n');
  }
}
}  // namespace

int main(int argc, char** argv)
{
  std::signal(SIGINT, [](int) { g_shutdown.store(true); });

  const HandSide side = (argc > 1 && std::string(argv[1]) == "left") ? HandSide::Left : HandSide::Right;

  try {
    HandConfig config{argc > 2 ? argv[2] : "can0", side};
    config.control_rate = 500;    // Hz
    config.auto_home    = false;  // home() is called below, so run() must not do it as well

    HandManager manager;
    Hand hand = manager.create(config);
    hand.connect();

    // 1) Cap the current before anything moves. Homing pushes into a hard stop on purpose, and
    //    this is the cap it pushes with.
    hand.set_max_effort(600.0);  // % of rated current × 10, so 60 %

    std::printf("homing_state = %s\n\n", to_string(hand.get_diagnostics().homing_state));

    // 2) A joint target without a reference. The SDK refuses it, and the code says why:
    //    WrongCallOrder is the call sequence being wrong, not the target.
    JointPositionCommand zero_pose;
    zero_pose.target.fill(0.0);  // rad
    try {
      hand.set_command(zero_pose);
      std::printf("the command was accepted, which the SDK is not supposed to allow yet\n");
    } catch (const Exception& error) {
      std::printf("before homing — %s: %s\n\n", to_string(error.code()), error.what());
    }

    // 3) Idle is the exception. It needs no reference because it sends an effort of 0.
    //    It still needs the drives on, so run() comes first.
    hand.run();
    hand.set_command(Idle{});
    std::printf("Idle was accepted before homing, since it asks for no position\n\n");

    // 4) The counts before homing. They are whatever the drives happened to power up with.
    const HandState before = hand.get_state();
    print_section("counts before homing — no reference yet, so these mean nothing");
    print_count_table(before);

    // 5) home() blocks until every finger has reached its hard stop and the zero has been
    //    written. It takes a few seconds, and it enables the drives itself when they are off,
    //    so the run() above was for the Idle command rather than for this call.
    print_section("homing — every finger travels to its hard stop");
    hand.home();
    std::printf("home() returned, homing_state = %s\n\n", to_string(hand.get_diagnostics().homing_state));

    // 6) The counts after homing. They are now measured from the hard stops, which is what
    //    makes a joint target mean the same thing on every run. Every one of them reads -1000
    //    rather than 0: the SDK zero sits a fixed 1000 counts off the hard stop, and the fingers
    //    are still pressed into that stop at this instant. Step 8 is where they back off it.
    const HandState after = hand.get_state();
    print_section("counts after homing — the hard stop itself, so every one reads -1000");
    print_count_table(after);

    // 7) The same command as step 2, unchanged. Homing is the only thing that happened in
    //    between, and homing also left a command holding the zero pose, so this replaces it.
    hand.set_command(zero_pose);
    std::printf("after homing — the same command was accepted\n\n");

    // 8) Give the fingers 2 s to leave the hard stop, then print the counts once. This is the
    //    half of the reference the table in step 6 cannot show on its own: -1000 was the stop
    //    itself, and this is the SDK zero the joint target was resolved against.
    //
    //    The wait prints nothing. Redrawing a table in place would need the cursor moved back
    //    over it, and the RT logging thread writes to stderr at times of its own — one log line
    //    landing mid-table shifts everything down, and every redraw after it is off by a line.
    for (int i = 0; i < 20 && !g_shutdown.load(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    print_section("settled off the hard stop — the counts have reached the SDK zero");
    print_count_table(hand.get_state());

    // 9) A failed homing is a HardwareFault whose message names the actuator, and a fault later
    //    on costs the reference: reconnect() puts homing_state back to NotRun, so a recovered
    //    hand has to be homed again before it takes a command.
    hand.stop();
    std::printf("done — homing_state stays %s until a reconnect() clears it\n",
                to_string(hand.get_diagnostics().homing_state));
  } catch (const Exception& error) {
    // A HardwareFault here is most likely homing itself failing, and the message names the
    // actuator that did not reach its stop.
    std::printf("\nfailed: %s: %s\n", to_string(error.code()), error.what());
    return 1;
  }

  return 0;
}
