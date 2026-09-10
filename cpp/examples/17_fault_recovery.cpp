// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, catching an Exception and getting back out of Faulted
//
// create -> connect -> run -> home -> watch the lifecycle -> reconnect -> run -> home -> resend
// Every failure arrives as one type, Exception, and its code() says what to check;
// reconnect() is the only public way out of Faulted, and it costs the homing and the command
//
// Run as: 17_fault_recovery [right|left] [interface=can0]
// Needs a real hand, and home() drives every finger into its hard stop, so clear the space
// around the hand before running. Nothing here provokes a fault on purpose: to watch the
// recovery path, unplug the CAN cable or switch the hand power off while the loop is running,
// then put it back within the retry budget

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <csignal>
#include <string>
#include <thread>

#include <aidin_hand2/aidin_hand2.hpp>

using namespace aidin_hand2;

namespace
{
// SIGINT only asks the tick loop to stop. The HandManager destructor is what confirms the
// quick stop and closes the link, so the point is to leave the loop and reach that destructor
std::atomic<bool> g_shutdown{false};
}  // namespace

// The three ErrorCode groups of error.hpp, so a caught code says where to look first
const char* what_to_check(ErrorCode code)
{
  switch (code) {
    case ErrorCode::None:
      return "nothing, no failure";

    // Fix the call sequence or the arguments: the calling code is wrong
    case ErrorCode::InvalidArgument:
    case ErrorCode::WrongCallOrder:
      return "this program: fix the argument or the call order";

    // Check the environment or the device: the cable, the power, the drives
    case ErrorCode::InterfaceUnavailable:
    case ErrorCode::CommunicationLost:
    case ErrorCode::HardwareFault:
      return "the hardware: CAN interface, cable, hand power, drive faults";

    // Report the SDK: the loop or an unforeseen path failed, not the caller
    case ErrorCode::ControlLoopFault:
    case ErrorCode::UnexpectedError:
      return "the SDK: report it with the log";
  }
  return "the SDK: report it with the log";
}

int main(int argc, char** argv)
{
  std::signal(SIGINT, [](int) { g_shutdown.store(true); });

  const HandSide side = (argc > 1 && std::string(argv[1]) == "left") ? HandSide::Left : HandSide::Right;
  HandConfig config{argc > 2 ? argv[2] : "can0", side};
  config.control_rate = 500;  // Hz

  // Off, because home() is called explicitly below; with true it is run() that homes
  config.auto_home    = false;

  // Off on purpose, so the recovery is visible in this file: with true the RT path retries
  // the link by itself, over and over, and the app never sees Faulted or calls reconnect()
  config.auto_reconnect = false;

  // A budget for the whole session, not per fault, so a flapping link ends the run instead
  // of recovering forever
  constexpr int kMaxRecoveryAttempts = 3;

  try {
    // 1) HandManager owns the resources and hands back a non-owning Hand
    HandManager manager;
    Hand hand = manager.create(config);
    hand.connect();

    // 2) Effort limit, percent of rated current
    //    This one setting survives reconnect(), which is why it is not set again below
    hand.set_max_effort(600.0);

    // 3) Enable the drives, then home; nothing but a homed hand takes a joint target
    hand.run();
    hand.home();

    // 4) The command the loop holds, and the same one a recovery has to send again
    //    Active joint index: T[0..3] I[4..6] M[7..9] R[10..12] B[13..15]
    //    The abduction joints stay at 0, being the coupled axes of the parallel wrist
    constexpr std::array<bool, kActiveJointCount> is_abduction = {
        false, false, true,  false,   // thumb:  j0 j1 [j2=abd] j3
        true,  false, false,          // index:  [j1=abd] j2 j3
        true,  false, false,          // middle: [j1=abd] j2 j3
        true,  false, false,          // ring:   [j1=abd] j2 j3
        true,  false, false};         // baby:   [j1=abd] j2 j3
    constexpr double kHoldRad = 15.0 * 3.14159265358979323846 / 180.0;

    JointPositionCommand cmd;
    cmd.target.fill(0.0);
    for (std::size_t j = 0; j < kActiveJointCount; ++j)
      if (!is_abduction[j]) cmd.target[j] = kHoldRad;
    hand.set_command(cmd);

    // 5) Hold that pose for 10 s, checking the lifecycle every 100 ms
    //    get_diagnostics() keeps reading in Faulted, so the loop sees the fault without
    //    having to call something that throws
    constexpr int kTicks = 100;
    int attempts = 0;

    for (int tick = 0; tick < kTicks && !g_shutdown.load(); ++tick) {
      const Diagnostics diag = hand.get_diagnostics();

      if (diag.lifecycle == HandLifecycle::Faulted) {
        // From here on run(), home() and set_command() all refuse, and the refusal does
        // not come back as WrongCallOrder: it carries the cause that stopped the hand,
        // CommunicationLost for a dead link or ControlLoopFault for a dead loop
        std::printf("[%3d] Faulted, homing_state=%s, recovering\n", tick,
                    to_string(diag.homing_state));

        if (++attempts > kMaxRecoveryAttempts) {
          std::printf("giving up after %d recovery attempts, the hand stays Faulted\n",
                      kMaxRecoveryAttempts);
          flush_log();
          return 1;
        }

        try {
          // a) reconnect() is the only way out of Faulted, and Faulted is the only state it is
          //    allowed from: it closes the socket, opens it again and restarts the loop, so it
          //    blocks and it throws if the link is still down
          hand.reconnect();

          // b) A successful reconnect() leaves the lifecycle at Connected: the drives are
          //    not enabled and homing_state is back to NotRun, because a new link may mean
          //    a rebooted drive and the old zero cannot be trusted
          const Diagnostics after = hand.get_diagnostics();
          std::printf("      reconnect ok, lifecycle=%s homing_state=%s\n",
                      to_string(after.lifecycle), to_string(after.homing_state));

          // c) Enable the drives again
          hand.run();

          // d) Home again, because reconnect() put homing_state back to NotRun and a joint
          //    target is refused until it reads Succeeded; the fingers move here
          hand.home();

          // e) Send the command again: reconnect() keeps max_effort but puts the controller
          //    input back to Idle, so without this the hand would sit at its home hold
          hand.set_command(cmd);

          std::printf("      recovered on attempt %d of %d\n", attempts, kMaxRecoveryAttempts);
        } catch (const Exception& error) {
          // A failed recovery stays Faulted, so the next tick tries reconnect() again
          std::printf("      attempt %d failed, %s: %s\n", attempts, to_string(error.code()),
                      error.what());
          std::printf("      check %s\n", what_to_check(error.code()));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }

      // Once a second, so the healthy path stays readable
      if (tick % 10 == 0) {
        std::printf("[%3d] %s homing=%s cycles=%llu misses=%llu period=%.3fms\n", tick,
                    to_string(diag.lifecycle), to_string(diag.homing_state),
                    static_cast<unsigned long long>(diag.control_cycles),
                    static_cast<unsigned long long>(diag.deadline_misses), diag.last_period_ms);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::printf("loop ended with %d recovery attempt(s) used\n", attempts);

    // 6) Idle drops the torque while the drives are still enabled, then end the session
    hand.set_command(Idle{});
    hand.stop();
    hand.disconnect();

    manager.destroy(hand);

    return 0;
  } catch (const Exception& error) {
    // Whatever failed, it arrives here as one type, and code() picks the group to check;
    // a failure this far out is either before the loop or a recovery that ran out of budget
    std::printf("%s: %s\n", to_string(error.code()), error.what());
    std::printf("check %s\n", what_to_check(error.code()));
    flush_log();
    return 1;
  }
}
