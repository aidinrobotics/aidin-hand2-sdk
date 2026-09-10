// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, tactile readings and the baseline you have to build yourself
//
// The taxel values are raw 16-bit counts as the sensors sent them. There is no unit, no
// normalization, and no contact threshold: the SDK does not offer one, because the resting
// value of a taxel depends on the sensor. So a reading on its own says very little, and the
// difference from a resting value says a great deal.
//
// This example takes that resting value first — a second of averaging with nothing touching
// the hand — and then displays every taxel as its deviation from it. Touch a fingertip and its
// column moves; let go and it returns. The raw counts are printed alongside, so the size of
// the deviation against the size of the number is visible.
//
// Reading tactile needs no torque, so this example never leaves Connected. Nothing moves: no
// run(), no home(), no set_command().
//
// Run as: 15_tactile [right|left] [interface=can0]
// Needs a real hand. Keep your hands off it for the first second, while the baseline is taken.

#include <array>
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
// SIGINT only asks the display loop to stop. The HandManager destructor is what closes the
// link, so the point is to leave the loop and reach that destructor.
std::atomic<bool> g_shutdown{false};
}  // namespace

int main(int argc, char** argv)
{
  std::signal(SIGINT, [](int) { g_shutdown.store(true); });

  const HandSide side = (argc > 1 && std::string(argv[1]) == "left") ? HandSide::Left : HandSide::Right;

  // Finger index follows the Finger enum: Thumb, Index, Middle, Ring, Baby
  const std::array<const char*, kFingerCount> finger_name = {"thumb", "index", "middle", "ring", "baby"};

  try {
    HandConfig config{argc > 2 ? argv[2] : "can0", side};
    config.control_rate = 500;    // Hz
    config.auto_home    = false;  // this example stays in Connected, so homing never comes up

    HandManager manager;
    Hand hand = manager.create(config);
    hand.connect();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 1) One snapshot, to show what the numbers look like before anything is subtracted.
    const HandState first = hand.get_state();
    std::printf("raw counts, %s fingertip taxels 0-5:", finger_name[1]);
    for (std::size_t t = 0; t < 6; ++t) {
      std::printf(" %7.0f", first.tactile.fingers[1][t]);
    }
    std::printf("\n");
    std::printf("raw counts, palm taxels 0-5:        ");
    for (std::size_t t = 0; t < 6; ++t) {
      std::printf(" %7.0f", first.tactile.palm[t]);
    }
    std::printf("\n\nnone of those say whether anything is touching the hand\n\n");

    // 2) The baseline. A second of averaging at 20 Hz, with nothing in contact, so a slow
    //    drift or an offset per taxel is folded in rather than showing up later as contact.
    std::printf("taking the baseline — keep your hands off the hand\n");
    std::array<std::array<double, kTactileTaxelsPerFinger>, kFingerCount> finger_baseline{};
    std::array<double, kPalmTactileCount> palm_baseline{};

    constexpr int kSamples = 20;
    for (int s = 0; s < kSamples; ++s) {
      const HandState state = hand.get_state();
      for (std::size_t f = 0; f < kFingerCount; ++f) {
        for (std::size_t t = 0; t < kTactileTaxelsPerFinger; ++t) {
          finger_baseline[f][t] += state.tactile.fingers[f][t] / kSamples;
        }
      }
      for (std::size_t t = 0; t < kPalmTactileCount; ++t) {
        palm_baseline[t] += state.tactile.palm[t] / kSamples;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::printf("baseline taken\n\n");

    // 3) The threshold is a choice, not a constant the SDK provides. This one is loose enough
    //    to ignore the noise on a resting taxel and tight enough to catch a light touch; the
    //    right value for an application comes from watching its own hand at rest.
    constexpr double kContactThreshold = 200.0;

    std::printf("touch a fingertip or the palm — Ctrl-C to finish\n");
    std::printf("each column is one finger: peak deviation, then which taxel, then contact\n\n");

    while (!g_shutdown.load()) {
      const HandState state = hand.get_state();

      // 4) Per finger, the taxel that moved furthest from its baseline. Which taxel it is
      //    matters as much as how far: the taxels are spread over the fingertip and the pad,
      //    so the index says roughly where the contact is.
      std::printf("\r\033[K");
      for (std::size_t f = 0; f < kFingerCount; ++f) {
        double      peak       = 0.0;
        std::size_t peak_taxel = 0;
        for (std::size_t t = 0; t < kTactileTaxelsPerFinger; ++t) {
          const double deviation = std::fabs(state.tactile.fingers[f][t] - finger_baseline[f][t]);
          if (deviation > peak) {
            peak       = deviation;
            peak_taxel = t;
          }
        }
        std::printf("%s %6.0f t%-2zu %s  ", finger_name[f], peak, peak_taxel,
                    peak > kContactThreshold ? "**" : "  ");
      }

      // 5) The palm is a flat array of 58, laid out as palm1 upper 20, palm1 lower 20, then
      //    palm2's 18. The boundaries are worth respecting when locating a contact, because
      //    the three regions are separate surfaces rather than a continuous grid.
      double      palm_peak  = 0.0;
      std::size_t palm_taxel = 0;
      for (std::size_t t = 0; t < kPalmTactileCount; ++t) {
        const double deviation = std::fabs(state.tactile.palm[t] - palm_baseline[t]);
        if (deviation > palm_peak) {
          palm_peak  = deviation;
          palm_taxel = t;
        }
      }
      const char* region = palm_taxel < kPalm1UpperCount ? "palm1 upper"
                           : palm_taxel < kPalm1UpperCount + kPalm1LowerCount ? "palm1 lower"
                                                                             : "palm2      ";
      std::printf("| %s %6.0f t%-2zu %s", region, palm_peak, palm_taxel,
                  palm_peak > kContactThreshold ? "**" : "  ");
      std::fflush(stdout);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::printf("\n\n");

    // 6) A baseline taken once goes stale. Temperature and a long grasp both move it, so an
    //    application that runs for a while re-takes it whenever the hand is known to be free.
    std::printf("done — re-take the baseline whenever nothing is in contact\n");
  } catch (const Exception& error) {
    std::printf("\nfailed: %s: %s\n", to_string(error.code()), error.what());
    return 1;
  }

  return 0;
}
