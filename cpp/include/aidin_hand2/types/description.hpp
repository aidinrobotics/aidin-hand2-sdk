// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstddef>

namespace aidin_hand2
{

// --------------------------------- Constants --------------------------------

// Fixed structural constants of the AIDIN Hand Gen2
inline constexpr std::size_t kActuatorCount          = 16;
inline constexpr std::size_t kJointCount             = 21;
inline constexpr std::size_t kActiveJointCount       = 16;

// The 4-bar coupled q4 of each digit
inline constexpr std::size_t kPassiveJointCount      = 5;

// Fingertip positions as xyz
inline constexpr std::size_t kTaskCount              = 15;

// Thumb 6-D pose with 4 fingertip positions
inline constexpr std::size_t kOrientedTaskCount      = 18;

inline constexpr std::size_t kFingerCount            = 5;
inline constexpr std::size_t kTactileTaxelsPerFinger = 17;
inline constexpr std::size_t kPalmTactileCount       = 58;
inline constexpr std::size_t kPalm1UpperCount        = 20;
inline constexpr std::size_t kPalm1LowerCount        = 20;
inline constexpr std::size_t kPalm2Count             = 18;

// ----------------------------------- Enums ----------------------------------

enum class HandSide {
  Left,
  Right,
};

enum class Finger {
  Thumb,
  Index,
  Middle,
  Ring,
  Baby,
};

}  // namespace aidin_hand2
