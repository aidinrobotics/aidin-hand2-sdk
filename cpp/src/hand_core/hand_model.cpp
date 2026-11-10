#include "hand_core/hand_model.hpp"

namespace aidin_hand2
{

namespace
{

// Thumb j0..j3, then j1..j3 of each long finger; the long finger j3 is computed from the pose
constexpr std::array<double, kActiveJointCount> kHardStiffnessGen2{
    27.4, 60.0, 11.6, 14.5, 5.33, 20.1, 0.0, 5.33, 20.1, 0.0, 5.33, 20.1, 0.0, 5.33, 20.1, 0.0};

// Thumb j1 and j2 are a quarter of Gen2's for the 2 mm lead, not yet measured on a Gen2_B hand
constexpr std::array<double, kActiveJointCount> kHardStiffnessGen2B{
    27.4, 15.0, 2.9, 14.5, 5.33, 20.1, 0.0, 5.33, 20.1, 0.0, 5.33, 20.1, 0.0, 5.33, 20.1, 0.0};

const HandModel kGen2{HandType::Gen2, kinematics::boundary_of(HandType::Gen2), kHardStiffnessGen2};
const HandModel kGen2B{HandType::Gen2_B, kinematics::boundary_of(HandType::Gen2_B), kHardStiffnessGen2B};

}  // namespace

const HandModel& hand_model_of(HandType type)
{
  return type == HandType::Gen2_B ? kGen2B : kGen2;
}

}  // namespace aidin_hand2
