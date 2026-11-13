#include "hand_core/kinematics/gen2/kinematics.hpp"
#include "hand_core/model/model_composition.hpp"

namespace aidin_hand2
{

namespace
{

// Thumb j0..j3, then j1..j3 of each long finger; the long finger j3 is computed from the pose
constexpr std::array<double, kActiveJointCount> kHardStiffness{
    27.4, 60.0, 11.6, 14.5, 5.33, 20.1, 0.0, 5.33, 20.1, 0.0, 5.33, 20.1, 0.0, 5.33, 20.1, 0.0};

constexpr ActuatorConstants kActuator{
    1024.0,   // 256 CPT x 4 on the motor shaft
    3.62e-3,  // Nm/A
    0.4,      // A
    3.3e-9,   // kg m^2
};

}  // namespace

const ModelComposition kGen2Composition{kinematics::gen2::kSolver, kHardStiffness, kActuator};

}  // namespace aidin_hand2
