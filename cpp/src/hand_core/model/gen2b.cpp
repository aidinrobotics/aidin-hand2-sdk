#include "hand_core/kinematics/gen2b/kinematics.hpp"
#include "hand_core/model/model_composition.hpp"

namespace aidin_hand2
{

namespace
{

// Gen2's, with thumb j1 and j2 at a quarter
constexpr std::array<double, kActiveJointCount> kHardStiffness{
    27.4, 15.0, 2.9, 14.5, 5.33, 20.1, 0.0, 5.33, 20.1, 0.0, 5.33, 20.1, 0.0, 5.33, 20.1, 0.0};

constexpr ActuatorConstants kActuator{
    1024.0,   // 256 CPT x 4 on the motor shaft
    3.62e-3,  // Nm/A
    0.4,      // A
    3.3e-9,   // kg m^2
};

}  // namespace

const ModelComposition kGen2BComposition{kinematics::gen2b::kSolver, kHardStiffness, kActuator};

}  // namespace aidin_hand2
