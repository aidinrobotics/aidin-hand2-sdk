#pragma once

namespace aidin_hand2
{

// One hand model's actuators, the same on all 16
struct ActuatorConstants {
  // Encoder count per motor turn
  double count_per_revolution;

  // The motor torque constant
  double torque_constant_Nm_per_A;

  // Current at effort 1000, the drive setting
  double rated_current_A;

  // On the motor shaft
  double motor_inertia_kg_m2;
};

}  // namespace aidin_hand2
