// A kinematics call into the packaged library, and an Exception out of it caught by its type
#include <array>
#include <cstdio>

#include <aidin_hand2/aidin_hand2.hpp>

using namespace aidin_hand2;

int main()
{
  const auto joint = fk_actuator_to_joint(HandModel::Gen2, std::array<int, kActuatorCount>{});

  HandManager manager;
  Hand hand = manager.create(HandConfig{"can0", HandSide::Right, HandModel::Gen2});
  try {
    hand.run();  // never connected
  } catch (const Exception& error) {
    if (error.kind() != ErrorKind::WrongCallOrder) {
      std::printf("FAIL: run() before connect() threw %s, not WrongCallOrder\n", to_string(error.kind()));
      return 1;
    }
    std::printf("package_consumer: SDK %s, joint 0 at %.3f rad, %s caught\n", kVersionString, joint[0],
                to_string(error.kind()));
    return 0;
  }
  std::printf("FAIL: run() before connect() did not throw\n");
  return 1;
}
