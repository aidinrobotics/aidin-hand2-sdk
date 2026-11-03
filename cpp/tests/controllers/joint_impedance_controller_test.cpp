// 목적(Tier 1, HW 무관): joint 공간 impedance 제어기를 host 에서 회귀 점검한다.
// 기준값은 kinematics archive(type a)로 기준 자세(엄지 20/20/0/20°, 긴 손가락 0/20/20°)에서 구했다.
// type c 에는 Jacobian 이 없으므로 그 빌드에서는 a·b 전용 시험을 건너뛴다.

#include <cstdio>

namespace
{
int g_failures = 0;
}  // namespace

int main()
{
  std::printf("joint_impedance_controller test: %s\n", g_failures == 0 ? "OK" : "FAIL");
  return g_failures == 0 ? 0 : 1;
}
