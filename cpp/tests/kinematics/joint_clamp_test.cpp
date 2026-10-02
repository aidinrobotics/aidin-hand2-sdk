// 목적(Tier 1, HW 무관): workspace clamp 경계가 **점에서 점으로 제대로 이어졌는지**와 투영이
// 정말 최단인지를 공개 API(JointPositionCommand::clamp) 만으로 검증한다.
//
// 경계 표(joint_clamp.cpp 의 kLongBoundary·kThumbBoundary)를 여기 다시 적지 않는다. 대신
// **clamp 자신에게 물어서** 경계를 알아낸다 — 어떤 굽힘에서 "손대지 않고 통과하는 가장 큰 벌림"
// 이 곧 그 굽힘에서의 경계다(probe_boundary). 그래서 표를 lin↔arc 로 바꾸거나 점을 옮겨도
// 이 테스트는 그대로 쓸 수 있다.
//
//   1) 이음새      굽힘을 0.01° 로 훑어 경계가 튀는 곳이 없는지 (구간 사이가 벌어지면 여기서 잡힌다)
//   2) 통과        경계 위/안의 목표는 손대지 않는지
//   3) 안쪽 보장   밖의 목표를 clamp 한 결과가 반드시 경계 안인지
//   4) 최단        clamp 결과가 경계를 촘촘히 샘플링한 기준의 최단점과 같은지
//   5) 대칭·고정점 벌림 부호 대칭, 굽힘 음수·상한 초과 처리, 몇 가지 손으로 아는 값
//   6) long j3 하한 clamp 한 (j1, j2) 에서 d3 가 home stop 에 닿는 각도까지 올리는지 (IK 로 판정)

#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

#include <aidin_hand2/hand/hand_kinematics.hpp>
#include <aidin_hand2/types/command.hpp>

using namespace aidin_hand2;

namespace
{

int g_failures = 0;

void check(bool ok, const char* what)
{
  if (!ok) { std::printf("  FAIL: %s\n", what); ++g_failures; }
}

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

// 검사 대상 finger — active_joint 슬롯과 굽힘 절대상한(joint_clamp.cpp 와 같은 값).
struct FingerUnderTest
{
  const char* name;
  int abduction_slot, flexion_slot;
  double flexion_max_deg;
};
constexpr std::array<FingerUnderTest, 2> kFingers{{
    {"thumb", 2, 1, 59.00},
    {"long",  4, 5, 96.20},   // index (baby/middle/ring 과 같은 경계)
}};

// 목표 (굽힘, 벌림) 을 clamp 한 결과 [deg].
void clamp_target(const FingerUnderTest& finger, double flexion_deg, double abduction_deg,
                  double& out_flexion_deg, double& out_abduction_deg)
{
  JointPositionCommand command;
  command.target[finger.flexion_slot]   = flexion_deg * kDeg2Rad;
  command.target[finger.abduction_slot] = abduction_deg * kDeg2Rad;
  command.clamp();
  out_flexion_deg   = command.target[finger.flexion_slot] / kDeg2Rad;
  out_abduction_deg = command.target[finger.abduction_slot] / kDeg2Rad;
}

bool passes_through(const FingerUnderTest& finger, double flexion_deg, double abduction_deg)
{
  double clamped_flexion_deg = 0.0, clamped_abduction_deg = 0.0;
  clamp_target(finger, flexion_deg, abduction_deg, clamped_flexion_deg, clamped_abduction_deg);
  return std::fabs(clamped_flexion_deg - flexion_deg) < 1e-9 &&
         std::fabs(clamped_abduction_deg - abduction_deg) < 1e-9;
}

// 이 굽힘에서 clamp 가 통과시키는 최대 벌림 = 경계. 벌림 0 조차 못 통과하면 −1(경계 없음).
double probe_boundary(const FingerUnderTest& finger, double flexion_deg)
{
  if (!passes_through(finger, flexion_deg, 0.0)) return -1.0;
  double inside_abduction_deg = 0.0, outside_abduction_deg = 60.0;
  for (int iteration = 0; iteration < 60; ++iteration) {
    const double middle_abduction_deg = 0.5 * (inside_abduction_deg + outside_abduction_deg);
    if (passes_through(finger, flexion_deg, middle_abduction_deg))
      inside_abduction_deg = middle_abduction_deg;
    else
      outside_abduction_deg = middle_abduction_deg;
  }
  return inside_abduction_deg;
}

// ── 1) 이음새: 경계가 굽힘에 따라 튀는 곳이 없는지 ──────────────────────────────
// 구간 사이가 어긋나 있으면(수직 점프) 인접 probe 값이 계단처럼 벌어진다. 마지막 점 뒤는
// 벌림 0 바닥이라, 경계가 0 으로 떨어지는 지점 하나는 정상적인 낙하로 인정한다.
void test_joins_are_continuous(const FingerUnderTest& finger)
{
  constexpr double kStepDeg = 0.01;
  // 0.01° 진행에 이보다 크게 변하면 끊긴 것으로 의심한다. 경계 끝의 벌림 닫힘 구간은 정상적으로도
  // 가파르다 — 2026-10-01 오른손 thumb 의 마지막 line 은 |기울기| ≈ 66, 즉 0.01° 당 0.66° 다.
  // 그래서 의심 구간은 100 배 촘촘히(0.0001°) 다시 훑는다. 기울기에서 온 변화는 간격에 비례해
  // 줄지만 수직 점프인 이음새 어긋남은 줄지 않으므로, 다시 훑어도 남는 변화만 끊김으로 본다.
  constexpr double kAllowedJumpDeg = 0.50;
  constexpr int kRefine = 100;
  double previous_boundary_deg = probe_boundary(finger, 0.0);
  double worst_jump_deg = 0.0, worst_jump_flexion_deg = 0.0;
  int unexpected_jumps = 0;
  for (double flexion_deg = kStepDeg; flexion_deg <= finger.flexion_max_deg; flexion_deg += kStepDeg) {
    const double boundary_deg = probe_boundary(finger, flexion_deg);
    if (boundary_deg < 0.0 || previous_boundary_deg < 0.0) { previous_boundary_deg = boundary_deg; continue; }
    double jump_deg = std::fabs(boundary_deg - previous_boundary_deg);
    if (jump_deg > kAllowedJumpDeg) {
      double fine_previous_deg = previous_boundary_deg;
      jump_deg = 0.0;
      for (int step = 1; step <= kRefine; ++step) {
        const double fine_deg = probe_boundary(finger, flexion_deg - kStepDeg + step * kStepDeg / kRefine);
        if (fine_deg >= 0.0 && fine_previous_deg >= 0.0 && fine_deg > 1e-6)
          jump_deg = std::fmax(jump_deg, std::fabs(fine_deg - fine_previous_deg));
        fine_previous_deg = fine_deg;
      }
    }
    if (jump_deg > kAllowedJumpDeg) {
      // 벌림 0 으로 떨어지는 낙하(바닥 진입)만 정상.
      if (boundary_deg > 1e-6) {
        ++unexpected_jumps;
        if (jump_deg > worst_jump_deg) { worst_jump_deg = jump_deg; worst_jump_flexion_deg = flexion_deg; }
      }
    }
    previous_boundary_deg = boundary_deg;
  }
  if (unexpected_jumps != 0)
    std::printf("  %s: 경계가 끊긴 곳 %d 군데, 최악 %.3f° (굽힘 %.2f°)\n",
                finger.name, unexpected_jumps, worst_jump_deg, worst_jump_flexion_deg);
  check(unexpected_jumps == 0, "구간 이음새에서 경계가 끊겼다");
}

// ── 2·3) 통과와 안쪽 보장 ──────────────────────────────────────────────────────
void test_inside_passes_and_outside_lands_inside(const FingerUnderTest& finger)
{
  int moved_inside_target = 0, landed_outside = 0;
  for (double flexion_deg = -5.0; flexion_deg <= finger.flexion_max_deg + 8.0; flexion_deg += 0.5) {
    const double boundary_deg = probe_boundary(finger, flexion_deg);
    for (double abduction_deg = -45.0; abduction_deg <= 45.0; abduction_deg += 0.5) {
      double clamped_flexion_deg = 0.0, clamped_abduction_deg = 0.0;
      clamp_target(finger, flexion_deg, abduction_deg, clamped_flexion_deg, clamped_abduction_deg);
      // 안에 있던 목표는 그대로여야 한다.
      if (boundary_deg >= 0.0 && flexion_deg >= 0.0 && std::fabs(abduction_deg) <= boundary_deg - 1e-6) {
        if (std::fabs(clamped_flexion_deg - flexion_deg) > 1e-9 ||
            std::fabs(clamped_abduction_deg - abduction_deg) > 1e-9) ++moved_inside_target;
      }
      // 결과는 언제나 경계 안이어야 한다.
      const double clamped_boundary_deg = probe_boundary(finger, clamped_flexion_deg);
      if (clamped_boundary_deg < 0.0 ||
          std::fabs(clamped_abduction_deg) > clamped_boundary_deg + 1e-6) ++landed_outside;
    }
  }
  if (moved_inside_target) std::printf("  %s: 안쪽 목표를 건드린 횟수 %d\n", finger.name, moved_inside_target);
  if (landed_outside) std::printf("  %s: clamp 결과가 경계 밖인 횟수 %d\n", finger.name, landed_outside);
  check(moved_inside_target == 0, "workspace 안의 목표를 clamp 가 건드렸다");
  check(landed_outside == 0, "clamp 결과가 경계 밖이다");
}

// ── 4) 최단: 경계를 촘촘히 샘플링한 기준과 대조 ────────────────────────────────
// probe 로 경계 점열을 만들고(바닥변 포함), 목표에서 가장 가까운 샘플점까지의 거리와
// clamp 결과까지의 거리를 비교한다. clamp 가 더 멀면 최단이 아니다.
void test_projection_is_shortest(const FingerUnderTest& finger)
{
  std::vector<std::array<double, 2>> boundary_samples;
  for (double flexion_deg = 0.0; flexion_deg <= finger.flexion_max_deg; flexion_deg += 0.02) {
    const double boundary_deg = probe_boundary(finger, flexion_deg);
    if (boundary_deg >= 0.0) boundary_samples.push_back({flexion_deg, boundary_deg});
  }
  check(boundary_samples.size() > 100, "경계 샘플을 못 만들었다");

  double worst_excess_deg = 0.0;
  int checked = 0;
  for (double flexion_deg = -4.0; flexion_deg <= finger.flexion_max_deg + 6.0; flexion_deg += 1.3)
    for (double abduction_deg = 0.5; abduction_deg <= 44.0; abduction_deg += 1.7) {
      double clamped_flexion_deg = 0.0, clamped_abduction_deg = 0.0;
      clamp_target(finger, flexion_deg, abduction_deg, clamped_flexion_deg, clamped_abduction_deg);
      const double query_flexion_deg = std::fmax(0.0, flexion_deg);
      if (std::fabs(clamped_flexion_deg - query_flexion_deg) < 1e-9 &&
          std::fabs(clamped_abduction_deg - abduction_deg) < 1e-9) continue;   // 안이라 안 움직였다
      const double clamped_distance_deg =
          std::hypot(query_flexion_deg - clamped_flexion_deg, abduction_deg - clamped_abduction_deg);
      double sampled_distance_deg = 1e300;
      for (const std::array<double, 2>& sample : boundary_samples)
        sampled_distance_deg = std::fmin(sampled_distance_deg,
            std::hypot(query_flexion_deg - sample[0], abduction_deg - sample[1]));
      worst_excess_deg = std::fmax(worst_excess_deg, clamped_distance_deg - sampled_distance_deg);
      ++checked;
    }
  std::printf("  %s: 최단성 검사 %d개, clamp 가 기준보다 더 멀리 간 최대량 %+.4f°\n",
              finger.name, checked, worst_excess_deg);
  // 기준은 0.02° 간격 샘플이라 그 절반(+여유)까지는 샘플 해상도 오차다.
  check(worst_excess_deg < 0.02, "clamp 투영이 최단이 아니다");
}

// ── 5) 대칭·범위·손으로 아는 값 ────────────────────────────────────────────────
void test_symmetry_and_known_points()
{
  for (const FingerUnderTest& finger : kFingers) {
    // 벌림 ±대칭: 부호만 반대인 목표는 결과도 부호만 반대여야 한다.
    int asymmetric = 0;
    for (double flexion_deg = 0.0; flexion_deg <= finger.flexion_max_deg; flexion_deg += 0.7)
      for (double abduction_deg = 1.0; abduction_deg <= 44.0; abduction_deg += 1.1) {
        double positive_flexion_deg = 0.0, positive_abduction_deg = 0.0;
        double negative_flexion_deg = 0.0, negative_abduction_deg = 0.0;
        clamp_target(finger, flexion_deg,  abduction_deg, positive_flexion_deg, positive_abduction_deg);
        clamp_target(finger, flexion_deg, -abduction_deg, negative_flexion_deg, negative_abduction_deg);
        if (std::fabs(positive_flexion_deg - negative_flexion_deg) > 1e-12 ||
            std::fabs(positive_abduction_deg + negative_abduction_deg) > 1e-12) ++asymmetric;
      }
    check(asymmetric == 0, "벌림 ± 대칭이 깨졌다");

    // 굽힘 음수는 0 으로, 굽힘 상한 초과는 상한 이하로.
    double clamped_flexion_deg = 0.0, clamped_abduction_deg = 0.0;
    clamp_target(finger, -30.0, 0.0, clamped_flexion_deg, clamped_abduction_deg);
    check(std::fabs(clamped_flexion_deg) < 1e-9, "굽힘 음수가 0 으로 안 잘렸다");
    clamp_target(finger, finger.flexion_max_deg + 20.0, 0.0, clamped_flexion_deg, clamped_abduction_deg);
    check(clamped_flexion_deg <= finger.flexion_max_deg + 1e-9, "굽힘이 상한을 넘었다");
    check(std::fabs(clamped_flexion_deg - finger.flexion_max_deg) < 1e-9,
          "굽힘 상한 초과가 상한으로 안 붙었다");
  }

  // plateau(경계가 수평인 구간)에서는 손으로 답을 안다 — 굽힘은 그대로, 벌림만 경계까지
  // 내려와야 한다. 수평이 아니면 최근접점이 비스듬해지므로 이 단정이 성립하지 않는다.
  //
  // 굽힘값을 박아 두지 않고 **clamp 자신에게 물어서** 수평 지점을 찾는다(이 파일의 방침).
  // 경계 모양은 회차마다 바뀐다 — 2026-07-30_left 튜닝에서 thumb 은 Line 구간이 아예 없어져
  // 예전에 박아 뒀던 30° 가 더는 plateau 가 아니다(9°~50° 가 29.6°→45.1° 로 상승하는 arc).
  for (const FingerUnderTest& finger : kFingers) {
    // 양옆 ±0.5° 에서 경계가 같은 곳 = 국소 수평. 없으면 이 finger 는 건너뛴다.
    double plateau_flexion_deg = -1.0, plateau_boundary_deg = -1.0;
    for (double flexion_deg = 1.0; flexion_deg <= finger.flexion_max_deg - 1.0; flexion_deg += 0.5) {
      const double here  = probe_boundary(finger, flexion_deg);
      const double left  = probe_boundary(finger, flexion_deg - 0.5);
      const double right = probe_boundary(finger, flexion_deg + 0.5);
      if (here <= 0.0 || left <= 0.0 || right <= 0.0) continue;
      if (std::fabs(here - left) < 1e-6 && std::fabs(here - right) < 1e-6) {
        plateau_flexion_deg  = flexion_deg;
        plateau_boundary_deg = here;
        break;
      }
    }
    if (plateau_flexion_deg < 0.0) {
      std::printf("  %s: 수평 구간이 없어 plateau 단정 검사 건너뜀\n", finger.name);
      continue;
    }
    double clamped_flexion_deg = 0.0, clamped_abduction_deg = 0.0;
    clamp_target(finger, plateau_flexion_deg, 44.0, clamped_flexion_deg, clamped_abduction_deg);
    check(std::fabs(clamped_flexion_deg - plateau_flexion_deg) < 1e-6,
          "plateau 밖 목표인데 굽힘이 움직였다");
    check(std::fabs(clamped_abduction_deg - plateau_boundary_deg) < 1e-6,
          "plateau 에서 벌림이 경계로 안 내려왔다");
  }
}

// 결합 없는 관절(상수 상한)도 같이 본다 — 표와 무관하지만 같은 clamp 호출이 담당한다.
void test_independent_joint_limits()
{
  JointPositionCommand command;
  for (double& value : command.target) value = 200.0 * kDeg2Rad;
  command.clamp();
  check(std::fabs(command.target[0] - 108.8 * kDeg2Rad) < 1e-12, "thumb j0 상한(108.8°)으로 안 붙었다");
  check(std::fabs(command.target[3] -  75.0 * kDeg2Rad) < 1e-12, "thumb j3 상한(75°)으로 안 붙었다");
  for (int base : {4, 7, 10, 13})
    check(std::fabs(command.target[base + 2] - 80.0 * kDeg2Rad) < 1e-12, "long j3 상한(80°)으로 안 붙었다");

  for (double& value : command.target) value = -50.0 * kDeg2Rad;
  command.clamp();
  check(command.target[0] >= -1e-12 && command.target[3] >= -1e-12, "독립 관절 하한(0) 미달");

  // JointImpedance 도 같은 경계를 써야 한다.
  JointImpedanceCommand position_like;
  JointPositionCommand reference;
  for (std::size_t slot = 0; slot < kActiveJointCount; ++slot) {
    position_like.target[slot] = 40.0 * kDeg2Rad;
    reference.target[slot]     = 40.0 * kDeg2Rad;
  }
  position_like.clamp();
  reference.clamp();
  for (std::size_t slot = 0; slot < kActiveJointCount; ++slot)
    check(std::fabs(position_like.target[slot] - reference.target[slot]) < 1e-15,
          "JointImpedance 와 JointPosition 의 clamp 결과가 다르다");
}

// ── 6) long j3 하한 ────────────────────────────────────────────────────────────
// d3 가 home stop 보다 덜 나가야 하는 j3 는 IK 가 count 0 으로 잘라 손이 가지 못한다. 그래서 clamp 는
// j3 를 clamp 한 (j1, j2) 에서 d3 가 home stop 에 닿는 각도까지 올린다. 하한 값을 여기 적지 않고 IK 로만
// 판정한다 — clamp 한 j3 에서 d3 는 home stop 위(count 0 근처)이고, 0.05° 더 굽히면 움직이기 시작해야
// 한다. 하한이 0 아래인 자세에서는 j3 0 이 그대로이고, 0.05° 에서 d3 가 이미 움직인다.
void test_long_distal_floor()
{
  // clamp 의 하한은 문서 14장 3.2절의 닫힌 식이다. IK 로 구한 하한과 경계 안에서 0.0004° 안으로 같고,
  // 하한 위치의 d3 는 count 0 이었다(2026-10-02, type a). 8 count(약 0.005°)는 상수가 틀어졌을 때 잡는 여유다.
  constexpr int kHomeStopCount = 8;
  constexpr std::array<int, 4> kLongBases{4, 7, 10, 13};   // index, middle, ring, baby 의 j1 슬롯
  int raised_past_floor = 0, left_below_floor = 0, fingers_differ = 0, checked = 0;
  for (double flexion_deg = -4.0; flexion_deg <= 104.0; flexion_deg += 1.5)
    for (double abduction_deg = -40.0; abduction_deg <= 40.0; abduction_deg += 2.5)
      for (double distal_deg : {-20.0, 0.0}) {
        JointPositionCommand command;
        for (int base : kLongBases) {
          command.target[base]     = abduction_deg * kDeg2Rad;
          command.target[base + 1] = flexion_deg * kDeg2Rad;
          command.target[base + 2] = distal_deg * kDeg2Rad;
        }
        command.clamp();
        const std::array<int, kActuatorCount> at_clamped = ik_joint_to_actuator(command.target);
        std::array<double, kActiveJointCount> bent_further = command.target;
        for (int base : kLongBases) bent_further[base + 2] += 0.05 * kDeg2Rad;
        const std::array<int, kActuatorCount> at_bent_further = ik_joint_to_actuator(bent_further);
        for (int base : kLongBases) {
          // 능동 슬롯과 actuator 번호가 long finger 에서는 같다(d3 = base + 2).
          if (command.target[base + 2] > 1e-12 && at_clamped[base + 2] > kHomeStopCount) ++raised_past_floor;
          if (at_bent_further[base + 2] <= 0) ++left_below_floor;
          if (std::fabs(command.target[base + 2] - command.target[kLongBases[0] + 2]) > 1e-12) ++fingers_differ;
          ++checked;
        }
      }
  std::printf("  long j3 하한 검사 %d개\n", checked);
  if (raised_past_floor) std::printf("  하한보다 높이 올린 횟수 %d\n", raised_past_floor);
  if (left_below_floor) std::printf("  하한 아래에 남긴 횟수 %d\n", left_below_floor);
  check(raised_past_floor == 0, "long j3 를 d3 home stop 보다 높이 올렸다");
  check(left_below_floor == 0, "long j3 가 d3 home stop 아래에 남았다");
  check(fingers_differ == 0, "long finger 넷의 j3 하한이 다르다");

  // 손으로 아는 값: flexion 40°, abduction 0° 의 하한은 5.687°, flexion 90° 에서는 하한이 0 아래다.
  // 범위 안의 j3 는 그대로. 상한은 자세와 무관하게 80° 다 — 손으로 굽혀 본 스톱이 모든 abduction 에서
  // 80° ± 0.1° 였다.
  auto clamp_distal = [](double flexion_deg, double abduction_deg, double distal_deg) {
    JointPositionCommand command;
    command.target[4] = abduction_deg * kDeg2Rad;
    command.target[5] = flexion_deg * kDeg2Rad;
    command.target[6] = distal_deg * kDeg2Rad;
    command.clamp();
    return command.target[6] / kDeg2Rad;
  };
  check(std::fabs(clamp_distal(40.0, 0.0, 0.0) - 5.687) < 0.01, "flexion 40° 에서 j3 0 이 하한 5.687° 로 안 올라갔다");
  check(std::fabs(clamp_distal(90.0, 0.0, 0.0)) < 1e-12, "flexion 90° 에서 j3 0 이 움직였다");
  check(std::fabs(clamp_distal(40.0, 0.0, 30.0) - 30.0) < 1e-9, "범위 안의 j3 30° 를 clamp 가 건드렸다");
  int ceiling_moved = 0;
  for (double flexion_deg = 0.0; flexion_deg <= 96.0; flexion_deg += 8.0)
    for (double abduction_deg = -30.0; abduction_deg <= 30.0; abduction_deg += 10.0)
      if (std::fabs(clamp_distal(flexion_deg, abduction_deg, 120.0) - 80.0) > 1e-9) ++ceiling_moved;
  check(ceiling_moved == 0, "long j3 상한이 자세에 따라 80° 에서 벗어났다");

  // thumb j3 는 결합이 없어 굽힘과 무관하게 0 이 그대로다.
  JointPositionCommand thumb;
  thumb.target[1] = 30.0 * kDeg2Rad;
  thumb.clamp();
  check(std::fabs(thumb.target[3]) < 1e-12, "thumb j3 0 이 움직였다");
}

}  // namespace

int main()
{
  for (const FingerUnderTest& finger : kFingers) {
    std::printf("[%s]\n", finger.name);
    test_joins_are_continuous(finger);
    test_inside_passes_and_outside_lands_inside(finger);
    test_projection_is_shortest(finger);
  }
  test_symmetry_and_known_points();
  test_independent_joint_limits();
  test_long_distal_floor();

  std::printf("joint_clamp test: %s\n", g_failures == 0 ? "OK" : "FAIL");
  return g_failures == 0 ? 0 : 1;
}
