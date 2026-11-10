// 목적(Tier 1, HW 무관): joint 공간 impedance 제어기를 host 에서 회귀 점검한다.
//   1) joint torque → actuator effort: Jacobian 전치와 효율, 2-rod 쌍, 긴 손가락 j3 는 d3 에만.
//   2) joint impedance torque: stiffness 항, 현재 자세 관성에서 구한 damping 항.
//   3) compute(): FK 각도·rpm 에서 frame 까지, clamp, NaN 자세, home 자세, 목표 필터·속도 제한,
//      stiffness 램프, hard stop 여유.
//   4) 마찰 보상: Coulomb + 점성을 impedance effort 를 직접 넣어 보고, compute() 가 더하는지 본다.
// 기준값은 kinematics archive(type a)로 기준 자세(엄지 20/20/0/20°, 긴 손가락 0/20/20°)에서 구했다.
// type b 는 엄지 d1·d2 리드만 다르다. type c 에는 Jacobian 이 없으므로 a·b 전용 시험을 건너뛴다.

#include <array>
#include <cmath>
#include <cstdio>

#include <aidin_hand2/types/command.hpp>
#include <aidin_hand2/types/config.hpp>
#include <aidin_hand2/types/description.hpp>
#include <aidin_hand2/types/state.hpp>

#include "hand_core/comms/canfd/protocol.hpp"
#include "hand_core/controllers/joint_impedance_controller.hpp"
#include "hand_core/kinematics/hand_kinematics_core.hpp"

using namespace aidin_hand2;

namespace
{
int g_failures = 0;

void check(bool ok, const char* what)
{
  if (!ok) { std::printf("  FAIL: %s\n", what); ++g_failures; }
}

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr double kCountPerRad = 1024.0 / (2.0 * 3.14159265358979323846);
constexpr double kRatedMotorTorqueNm = 3.62e-3 * 0.4;
constexpr long kPeriodNs = 2'000'000;  // 500 Hz

// type b 의 엄지 d1·d2 는 리드 2 mm(type a 는 1 mm)라 같은 torque 에 effort 가 두 배다.
#if defined(AIDIN_HAND2_HAND_TYPE_B)
constexpr double kThumbPairLeadScale = 2.0;
#else
constexpr double kThumbPairLeadScale = 1.0;
#endif

using Config = ControllerConfig::JointImpedanceController;
using Active = std::array<double, kActiveJointCount>;

#if !defined(AIDIN_HAND2_HAND_TYPE_C)
// 엄지 q0..q3, 긴 손가락마다 q1..q3 (deg)
Active pose_deg(std::array<double, 4> thumb, std::array<double, 3> finger)
{
  Active q{};
  for (std::size_t k = 0; k < 4; ++k) q[k] = thumb[k] * kDeg;
  for (std::size_t f = 0; f < 4; ++f) {
    for (std::size_t k = 0; k < 3; ++k) q[4 + 3 * f + k] = finger[k] * kDeg;
  }
  return q;
}

// 기준 자세: GUI 가 n 상수를 읽던 자세.
Active reference_pose() { return pose_deg({20, 20, 0, 20}, {0, 20, 20}); }

struct Jacobians {
  controllers::JointJacobian joint_per_count{};
  controllers::JointJacobian count_per_joint{};
};

Jacobians jacobians_at(const Active& q)
{
  Jacobians j{};
  kinematics::jacobian_actuator_to_joint(q, j.joint_per_count);
  kinematics::jacobian_joint_to_actuator(q, j.count_per_joint);
  return j;
}

// joint 하나에만 torque 를 준다.
Active torque_on(std::size_t joint, double torque)
{
  Active tau{};
  tau[joint] = torque;
  return tau;
}

// expected 에 없는 actuator 는 0 이어야 한다.
bool efforts_are(const std::array<double, kActuatorCount>& effort,
                 std::initializer_list<std::pair<std::size_t, double>> expected)
{
  std::array<bool, kActuatorCount> listed{};
  for (const auto& [i, value] : expected) {
    listed[i] = true;
    if (std::abs(effort[i] - value) > 0.05) return false;
  }
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    if (!listed[i] && std::abs(effort[i]) > 0.01) return false;
  }
  return true;
}

// ── Test 1: joint torque → actuator effort ──
void test_joint_torque_to_effort()
{
  const Jacobians j = jacobians_at(reference_pose());
  const Config config{};
  auto effort = [&](std::size_t joint, double k) {
    return controllers::joint_torque_to_effort(torque_on(joint, k * kDeg), j.joint_per_count, config);
  };
  check(efforts_are(effort(0, 27), {{0, 319.57}}), "thumb j0 1 deg at K 27");
  const double s = kThumbPairLeadScale;
  check(efforts_are(effort(1, 90), {{1, 323.94 * s}, {2, 323.94 * s}}), "thumb j1 is the common mode of d1 d2");
  check(efforts_are(effort(2, 24), {{1, -248.06 * s}, {2, 248.06 * s}}), "thumb j2 is the differential mode of d1 d2");
  check(efforts_are(effort(3, 5), {{3, 82.76}}), "thumb j3 1 deg at K 5");
  check(efforts_are(effort(4, 3.75), {{4, 77.52}, {5, -77.52}}), "index j1 is the differential mode");
  check(efforts_are(effort(5, 22.5), {{4, 186.75}, {5, 186.75}}), "index j2 is the common mode");
  // d3 링크의 반작용(d1·d2 로 5.64 씩 상쇄하던 몫)은 보내지 않는다. j2 가 자기 강성으로 버틴다.
  check(efforts_are(effort(6, 6.75), {{6, 151.90}}), "index j3 drives d3 only");

  Config half = config;
  half.efficiency[0] = 0.5;
  check(efforts_are(controllers::joint_torque_to_effort(torque_on(0, 27 * kDeg), j.joint_per_count, half), {{0, 639.14}}),
        "half the efficiency doubles the effort");

  // 엄지 q2 20°: 두 rod 의 몫이 갈라져도 j1 torque 는 j1 에만 걸린다.
  const Jacobians tilted = jacobians_at(pose_deg({20, 20, 20, 20}, {0, 20, 20}));
  const auto split = controllers::joint_torque_to_effort(torque_on(1, 90 * kDeg), tilted.joint_per_count, config);
  check(efforts_are(split, {{1, 308.99 * s}, {2, 373.60 * s}}), "thumb j1 at j2 20 deg splits 308.99 / 373.60 (x2 on type b)");
  Active back{};
  for (std::size_t q = 0; q < kActiveJointCount; ++q) {
    for (std::size_t i = 0; i < kActuatorCount; ++i) {
      back[q] += tilted.count_per_joint[i * kActiveJointCount + q] / kCountPerRad * split[i] / 1000.0 * kRatedMotorTorqueNm;
    }
  }
  check(std::abs(back[1] - 90 * kDeg) < 1e-4, "the efforts give back the j1 torque");
  check(std::abs(back[2]) < 1e-6, "no torque leaks to j2");
}

// ── Test 2: joint impedance torque ──
void test_joint_impedance_torque()
{
  const Active q = reference_pose();
  const Jacobians j = jacobians_at(q);
  Config config{};
  config.stiffness.fill(0.0);
  config.stiffness[0] = 1.0;

  // 엄지 j0 stiffness 1 은 hard 27.4 N·m/rad.
  Active target = q;
  target[0] += 1.0 * kDeg;
  const Active still{};
  const Active spring = controllers::joint_impedance_torque(target, q, still, config.stiffness, config, j.count_per_joint);
  bool others_zero = true;
  for (std::size_t k = 1; k < kActiveJointCount; ++k) others_zero = others_zero && spring[k] == 0.0;
  check(std::abs(spring[0] - 0.478220) < 1e-6 && others_zero, "stiffness 1 is the hard stiffness times the joint error");

  Config middle = config;
  middle.stiffness[0] = 0.5;
  check(std::abs(controllers::joint_impedance_torque(target, q, still, middle.stiffness, middle, j.count_per_joint)[0] -
                 0.239110) < 1e-6,
        "stiffness scales the hard stiffness linearly");

  // q̇0 0.61698 rad/s 는 d0 6000 rpm. 관성 1018.4² × 3.3e-9 = 3.4224e-3 kg·m².
  // 엄지 j0 의 감쇠비는 0.62 로 고정이라 D = 0.37972.
  Active moving{};
  moving[0] = 0.61698;
  const Active damped = controllers::joint_impedance_torque(q, q, moving, config.stiffness, config, j.count_per_joint);
  check(std::abs(damped[0] - (-0.234279)) < 1e-4, "thumb j0 damping ratio is 0.62");

  Config soft = config;
  soft.stiffness[0] = 0.0;
  check(controllers::joint_impedance_torque(q, q, moving, soft.stiffness, soft, j.count_per_joint)[0] == 0.0,
        "no stiffness, no damping");

  Config heavy = config;
  heavy.link_inertia_kgm2[0] = 3.4224e-3;
  check(std::abs(controllers::joint_impedance_torque(q, q, moving, heavy.stiffness, heavy, j.count_per_joint)[0] -
                 (-0.331321)) < 1e-4,
        "link inertia adds to the motor inertia");

  // 긴 손가락 j3 의 hard 는 모터 쪽 강성 3.62e-5 N·m/rad 를 지킨다. middle j3(9) 는 (0, 20, 20) 에서
  // J⁻¹ 536 이라 10.40 N·m/rad, (0, 80, 0) 에서는 J⁻¹ 744 가 상한 652 에서 멈춰 15.39 N·m/rad.
  Config j3{};
  j3.stiffness.fill(0.0);
  j3.stiffness[9] = 1.0;
  Active bent = q;
  bent[9] += 1.0 * kDeg;
  check(std::abs(controllers::joint_impedance_torque(bent, q, still, j3.stiffness, j3, j.count_per_joint)[9] - 0.18151) <
            1e-3,
        "long finger j3 stiffness 1 holds the motor-side stiffness at (0, 20, 20)");
  const Active flexed = pose_deg({20, 20, 0, 20}, {0, 80, 0});
  Active flexed_target = flexed;
  flexed_target[9] += 1.0 * kDeg;
  const Jacobians at_flexed = jacobians_at(flexed);
  check(std::abs(controllers::joint_impedance_torque(flexed_target, flexed, still, j3.stiffness, j3,
                                                     at_flexed.count_per_joint)[9] -
                 0.26859) < 1e-4,
        "long finger j3 hard stiffness stops at the cap");
}

// count 를 actuator 에, 그 count 의 FK 를 joint 에 채운 상태. 실제 루프와 같은 경로다.
HandState state_from_counts(const std::array<int, kActuatorCount>& count)
{
  HandState state{};
  for (std::size_t i = 0; i < kActuatorCount; ++i) state.actuators.position_count[i] = count[i];
  std::array<double, kinematics::JOINT_NUM> joint{};
  std::array<double, kinematics::TASK_NUM> task{};
  kinematics::fk_actuator_to_joint(count, joint, task);
  for (std::size_t k = 0; k < kJointCount; ++k) state.joints.position_rad[k] = joint[k];
  return state;
}

// 자세 q 의 IK count 로 만든 상태.
HandState state_at(const Active& q)
{
  std::array<int, kActuatorCount> count{};
  kinematics::ik_joint_to_actuator(q, count);
  return state_from_counts(count);
}

// 21 개 FK 각도에서 passive q4 를 뺀 16 개.
Active active_of(const HandState& state)
{
  Active active{};
  for (std::size_t k = 0; k < 4; ++k) active[k] = state.joints.position_rad[k];
  for (std::size_t f = 0; f < 4; ++f) {
    for (std::size_t k = 0; k < 3; ++k) active[4 + 3 * f + k] = state.joints.position_rad[5 + 4 * f + k];
  }
  return active;
}

// 엄지 j0 에만 stiffness 1(27.4 N·m/rad), 나머지 0, 마찰 없음. 목표 필터를 꺼서 한 주기에 법칙만 본다.
Config thumb_j0_gains()
{
  Config config{};
  config.stiffness.fill(0.0);
  config.stiffness[0] = 1.0;
  config.filter_enabled = false;
  return config;
}

canfd::CommandFrames run_once(const Active& target, const Config& config, const HandState& state, double max_effort)
{
  controllers::JointImpedanceController controller(kPeriodNs);
  JointImpedanceCommand command{};
  command.target = target;
  std::array<double, kActuatorCount> limit{};
  limit.fill(max_effort);
  canfd::CommandFrames frames{};
  controller.compute(command, config, state, limit, frames);
  return frames;
}

bool all_cst(const canfd::CommandFrames& frames)
{
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    if (frames.mode_of_operation[i] != canfd::mode_of_operation::kCST) return false;
  }
  return true;
}

// ── Test 3: compute() ──
void test_compute()
{
  const HandState state = state_at(reference_pose());
  const Active pose = active_of(state);

  // a. 엄지 j0 1° 오차 → 324.30, int16 절단
  Active target = pose;
  target[0] += 1.0 * kDeg;
  const auto stiff = run_once(target, thumb_j0_gains(), state, 30000.0);
  bool others_zero = true;
  for (std::size_t i = 1; i < kActuatorCount; ++i) others_zero = others_zero && stiff.target_effort[i] == 0;
  check(stiff.target_effort[0] >= 323 && stiff.target_effort[0] <= 325 && others_zero && all_cst(stiff),
        "compute: thumb j0 1 deg gives 323..325 on d0 only, CST");

  // b. 오차 0, d0 6000 rpm, ζ 0.62 → −158.87
  HandState moving = state;
  moving.actuators.velocity_rpm[0] = 6000;
  const auto damped = run_once(pose, thumb_j0_gains(), moving, 30000.0);
  check(damped.target_effort[0] >= -160 && damped.target_effort[0] <= -158, "compute: damping from the drive rpm");

  // c. 10° 오차는 max_effort 에서 잘린다
  Active far = pose;
  far[0] += 10.0 * kDeg;
  check(run_once(far, thumb_j0_gains(), state, 1000.0).target_effort[0] == 1000, "compute: clamped to max_effort");

  // d. joint 가 NaN 이면 effort 0
  HandState unknown = state;
  unknown.joints.position_rad.fill(std::nan(""));
  const auto blind = run_once(target, thumb_j0_gains(), unknown, 1000.0);
  bool zero = all_cst(blind);
  for (std::size_t i = 0; i < kActuatorCount; ++i) zero = zero && blind.target_effort[i] == 0;
  check(zero, "compute: a NaN pose sends zero effort");

  // e. home 자세(count 0, joint 0), 엄지 j0 1° → 유한하고 max_effort 안
  HandState home{};
  Active home_target{};
  home_target[0] = 1.0 * kDeg;
  const auto edge = run_once(home_target, thumb_j0_gains(), home, 1000.0);
  bool bounded = all_cst(edge);
  for (std::size_t i = 0; i < kActuatorCount; ++i) bounded = bounded && std::abs(edge.target_effort[i]) <= 1000;
  check(bounded && edge.target_effort[0] > 0, "compute: the home pose gives a bounded effort toward the target");

  // f. 목표 필터(기본 10 Hz)는 측정 자세에서 시작해, 첫 주기에는 거의 0 이고 1 s 뒤에는 1° 오차의 effort 다.
  {
    Config filtered = thumb_j0_gains();
    filtered.filter_enabled = true;
    controllers::JointImpedanceController controller(kPeriodNs);
    JointImpedanceCommand command{};
    command.target = target;
    std::array<double, kActuatorCount> limit{};
    limit.fill(30000.0);
    canfd::CommandFrames frames{};
    controller.compute(command, filtered, state, limit, frames);
    const int first = frames.target_effort[0];
    for (int k = 1; k < 500; ++k) controller.compute(command, filtered, state, limit, frames);
    check(std::abs(first) <= 1 && frames.target_effort[0] >= 323 && frames.target_effort[0] <= 325,
          "compute: the target filter starts at the measured pose and settles on the target");
  }

  // g. 목표 속도 제한 1 rad/s 는 한 주기에 목표를 0.002 rad 만 옮긴다 → 324.30 × 0.1146 = 37.2.
  {
    Config limited = thumb_j0_gains();
    limited.max_target_velocity = 1.0;
    const auto slow = run_once(target, limited, state, 30000.0);
    check(slow.target_effort[0] >= 36 && slow.target_effort[0] <= 38, "compute: the target moves at most max_target_velocity");
  }

  // h. 새 stiffness 는 0.2 s 시정수로 다가간다: 1 → 0.5 로 바꾼 다음 주기는 0.995 → 322.7.
  {
    Config ramp = thumb_j0_gains();
    controllers::JointImpedanceController controller(kPeriodNs);
    JointImpedanceCommand command{};
    command.target = target;
    std::array<double, kActuatorCount> limit{};
    limit.fill(30000.0);
    canfd::CommandFrames frames{};
    controller.compute(command, ramp, state, limit, frames);
    ramp.stiffness[0] = 0.5;
    controller.compute(command, ramp, state, limit, frames);
    check(frames.target_effort[0] >= 321 && frames.target_effort[0] <= 324, "compute: a new stiffness is approached, not jumped to");
  }

  // 엄지 j0 2° 자세(d0 4197). i·j 는 d0 만 바꾼다.
  std::array<int, kActuatorCount> near_stop{};
  kinematics::ik_joint_to_actuator(pose_deg({2, 20, 0, 20}, {0, 20, 20}), near_stop);

  // i. −1° 목표는 d0 1000(0.486°)으로 올라간다.
  {
    const HandState at_2deg = state_from_counts(near_stop);
    Active below = active_of(at_2deg);
    below[0] = -1.0 * kDeg;
    std::array<int, kActuatorCount> margin_count = near_stop;
    margin_count[0] = 1000;
    Active margin = below;
    margin[0] = active_of(state_from_counts(margin_count))[0];
    const int raised = run_once(below, thumb_j0_gains(), at_2deg, 30000.0).target_effort[0];
    const int direct = run_once(margin, thumb_j0_gains(), at_2deg, 30000.0).target_effort[0];
    check(std::abs(raised - direct) <= 1, "compute: a target below count 1000 is raised to it");
  }

  // j. count 1000 아래에서 − 쪽 상한만 count 0 까지 0 으로 준다. d0 500, 30000 rpm → −500.
  {
    auto at = [&](int d0, double rpm) {
      std::array<int, kActuatorCount> count = near_stop;
      count[0] = d0;
      HandState s = state_from_counts(count);
      s.actuators.velocity_rpm[0] = rpm;
      return s;
    };
    const HandState zone = at(500, 30000);
    check(run_once(active_of(zone), thumb_j0_gains(), zone, 1000.0).target_effort[0] == -500,
          "compute: the effort toward the stop tapers below count 1000");
    const HandState resting = at(500, 0);
    Active out = active_of(resting);
    out[0] += 10.0 * kDeg;
    check(run_once(out, thumb_j0_gains(), resting, 1000.0).target_effort[0] == 1000,
          "compute: the effort away from the stop keeps max_effort");
    const HandState beyond = at(-200, 30000);
    check(run_once(active_of(beyond), thumb_j0_gains(), beyond, 1000.0).target_effort[0] == 0,
          "compute: past the stop nothing pushes toward it");
  }
}
#else
// ── Test 3: type c 는 Jacobian 이 없어 모든 actuator 에 CST 0 ──
void test_compute_type_c()
{
  JointImpedanceCommand command{};
  command.target.fill(0.3);
  std::array<double, kActuatorCount> max_effort{};
  max_effort.fill(1000.0);
  controllers::JointImpedanceController controller(kPeriodNs);
  canfd::CommandFrames frames{};
  controller.compute(command, Config{}, HandState{}, max_effort, frames);
  bool zero = true;
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    zero = zero && frames.mode_of_operation[i] == canfd::mode_of_operation::kCST && frames.target_effort[i] == 0;
  }
  check(zero, "compute: type c sends CST 0 to every actuator");
}
#endif

}  // namespace

int main()
{
#if !defined(AIDIN_HAND2_HAND_TYPE_C)
  test_joint_torque_to_effort();
  test_joint_impedance_torque();
  test_compute();
#else
  test_compute_type_c();
#endif

  std::printf("joint_impedance_controller test: %s\n", g_failures == 0 ? "OK" : "FAIL");
  return g_failures == 0 ? 0 : 1;
}
