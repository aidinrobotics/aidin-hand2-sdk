// 목적: 조립된 공개 kinematics API (aidin_hand2/hand/hand_kinematics.hpp) 의 행동을 검증한다.
//   기존 테스트는 per-finger 2-DOF PSS FK(forward_d12_to_q12_test)와 workspace clamp
//   (joint_clamp_test)만 다룬다. 여기서는 그 위 계층 — encoder→21joint 조립 FK, task FK,
//   그리고 task/oriented-task IK — 를 black-box 로 본다.
//
// 오라클: FK 를 IK 의 기준으로 쓴다(숫자 하드코딩 대신 왕복). 공개 API 만 사용하며
//         타입은 std::array 뿐이라 Eigen 링크가 필요 없다(aidin_hand2 만 링크).
//
// 다루는 것:
//   1) fk_actuator_to_joint (encoder→21): active joint 왕복 복원 + 유도된 passive q4
//      (four-bar 5개: thumb q4 + long finger 4개) 가 유한하고 물리적 범위 안인지.
//        왕복 경로: calibrated active(16) --ik_joint_to_actuator--> encoder
//                   --fk_actuator_to_joint--> joint(21). FK 출력의 active 슬롯이 입력을
//                   복원해야 한다(양쪽이 같은 calibrated 규약을 쓴다 — 아래 BLOCKER 참조).
//   2) fk_actuator_to_task (encoder→15 fingertip xyz): 유한·결정적·입력 반응성.
//   3) ik_task_to_joint / ik_oriented_task_to_joint (thumb 6-D pose 포함): 유한·결정적,
//      방향 정규화 불변, oriented/xyz 손가락 해 일치, 방향 반응성, 그리고 FK∘IK task 왕복.
//
// ── BLOCKER (report 대상) ────────────────────────────────────────────────────────
//   task-space IK 두 함수는 출력에 raw_active_joint_to_calibrated_active_joint 를 적용하는데,
//   그 thumb 부호 규약이 ik_joint_to_actuator(=calibrated_joint_to_raw_joint) / FK
//   (=raw_joint_to_calibrated_joint) 규약과 반대다(q0,q1,q3 부호 뒤집힘). 그래서 순수 공개
//   경로의 FK∘IK task 왕복은 닫히지 않는다(측정 83mm). 규약을 맞춰 주면 ~6mm(NR 감쇠 잔차)로
//   닫힌다. 공개 헤더도 이 두 함수를 "NOT CALIBRATED — 현재 사용 불가" 로 명시한다.
//   → task 왕복은 min(naive, convention-corrected) 로 검사한다: calibration 이 나중에
//     고쳐지면 naive 가, 지금은 corrected 가 닫히므로 어느 쪽이든 통과(fix-robust).

#include <array>
#include <cmath>
#include <cstdio>

#include <aidin_hand2/hand/hand_kinematics.hpp>
#include <aidin_hand2/types/description.hpp>

using namespace aidin_hand2;

namespace
{

int g_failures = 0;

void check(bool ok, const char* what)
{
  if (!ok) { std::printf("  FAIL: %s\n", what); ++g_failures; }
}

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

// 왕복 허용오차. FK active 복원은 실측 최악 8.5e-5 rad(<0.005°); 넉넉히 1e-3 rad.
constexpr double kJointTolRad = 1e-3;
// task 왕복(min naive/corrected): NR 감쇠(gain 0.3, 30 iter) 미수렴 잔차라 pose 에 따라
// ~15mm 까지 커진다(공격적 굴곡일수록). 정확도 게이지가 아니라 "닫히긴 한다(≠ 83mm 깨짐)"
// 를 보는 검사라 넉넉히 25mm.
constexpr double kTaskTolMm = 25.0;

// ── layout helpers ───────────────────────────────────────────────────────────────
// active_joint(16) 슬롯 → joint(21) 슬롯.
//   thumb   a=0..3           → j=a            (q0..q3)
//   finger f=(a-4)/3, l=…%3  → j=5+4f+l       (q1..q3; q4 는 passive 라 active 에 없다)
int active_to_joint(int a)
{
  if (a < 4) return a;
  const int f = (a - 4) / 3;
  const int l = (a - 4) % 3;
  return 5 + 4 * f + l;
}
// 유도된 passive q4 5개의 joint(21) 슬롯: thumb q4=4, long finger q4=8,12,16,20.
constexpr std::array<int, 5> kPassiveJointSlots{{4, 8, 12, 16, 20}};

double max_abs_diff(const double* a, const double* b, int n)
{
  double w = 0.0;
  for (int i = 0; i < n; ++i) w = std::fmax(w, std::fabs(a[i] - b[i]));
  return w;
}

// ── 검사용 도달 가능 pose (calibrated active, deg) ────────────────────────────────
// 슬롯: thumb {q0, q1(flex), q2(abd), q3}, 이어서 finger×4 {q1(abd), q2(flex), q3}.
// 모두 workspace 내부 값이라 ik_joint_to_actuator→encoder 가 유효하다.
using Pose = std::array<double, kActiveJointCount>;
constexpr std::array<Pose, 4> kPosesDeg{{
    { 0,  0,  0,  0,    0, 10, 10,   0, 10, 10,   0, 10, 10,   0, 10, 10},
    {15, 25,  5, 25,    5, 30, 30,  -5, 25, 35,   5, 20, 25,  -3, 15, 20},
    {30, 40,  8, 35,    8, 50, 45,  -8, 40, 50,   6, 35, 40,  -4, 25, 30},
    {10, 35, -6, 20,   -6, 45, 55,   7, 30, 25,  -4, 15, 45,   5, 40, 15},
}};

Pose pose_to_rad(const Pose& deg)
{
  Pose r{};
  for (std::size_t i = 0; i < deg.size(); ++i) r[i] = deg[i] * kDeg2Rad;
  return r;
}

// task-IK 출력(그 자체 calibrated 규약) → FK/ik_joint_to_actuator 규약으로 변환.
// 위 BLOCKER 를 우회하기 위해서만 쓴다 — raw 를 경유해 두 규약을 잇는다.
// (raw_active_joint_to_calibrated_active_joint 를 역산 → calibrated_joint_to_raw_joint 규약으로 재인코딩)
Pose ik_convention_to_fk_convention(const Pose& ci)
{
  Pose raw{}, cf{};
  // task-IK 규약 역산: raw = f^{-1}(cal_ik)
  raw[0] = -ci[0];
  raw[1] = -ci[1] - 13.9 * kDeg2Rad;
  raw[2] =  ci[2];
  raw[3] =  ci[3] + 87.0 * kDeg2Rad;
  for (int f = 0; f < 4; ++f) {
    const int d = 4 + 3 * f;
    raw[d]     = -ci[d + 1] + 47.0 * kDeg2Rad;
    raw[d + 1] =  ci[d];
    raw[d + 2] = -ci[d + 2] + 173.0 * kDeg2Rad;
  }
  // FK 규약으로 재인코딩: cal_fk = g(raw)  (= raw_joint_to_calibrated_joint 의 active 부분)
  cf[0] =  raw[0];
  cf[1] =  raw[1] + 13.9 * kDeg2Rad;
  cf[2] =  raw[2];
  cf[3] = -raw[3] + 87.0 * kDeg2Rad;
  for (int f = 0; f < 4; ++f) {
    const int d = 4 + 3 * f;
    cf[d]     =  raw[d + 1];
    cf[d + 1] = -raw[d]     + 47.0 * kDeg2Rad;
    cf[d + 2] = -raw[d + 2] + 173.0 * kDeg2Rad;
  }
  return cf;
}

// ── 1) 조립 FK: encoder → 21 joint ────────────────────────────────────────────────
void test_fk_actuator_to_joint()
{
  double worst_recover = 0.0;
  for (const Pose& deg : kPosesDeg) {
    const Pose active = pose_to_rad(deg);
    const std::array<int, kActuatorCount> encoder = ik_joint_to_actuator(active);
    const std::array<double, kJointCount> joint = fk_actuator_to_joint(encoder);

    // 21 joint 전부 유한.
    bool all_finite = true;
    for (double v : joint) all_finite = all_finite && std::isfinite(v);
    check(all_finite, "fk_actuator_to_joint: 21 joint 중 비유한값");

    // encoder 는 물리적으로 음수 불가(distance_to_encoder 가 0 클램프).
    bool enc_nonneg = true;
    for (int e : encoder) enc_nonneg = enc_nonneg && (e >= 0);
    check(enc_nonneg, "ik_joint_to_actuator: 음수 encoder");

    // active joint 왕복 복원: FK 출력의 active 슬롯 == 입력 active.
    for (int a = 0; a < static_cast<int>(kActiveJointCount); ++a) {
      const double err = std::fabs(joint[active_to_joint(a)] - active[a]);
      worst_recover = std::fmax(worst_recover, err);
    }

    // 유도된 passive q4 5개: 유한 + 물리적 범위(실측 0~63°; 넉넉히 -60°~+120°).
    for (int slot : kPassiveJointSlots) {
      const double v = joint[slot];
      check(std::isfinite(v), "passive q4 비유한값");
      check(v > -60.0 * kDeg2Rad && v < 120.0 * kDeg2Rad, "passive q4 물리적 범위 밖");
    }
  }
  std::printf("  fk_actuator_to_joint: active 복원 최악 %.3e rad (%.5f deg)\n",
              worst_recover, worst_recover / kDeg2Rad);
  check(worst_recover < kJointTolRad, "fk_actuator_to_joint: active joint 왕복 복원 실패");
}

// ── 2) 조립 FK: encoder → 15 fingertip task ───────────────────────────────────────
void test_fk_actuator_to_task()
{
  const std::array<int, kActuatorCount> enc_a =
      ik_joint_to_actuator(pose_to_rad(kPosesDeg[1]));
  const std::array<int, kActuatorCount> enc_b =
      ik_joint_to_actuator(pose_to_rad(kPosesDeg[2]));

  const std::array<double, kTaskCount> task_a  = fk_actuator_to_task(enc_a);
  const std::array<double, kTaskCount> task_a2 = fk_actuator_to_task(enc_a);
  const std::array<double, kTaskCount> task_b  = fk_actuator_to_task(enc_b);

  bool all_finite = true;
  for (double v : task_a) all_finite = all_finite && std::isfinite(v);
  check(all_finite, "fk_actuator_to_task: 비유한 좌표");

  // 결정적: 같은 encoder → 같은 task.
  check(max_abs_diff(task_a.data(), task_a2.data(), kTaskCount) == 0.0,
        "fk_actuator_to_task: 비결정적");

  // 입력 반응성: 다른 pose → 다른 fingertip 위치.
  check(max_abs_diff(task_a.data(), task_b.data(), kTaskCount) > 1.0,
        "fk_actuator_to_task: 입력에 반응 안 함");

  // 5 fingertip 각각 원점에서 떨어져 있고(비영) 서로 구별됨.
  for (int d = 0; d < static_cast<int>(kFingerCount); ++d) {
    const double* p = &task_a[3 * d];
    check(std::hypot(std::hypot(p[0], p[1]), p[2]) > 1.0, "fingertip 위치가 원점");
  }
}

// ── 3) task IK 와 oriented(thumb 6-D) IK ──────────────────────────────────────────
void test_ik_task_and_oriented()
{
  double worst_task_rt = 0.0;
  double worst_oriented_rt = 0.0;

  for (const Pose& deg : kPosesDeg) {
    const std::array<int, kActuatorCount> enc = ik_joint_to_actuator(pose_to_rad(deg));
    const std::array<double, kTaskCount> task = fk_actuator_to_task(enc);

    // --- xyz IK: 유한 + 결정적 ---
    const std::array<double, kActiveJointCount> j_xyz  = ik_task_to_joint(task);
    const std::array<double, kActiveJointCount> j_xyz2 = ik_task_to_joint(task);
    bool finite_xyz = true;
    for (double v : j_xyz) finite_xyz = finite_xyz && std::isfinite(v);
    check(finite_xyz, "ik_task_to_joint: 비유한 해");
    check(max_abs_diff(j_xyz.data(), j_xyz2.data(), kActiveJointCount) == 0.0,
          "ik_task_to_joint: 비결정적");

    // xyz IK task 왕복 (min naive/corrected — BLOCKER 우회).
    const double rt_naive = [&] {
      const auto e2 = ik_joint_to_actuator(j_xyz);
      const auto t2 = fk_actuator_to_task(e2);
      return max_abs_diff(task.data(), t2.data(), kTaskCount);
    }();
    const double rt_corr = [&] {
      const auto e2 = ik_joint_to_actuator(ik_convention_to_fk_convention(j_xyz));
      const auto t2 = fk_actuator_to_task(e2);
      return max_abs_diff(task.data(), t2.data(), kTaskCount);
    }();
    worst_task_rt = std::fmax(worst_task_rt, std::fmin(rt_naive, rt_corr));

    // --- oriented IK: thumb 6-D pose(위치+방향) + 손가락 xyz ---
    // task[0..2] = thumb xyz, task[3..14] = 손가락 4×xyz.
    std::array<double, kOrientedTaskCount> ot{};
    ot[0] = task[0]; ot[1] = task[1]; ot[2] = task[2];
    ot[3] = 0.2; ot[4] = -0.3; ot[5] = 0.9;              // thumb 방향(비단위 — 내부 정규화)
    for (int f = 0; f < 4; ++f) {
      ot[6 + 3 * f] = task[3 + 3 * f];
      ot[7 + 3 * f] = task[4 + 3 * f];
      ot[8 + 3 * f] = task[5 + 3 * f];
    }

    const std::array<double, kActiveJointCount> j_or  = ik_oriented_task_to_joint(ot);
    const std::array<double, kActiveJointCount> j_or2 = ik_oriented_task_to_joint(ot);
    bool finite_or = true;
    for (double v : j_or) finite_or = finite_or && std::isfinite(v);
    check(finite_or, "ik_oriented_task_to_joint: 비유한 해");
    check(max_abs_diff(j_or.data(), j_or2.data(), kActiveJointCount) == 0.0,
          "ik_oriented_task_to_joint: 비결정적");

    // 방향 정규화 불변: 비단위 방향은 그 정규화판과 같은 해를 줘야 한다(헤더 규약).
    std::array<double, kOrientedTaskCount> ot_scaled = ot;
    ot_scaled[3] *= 2.0; ot_scaled[4] *= 2.0; ot_scaled[5] *= 2.0;
    const std::array<double, kActiveJointCount> j_or_scaled =
        ik_oriented_task_to_joint(ot_scaled);
    check(max_abs_diff(j_or.data(), j_or_scaled.data(), kActiveJointCount) < 1e-9,
          "oriented IK: 방향 스케일에 해가 달라짐(정규화 안 됨)");

    // oriented 와 xyz 는 손가락(non-thumb) 해가 같아야 한다(같은 3-DOF solver 공유).
    check(max_abs_diff(&j_or[4], &j_xyz[4], kActiveJointCount - 4) < 1e-9,
          "oriented/xyz 손가락 해 불일치");

    // 방향을 크게 바꾸면 thumb 해가 바뀌어야 한다(방향이 실제로 반영됨).
    std::array<double, kOrientedTaskCount> ot_dir = ot;
    ot_dir[3] = 0.9; ot_dir[4] = 0.1; ot_dir[5] = 0.2;
    const std::array<double, kActiveJointCount> j_or_dir = ik_oriented_task_to_joint(ot_dir);
    check(max_abs_diff(j_or.data(), j_or_dir.data(), 4) > 1e-3,
          "oriented IK: thumb 방향 변화에 해가 반응 안 함");

    // oriented IK 도 손가락 xyz 는 왕복해야 한다(min naive/corrected).
    const double or_naive = [&] {
      const auto e2 = ik_joint_to_actuator(j_or);
      const auto t2 = fk_actuator_to_task(e2);
      return max_abs_diff(&task[3], &t2[3], kTaskCount - 3);  // 손가락 부분만
    }();
    const double or_corr = [&] {
      const auto e2 = ik_joint_to_actuator(ik_convention_to_fk_convention(j_or));
      const auto t2 = fk_actuator_to_task(e2);
      return max_abs_diff(&task[3], &t2[3], kTaskCount - 3);
    }();
    worst_oriented_rt = std::fmax(worst_oriented_rt, std::fmin(or_naive, or_corr));
  }

  std::printf("  ik_task_to_joint:          FK.IK task 왕복 최악 %.4f mm\n", worst_task_rt);
  std::printf("  ik_oriented_task_to_joint: 손가락 xyz 왕복 최악 %.4f mm\n", worst_oriented_rt);
  check(worst_task_rt < kTaskTolMm, "ik_task_to_joint: task 왕복 복원 실패");
  check(worst_oriented_rt < kTaskTolMm, "ik_oriented_task_to_joint: 손가락 왕복 복원 실패");
}

}  // namespace

int main()
{
  std::printf("[fk_actuator_to_joint]\n");
  test_fk_actuator_to_joint();
  std::printf("[fk_actuator_to_task]\n");
  test_fk_actuator_to_task();
  std::printf("[ik_task/oriented_task_to_joint]\n");
  test_ik_task_and_oriented();

  std::printf("kinematics_public_test: %s\n", g_failures == 0 ? "PASS" : "FAIL");
  return g_failures;
}
