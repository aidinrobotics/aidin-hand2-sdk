// 목적(Tier 1, HW 무관): JointPositionController 의 filter 계약을 순수 host 에서 회귀 점검한다.
// controller 출력은 actuator count(IK 결과)이므로, "이 관절각을 태웠다" 계약은 동일 IK 를 태운
// 기준 프레임과 정수로 비교해 검증한다(FK 오차 무관).
//   1) 저역통과: 먼 목표를 한 cycle 에 통과시키지 않고 여러 cycle 에 걸쳐 수렴한다.
//   2) send-on-delta deadband: 직전 채택값에서 임계 안의 변화는 출력을 바꾸지 않고, 임계를 넘는
//      변화는 target 이 그대로 통과한다. 정착값은 target 과 같아 정상 오차가 남지 않는다.
//   3) reset() 후 재진입: stale 목표가 아니라 관측 자세에서 시작한다.
//   4) filter_enabled=false: 두 단계를 우회해 목표가 그대로 통과한다.
// IK/FK 자체 정합은 command_to_frames_test 가 담당 — 여기서는 필터 상태 로직만 본다.

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>

#include <aidin_hand2/types/command.hpp>       // JointPositionCommand
#include <aidin_hand2/types/config.hpp>        // ControllerConfig
#include <aidin_hand2/types/description.hpp>   // kActuatorCount, kActiveJointCount
#include <aidin_hand2/types/state.hpp>         // HandState

#include "hand_core/comms/canfd/protocol.hpp"  // CommandFrames, mode_of_operation::kCSP
#include "hand_core/controllers/joint_position_controller.hpp"
#include "hand_core/kinematics/hand_kinematics_core.hpp"

using namespace aidin_hand2;

namespace
{
int g_failures = 0;

void check(bool ok, const char* what)
{
  if (!ok) { std::printf("  FAIL: %s\n", what); ++g_failures; }
}

// 실기와 동일한 제어 주기 — 필터 계수가 dt 에 의존하므로 500Hz 를 그대로 쓴다.
constexpr long kPeriodNs = 2'000'000;  // 2 ms

// 기본 config 로 계약을 본다 — deadband 도 여기서 읽어 기본값과 테스트가 어긋나지 않게 한다.
const ControllerConfig::JointPositionController kConfig{};
const double kDeadbandRad = kConfig.deadband;

// active_joint(16) 인덱스 → joint(21) 인덱스. thumb q0..3, long finger 별 q1..3 (passive q4 제외).
std::array<std::size_t, kActiveJointCount> active_to_joint_index()
{
  std::array<std::size_t, kActiveJointCount> map{};
  for (std::size_t i = 0; i < 4; ++i) map[i] = i;  // thumb
  for (std::size_t f = 0; f < 4; ++f) {
    const std::size_t active_base = 4 + 3 * f;
    const std::size_t joint_base  = 5 + 4 * f;
    for (std::size_t k = 0; k < 3; ++k) map[active_base + k] = joint_base + k;
  }
  return map;
}

// 도달가능한 active_joint 목표를 얻는다: 균일 encoder → FK → active_joint(rad).
std::array<double, kActiveJointCount> active_from_uniform_encoder(double encoder_scalar)
{
  const auto map = active_to_joint_index();
  std::array<int, kinematics::ACTUATOR_NUM> encoder{};
  encoder.fill(static_cast<int>(encoder_scalar));
  std::array<double, kinematics::JOINT_NUM> joint{};
  std::array<double, kinematics::TASK_NUM>  task{};
  kinematics::fk_actuator_to_joint(encoder, joint, task);
  std::array<double, kActiveJointCount> active{};
  for (std::size_t k = 0; k < kActiveJointCount; ++k) active[k] = joint[map[k]];
  return active;
}

// 균일 encoder 자세를 관측 state 로. compute 가 재진입 시 이 자세에서 seed 한다.
HandState state_from_uniform_encoder(double encoder_scalar)
{
  std::array<int, kinematics::ACTUATOR_NUM> encoder{};
  encoder.fill(static_cast<int>(encoder_scalar));
  std::array<double, kinematics::JOINT_NUM> joint{};
  std::array<double, kinematics::TASK_NUM>  task{};
  kinematics::fk_actuator_to_joint(encoder, joint, task);
  HandState state{};
  state.joints.position_rad = joint;
  return state;
}

bool same_positions(const canfd::CommandFrames& a, const canfd::CommandFrames& b)
{
  for (std::size_t i = 0; i < kActuatorCount; ++i)
    if (a.target_position[i] != b.target_position[i]) return false;
  return true;
}

bool all_csp(const canfd::CommandFrames& f)
{
  for (std::size_t i = 0; i < kActuatorCount; ++i)
    if (f.mode_of_operation[i] != canfd::mode_of_operation::kCSP) return false;
  return true;
}

// 주어진 관절각을 그대로 IK 한 기준 프레임. 필터가 완전히 정착한 상태와 비교할 때 쓴다.
canfd::CommandFrames frames_of(const std::array<double, kActiveJointCount>& joint_rad)
{
  std::array<int, kActuatorCount> cnt{};
  kinematics::ik_joint_to_actuator(joint_rad, cnt);
  canfd::CommandFrames frames{};
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    frames.mode_of_operation[i] = canfd::mode_of_operation::kCSP;
    frames.target_position[i]   = static_cast<std::int32_t>(cnt[i]);
  }
  return frames;
}

// from → to 로 갈 때 send-on-delta deadband 가 정착하는 값: 임계를 넘으면 to 를 그대로 채택하고,
// 임계 안의 이동이면 움직이지 않는다(from 유지).
std::array<double, kActiveJointCount> settled(
    const std::array<double, kActiveJointCount>& from,
    const std::array<double, kActiveJointCount>& to)
{
  std::array<double, kActiveJointCount> out{};
  for (std::size_t j = 0; j < kActiveJointCount; ++j) {
    out[j] = (std::abs(to[j] - from[j]) >= kDeadbandRad) ? to[j] : from[j];
  }
  return out;
}

// 목표를 고정한 채 n cycle 진행.
canfd::CommandFrames drive(controllers::JointPositionController& c,
                           const std::array<double, kActiveJointCount>& target,
                           const HandState& state, int cycles,
                           const ControllerConfig::JointPositionController& config = kConfig)
{
  JointPositionCommand cmd{};
  cmd.target = target;
  canfd::CommandFrames frames{};
  for (int n = 0; n < cycles; ++n) c.compute(cmd, config, state, frames);
  return frames;
}

// ── Test 1: 먼 목표는 한 cycle 에 통과하지 않고 여러 cycle 에 걸쳐 수렴한다 ──
void test_lowpass_converges()
{
  const auto A = active_from_uniform_encoder(0.0);
  const auto B = active_from_uniform_encoder(4000.0);
  const HandState at_a = state_from_uniform_encoder(0.0);

  const canfd::CommandFrames jump = frames_of(B);
  check(!same_positions(frames_of(A), jump), "fixture: A 와 B 가 구별되는 count 를 낸다");

  controllers::JointPositionController controller(kPeriodNs);

  // cycle 1 — 관측 자세 A 에서 seed 하고 한 step 만 이동. 즉시 점프가 아니다.
  const canfd::CommandFrames f1 = drive(controller, B, at_a, 1);
  check(!same_positions(f1, jump), "cycle 1 은 목표로 점프하지 않는다");
  check(!same_positions(f1, frames_of(A)), "cycle 1 은 시작점에 머무르지도 않는다");
  check(all_csp(f1), "CSP 모드로 낸다");

  // 충분히 진행하면 목표에 정착한다 — 임계를 넘는 이동이므로 target 이 그대로 채택된다.
  const canfd::CommandFrames fn = drive(controller, B, at_a, 400);
  check(same_positions(fn, frames_of(settled(A, B))), "충분한 cycle 후 정착값에 도달한다");
  check(same_positions(fn, jump), "정착값은 목표와 같다 — 정상 오차가 남지 않는다");
}

// ── Test 2: send-on-delta deadband — 임계 안은 무시, 임계를 넘으면 target 그대로 ──
void test_deadband()
{
  const auto B = active_from_uniform_encoder(4000.0);
  const HandState at_a = state_from_uniform_encoder(0.0);

  controllers::JointPositionController controller(kPeriodNs);
  const canfd::CommandFrames base = drive(controller, B, at_a, 400);  // B 로 정착 (= B)

  // (a) 임계 안(0.4·deadband)에서 흔들어도 출력이 변하지 않는다.
  auto wiggle = B;
  for (double& v : wiggle) v -= 0.4 * kDeadbandRad;
  const canfd::CommandFrames held = drive(controller, wiggle, at_a, 50);
  check(same_positions(held, base), "임계 안 변화는 출력을 바꾸지 않는다");

  // (b) 임계를 넘는 변화(2·deadband)는 target 이 그대로 채택된다.
  auto forward = B;
  for (double& v : forward) v += 2.0 * kDeadbandRad;
  const canfd::CommandFrames moved = drive(controller, forward, at_a, 400);
  check(!same_positions(moved, base), "임계를 넘는 변화는 통과한다");
  check(same_positions(moved, frames_of(forward)),
        "임계를 넘으면 target 그대로 정착한다");
}

// ── Test 3: reset() 후 재진입은 stale 목표가 아니라 관측 자세에서 시작한다 ──
void test_reset_seeds_from_measured()
{
  const auto A = active_from_uniform_encoder(0.0);     // stale 상태를 만들 목표
  const auto B = active_from_uniform_encoder(4000.0);  // 새 목표
  const HandState at_a = state_from_uniform_encoder(0.0);
  const HandState at_c = state_from_uniform_encoder(2000.0);

  controllers::JointPositionController controller(kPeriodNs);
  drive(controller, A, at_a, 400);  // A 로 정착시켜 stale 상태를 만든다

  controller.reset();
  const canfd::CommandFrames f = drive(controller, B, at_c, 1);
  check(!same_positions(f, frames_of(B)), "reset 직후 첫 cycle 도 목표로 점프하지 않는다");

  // 대조군: reset 없이 같은 순서를 밟으면 관측 자세(C)와 무관하게 stale 값(A)에서 출발한다.
  controllers::JointPositionController control(kPeriodNs);
  drive(control, A, at_a, 400);
  const canfd::CommandFrames g = drive(control, B, at_c, 1);
  check(!same_positions(f, g), "reset 이 첫 cycle 출발점을 바꾼다(관측 자세 vs stale)");
}

// ── Test 4: filter_enabled=false 는 두 단계를 우회해 목표를 그대로 태운다 ──
void test_filter_disabled()
{
  const auto B = active_from_uniform_encoder(4000.0);
  const HandState at_a = state_from_uniform_encoder(0.0);

  ControllerConfig::JointPositionController off{};
  off.filter_enabled = false;

  controllers::JointPositionController controller(kPeriodNs);
  const canfd::CommandFrames f1 = drive(controller, B, at_a, 1, off);
  check(same_positions(f1, frames_of(B)), "첫 cycle 에 목표가 그대로 통과한다");
  check(all_csp(f1), "CSP 모드로 낸다");

  // 다시 켜면 목표에 붙어 있던 상태에서 이어간다 — 점프가 없다.
  const canfd::CommandFrames f2 = drive(controller, B, at_a, 1);
  check(same_positions(f2, frames_of(B)), "필터를 켜도 이미 목표에 있으므로 그대로 유지한다");
}

}  // namespace

int main()
{
  test_lowpass_converges();
  test_deadband();
  test_reset_seeds_from_measured();
  test_filter_disabled();

  if (g_failures == 0) std::printf("joint_position_controller_test: PASS\n");
  return g_failures;
}
