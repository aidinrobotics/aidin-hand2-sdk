// 목적: Protocol 공개 계약의 byte_order_test 미커버 영역을 black-box 검증한다.
//   1) decode() 결과 분류 — Unknown / LengthError / ConflictingCommand / Error / Decoded
//      (velocity INT32, current INT16, tactile finger, palm upper/lower/palm2).
//   2) frames_to_hand_state() — tactile 5-finger 매핑, palm 3-pad → 58 cell 평탄화 index 산술.
//   3) frames_to_actuator_health() — statusword enabled 마스크, error_code → ActuatorFault 매핑.
//   4) HandSide::Right — 왼손 ID 는 Unknown, 오른손 ID 는 Decoded (side 별 ID 선택).
// byte_order_test 가 이미 다루는 LE/encode-decode round-trip(position/statusword/effort)은 중복하지 않는다.

#include <linux/can.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include <aidin_hand2/types/state.hpp>  // HandState, ActuatorHealth, ActuatorFault

#include "hand_core/comms/canfd/protocol.hpp"

namespace
{

int g_failures = 0;

void check(bool ok, const char* what)
{
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    ++g_failures;
  }
}

using namespace aidin_hand2;
using namespace aidin_hand2::canfd;

// LE16 오라클 (라이브러리 memcpy 와 독립) — payload 를 손으로 채워 넣는 데 쓴다.
void put_le16(std::uint8_t* b, std::uint16_t v)
{
  b[0] = std::uint8_t(v);
  b[1] = std::uint8_t(v >> 8);
}
void put_le32(std::uint8_t* b, std::int32_t s)
{
  const std::uint32_t v = static_cast<std::uint32_t>(s);
  b[0] = std::uint8_t(v);
  b[1] = std::uint8_t(v >> 8);
  b[2] = std::uint8_t(v >> 16);
  b[3] = std::uint8_t(v >> 24);
}

// 편의: can_id/len 만 세팅한 frame.
canfd_frame make_frame(std::uint32_t can_id, std::uint8_t len)
{
  canfd_frame rx{};
  rx.can_id = can_id;
  rx.len = len;
  return rx;
}

// ── 1) decode() 결과 분류 ────────────────────────────────────────────────
void test_decode_classification()
{
  Protocol protocol(HandSide::Left);
  // rx_filters 순서(protocol.cpp): [0]=position [1]=velocity [2]=current [3]=status_word
  //   [4..8]=finger0..4 [9]=palm_upper [10]=palm_lower [11]=palm2 [12..16]=command 5.
  const auto f = protocol.rx_filters();
  check(f.size() == 17, "rx_filters: 12 state + 5 command = 17 ids");

  StateFrames sf{};

  // Unknown: 이 손의 ID 집합에 없는 can_id.
  {
    auto rx = make_frame(0x7A5, 64);
    check(protocol.decode(rx, sf) == DecodeResult::Unknown, "decode: unknown can_id -> Unknown");
  }

  // Error: CAN_ERR_FLAG 는 can_id 매칭보다 우선한다 (state id 위에 flag 를 얹어도 Error).
  {
    auto rx = make_frame(f[0].can_id | CAN_ERR_FLAG, 64);
    check(protocol.decode(rx, sf) == DecodeResult::Error, "decode: CAN_ERR_FLAG -> Error");
  }

  // LengthError: 알려진 state id 인데 페이로드가 규약 최소치 미달.
  {
    // position 은 INT32×16 = 64B 필요. 63B 는 미달.
    auto rx = make_frame(f[0].can_id, 63);
    check(protocol.decode(rx, sf) == DecodeResult::LengthError, "decode: short position -> LengthError");
    // current 는 INT16×16 = 32B 필요. 31B 미달.
    auto rx2 = make_frame(f[2].can_id, 31);
    check(protocol.decode(rx2, sf) == DecodeResult::LengthError, "decode: short current -> LengthError");
    // finger 는 UINT16×17 = 34B 필요. 33B 미달.
    auto rx3 = make_frame(f[4].can_id, 33);
    check(protocol.decode(rx3, sf) == DecodeResult::LengthError, "decode: short finger -> LengthError");
    // palm2 는 UINT16×18 = 36B 필요. 35B 미달.
    auto rx4 = make_frame(f[11].can_id, 35);
    check(protocol.decode(rx4, sf) == DecodeResult::LengthError, "decode: short palm2 -> LengthError");
  }

  // ConflictingCommand: 이 손의 command id 수신 (host→hand 전용이므로 남이 명령 중).
  {
    // command id 는 rx_filters[12..16]. 각각 확인.
    for (std::size_t i = 12; i < 17; ++i) {
      auto rx = make_frame(f[i].can_id, 64);
      check(protocol.decode(rx, sf) == DecodeResult::ConflictingCommand,
            "decode: command id -> ConflictingCommand");
    }
  }

  // Decoded: velocity INT32 — 정확 길이(64)에서 LE int32 를 읽는다.
  {
    auto rx = make_frame(f[1].can_id, 64);
    put_le32(rx.data + 0 * 4, -12345);
    put_le32(rx.data + 3 * 4, 0x0055AA33);
    StateFrames v{};
    check(protocol.decode(rx, v) == DecodeResult::Decoded, "decode velocity: result Decoded");
    check(v.actual_velocity[0] == -12345, "decode velocity: LE int32 slot0 (signed)");
    check(v.actual_velocity[3] == 0x0055AA33, "decode velocity: LE int32 slot3");
  }

  // Decoded: current INT16 — 32B 에서 LE int16, 부호 포함.
  {
    auto rx = make_frame(f[2].can_id, 32);
    put_le16(rx.data + 0 * 2, static_cast<std::uint16_t>(static_cast<std::int16_t>(-2)));  // 0xFFFE
    put_le16(rx.data + 5 * 2, 0x1234);
    StateFrames c{};
    check(protocol.decode(rx, c) == DecodeResult::Decoded, "decode current: result Decoded");
    check(c.actual_current[0] == -2, "decode current: LE int16 slot0 (signed)");
    check(c.actual_current[5] == 0x1234, "decode current: LE int16 slot5");
  }

  // Decoded: tactile finger (thumb=finger0) — UINT16×17.
  {
    auto rx = make_frame(f[4].can_id, static_cast<std::uint8_t>(kTactileTaxelsPerFinger * 2));
    put_le16(rx.data + 0 * 2, 0xABCD);
    put_le16(rx.data + (kTactileTaxelsPerFinger - 1) * 2, 0x0102);
    StateFrames t{};
    check(protocol.decode(rx, t) == DecodeResult::Decoded, "decode tactile thumb: result Decoded");
    check(t.tactile_thumb[0] == 0xABCD, "decode tactile thumb: taxel0");
    check(t.tactile_thumb[kTactileTaxelsPerFinger - 1] == 0x0102, "decode tactile thumb: last taxel");
  }

  // Decoded: palm frames upper/lower/palm2 — 각 최소 길이에서 Decoded.
  {
    auto up = make_frame(f[9].can_id, static_cast<std::uint8_t>(kPalm1UpperCount * 2));
    put_le16(up.data + 0, 0x1111);
    StateFrames s{};
    check(protocol.decode(up, s) == DecodeResult::Decoded, "decode palm upper: Decoded");
    check(s.palm1_upper[0] == 0x1111, "decode palm upper: cell0");

    auto lo = make_frame(f[10].can_id, static_cast<std::uint8_t>(kPalm1LowerCount * 2));
    put_le16(lo.data + 0, 0x2222);
    check(protocol.decode(lo, s) == DecodeResult::Decoded, "decode palm lower: Decoded");
    check(s.palm1_lower[0] == 0x2222, "decode palm lower: cell0");

    auto p2 = make_frame(f[11].can_id, static_cast<std::uint8_t>(kPalm2Count * 2));
    put_le16(p2.data + 0, 0x3333);
    check(protocol.decode(p2, s) == DecodeResult::Decoded, "decode palm2: Decoded");
    check(s.palm2[0] == 0x3333, "decode palm2: cell0");
  }

  // Decoded: 규약 길이보다 '긴' frame (CAN-FD DLC 패딩) 은 통과해야 한다 ('<' 만 거른다).
  {
    auto rx = make_frame(f[2].can_id, 64);  // current 는 32B 필요, 64B(패딩) 은 OK
    StateFrames c{};
    check(protocol.decode(rx, c) == DecodeResult::Decoded, "decode: padded (len>min) current -> Decoded");
  }
}

// ── 2) frames_to_hand_state(): tactile 매핑 + palm 평탄화 ─────────────────
void test_frames_to_hand_state()
{
  Protocol protocol(HandSide::Left);
  StateFrames sf{};

  // 손가락별로 taxel i 를 (finger_base + i) 로 채워, 손가락 매핑 뒤섞임을 잡는다.
  const std::uint16_t finger_base[kFingerCount] = {1000, 2000, 3000, 4000, 5000};
  for (std::size_t i = 0; i < kTactileTaxelsPerFinger; ++i) {
    sf.tactile_thumb[i]  = static_cast<std::uint16_t>(finger_base[0] + i);
    sf.tactile_index[i]  = static_cast<std::uint16_t>(finger_base[1] + i);
    sf.tactile_middle[i] = static_cast<std::uint16_t>(finger_base[2] + i);
    sf.tactile_ring[i]   = static_cast<std::uint16_t>(finger_base[3] + i);
    sf.tactile_baby[i]   = static_cast<std::uint16_t>(finger_base[4] + i);
  }

  // palm 3-pad: 값 자체에 '어느 pad + local index' 를 인코딩 (upper=10000+, lower=20000+, palm2=30000+).
  for (std::size_t i = 0; i < kPalm1UpperCount; ++i)
    sf.palm1_upper[i] = static_cast<std::uint16_t>(10000 + i);
  for (std::size_t i = 0; i < kPalm1LowerCount; ++i)
    sf.palm1_lower[i] = static_cast<std::uint16_t>(20000 + i);
  for (std::size_t i = 0; i < kPalm2Count; ++i)
    sf.palm2[i] = static_cast<std::uint16_t>(30000 + i);

  // actuator 관측 수치도 몇 개 세팅해 cast 경로 확인.
  sf.actual_position[7] = -777;
  sf.actual_velocity[2] = 321;
  sf.actual_current[9]  = -50;

  HandState state{};
  protocol.frames_to_hand_state(sf, state);

  // tactile 5-finger 매핑: fingers[f][i] == finger_base[f] + i.
  bool finger_ok = true;
  for (std::size_t fdx = 0; fdx < kFingerCount; ++fdx)
    for (std::size_t i = 0; i < kTactileTaxelsPerFinger; ++i)
      finger_ok &= (state.tactile.fingers[fdx][i] == static_cast<double>(finger_base[fdx] + i));
  check(finger_ok, "frames_to_hand_state: tactile 5-finger mapping (thumb,index,middle,ring,baby)");

  // palm 평탄화 index 산술을 독립 오라클로 확인:
  //   [0 .. 19]   <- palm1_upper[i]          (== 10000+i)
  //   [20 .. 39]  <- palm1_lower[i]          (== 20000+i)
  //   [40 .. 57]  <- palm2[i]                (== 30000+i)
  bool palm_ok = (kPalmTactileCount == kPalm1UpperCount + kPalm1LowerCount + kPalm2Count);
  check(palm_ok, "frames_to_hand_state: palm total = 20+20+18 = 58");
  bool upper_ok = true, lower_ok = true, palm2_ok = true;
  for (std::size_t i = 0; i < kPalm1UpperCount; ++i)
    upper_ok &= (state.tactile.palm[i] == static_cast<double>(10000 + i));
  for (std::size_t i = 0; i < kPalm1LowerCount; ++i)
    lower_ok &= (state.tactile.palm[kPalm1UpperCount + i] == static_cast<double>(20000 + i));
  for (std::size_t i = 0; i < kPalm2Count; ++i)
    palm2_ok &= (state.tactile.palm[kPalm1UpperCount + kPalm1LowerCount + i] == static_cast<double>(30000 + i));
  check(upper_ok, "frames_to_hand_state: palm[0..19] <- palm1_upper");
  check(lower_ok, "frames_to_hand_state: palm[20..39] <- palm1_lower");
  check(palm2_ok, "frames_to_hand_state: palm[40..57] <- palm2");

  // actuator 수치 cast.
  check(state.actuators.position_count[7] == -777.0, "frames_to_hand_state: position_count cast");
  check(state.actuators.velocity_rpm[2] == 321.0, "frames_to_hand_state: velocity_rpm cast");
  check(state.actuators.current_mA[9] == -50.0, "frames_to_hand_state: current_mA cast");
}

// ── 3) frames_to_actuator_health(): enabled 마스크 + fault 매핑 ───────────
void test_frames_to_actuator_health()
{
  Protocol protocol(HandSide::Left);
  StateFrames sf{};

  // enabled 판정: (status & 0x6F) == 0x27. mask 밖 비트(4=0x10, 7=0x80)는 don't-care.
  sf.status_word[0] = 0x0027;                 // 정확한 Operation Enable -> true
  sf.status_word[1] = 0x0027 | 0x10 | 0x80;   // mask 밖 비트 켜짐 -> 여전히 true
  sf.status_word[2] = 0x0000;                 // -> false
  sf.status_word[3] = 0x002F;                 // bit3(0x08) 켜짐 -> false
  sf.status_word[4] = 0x0067;                 // bit6(0x40) 켜짐 -> false
  sf.status_word[5] = 0x0023;                 // bit2(0x04) 빠짐 -> false

  // fault 매핑: error_code (uint16) 를 ActuatorFault enum 으로.
  sf.error_code[0] = 0x0000;  // None
  sf.error_code[1] = 0x2310;  // OverCurrentError
  sf.error_code[2] = 0x8611;  // FollowingError
  sf.error_code[3] = 0xFF24;  // SerialEncoderChannelAError
  sf.error_code[4] = 0x1234;  // 이름 없는 값 -> 그대로 캐스팅 (drop 없음)

  ActuatorHealth health{};
  protocol.frames_to_actuator_health(sf, health);

  // enabled: 독립 오라클 (mask 재구현) 로 대조.
  auto oracle_enabled = [](std::uint16_t sw) { return (sw & 0x6F) == 0x27; };
  bool enabled_ok = true;
  for (std::size_t i = 0; i < 6; ++i)
    enabled_ok &= (health.enabled[i] == oracle_enabled(sf.status_word[i]));
  check(enabled_ok, "frames_to_actuator_health: enabled mask (0x6F)==0x27 incl. don't-care bits");
  // 개별 기대값도 명시.
  check(health.enabled[0] == true,  "health.enabled[0]: 0x0027 -> enabled");
  check(health.enabled[1] == true,  "health.enabled[1]: 0x00B7 (mask-외 비트) -> enabled");
  check(health.enabled[2] == false, "health.enabled[2]: 0x0000 -> disabled");
  check(health.enabled[3] == false, "health.enabled[3]: 0x002F (bit3) -> disabled");
  check(health.enabled[4] == false, "health.enabled[4]: 0x0067 (bit6) -> disabled");
  check(health.enabled[5] == false, "health.enabled[5]: 0x0023 (bit2 missing) -> disabled");

  // fault: error_code -> ActuatorFault.
  check(health.fault[0] == ActuatorFault::None, "health.fault[0]: 0x0000 -> None");
  check(health.fault[1] == ActuatorFault::OverCurrentError, "health.fault[1]: 0x2310 -> OverCurrentError");
  check(health.fault[2] == ActuatorFault::FollowingError, "health.fault[2]: 0x8611 -> FollowingError");
  check(health.fault[3] == ActuatorFault::SerialEncoderChannelAError,
        "health.fault[3]: 0xFF24 -> SerialEncoderChannelAError");
  check(health.fault[4] == static_cast<ActuatorFault>(0x1234),
        "health.fault[4]: unnamed 0x1234 -> raw cast (no drop)");
}

// ── 4) HandSide::Right: side 별 CAN ID 선택 ──────────────────────────────
void test_right_side_ids()
{
  Protocol left(HandSide::Left);
  Protocol right(HandSide::Right);

  const auto lf = left.rx_filters();
  const auto rf = right.rx_filters();
  // 왼손 0x2xx / 오른손 0x1xx — 첫 필터(state_position) 로 side 별 ID 대역 확인.
  check(lf[0].can_id == 0x221, "left rx_filters[0] = 0x221 (state_position)");
  check(rf[0].can_id == 0x121, "right rx_filters[0] = 0x121 (state_position)");

  StateFrames sf{};

  // 오른손 protocol 에 왼손 state id 를 주면 자기 집합이 아니므로 Unknown.
  {
    auto rx = make_frame(0x221, 64);
    check(right.decode(rx, sf) == DecodeResult::Unknown, "right protocol: left id 0x221 -> Unknown");
  }
  // 반대로 왼손 protocol 에 오른손 id 도 Unknown.
  {
    auto rx = make_frame(0x121, 64);
    check(left.decode(rx, sf) == DecodeResult::Unknown, "left protocol: right id 0x121 -> Unknown");
  }
  // 오른손 protocol 이 오른손 position id 를 정상 decode.
  {
    auto rx = make_frame(0x121, 64);
    put_le32(rx.data + 0, 0x0A0B0C0D);
    StateFrames v{};
    check(right.decode(rx, v) == DecodeResult::Decoded, "right protocol: right id 0x121 -> Decoded");
    check(v.actual_position[0] == 0x0A0B0C0D - canfd::kHomeOffsetCount,
          "right protocol: decoded LE int32 minus home offset");
  }
  // 오른손 command id (0x111) 는 ConflictingCommand.
  {
    auto rx = make_frame(0x111, 64);
    check(right.decode(rx, sf) == DecodeResult::ConflictingCommand,
          "right protocol: right cmd id 0x111 -> ConflictingCommand");
  }
}

}  // namespace

int main()
{
  test_decode_classification();
  test_frames_to_hand_state();
  test_frames_to_actuator_health();
  test_right_side_ids();

  if (g_failures == 0) {
    std::printf("protocol_test: PASS\n");
    return 0;
  }
  std::printf("protocol_test: %d failure(s)\n", g_failures);
  return g_failures;
}
