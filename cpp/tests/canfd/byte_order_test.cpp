// 목적: Protocol의 CAN payload 직렬화가 (1) little-endian, (2) 한 frame 안 두 영역
//       (statusword=status+error, effort=target+max) 레이아웃을 지키는지 검증.
// 방법: shift 기반 LE 오라클로 frame 바이트를 만들거나 읽어, decode/encode 결과와 대조한다.
//       (라이브러리는 memcpy로 직렬화 → LE host에서만 오라클과 일치)

#include <linux/can.h>

#include <cstdint>
#include <cstdio>
#include <cstring>

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

// shift 기반 LE 오라클 (라이브러리 memcpy와 독립).
void put_le16(std::uint8_t* b, std::uint16_t v) { b[0] = std::uint8_t(v); b[1] = std::uint8_t(v >> 8); }
void put_le32(std::uint8_t* b, std::int32_t s)
{
  const std::uint32_t v = static_cast<std::uint32_t>(s);
  b[0] = std::uint8_t(v); b[1] = std::uint8_t(v >> 8); b[2] = std::uint8_t(v >> 16); b[3] = std::uint8_t(v >> 24);
}
std::uint16_t get_le16(const std::uint8_t* b) { return std::uint16_t(b[0] | (std::uint16_t(b[1]) << 8)); }
std::int32_t get_le32(const std::uint8_t* b)
{
  return std::int32_t(std::uint32_t(b[0]) | (std::uint32_t(b[1]) << 8) |
                      (std::uint32_t(b[2]) << 16) | (std::uint32_t(b[3]) << 24));
}

}  // namespace

int main()
{
  using namespace aidin_hand2;
  using namespace aidin_hand2::canfd;

  Protocol protocol(HandSide::Left);
  const auto filters = protocol.rx_filters();  // [0]=position, [3]=status_word (rx_filters 순서)

  // 1) decode: position frame이 LE int32을 읽는가.
  {
    canfd_frame rx{};
    rx.can_id = filters[0].can_id;
    rx.len = 64;
    put_le32(rx.data, 0x01020304);
    StateFrames sf{};
    check(protocol.decode(rx, sf) == DecodeResult::Decoded, "decode position: result");
    // position 은 wire(드라이브 원점) → SDK 원점 변환이 붙는다.
    check(sf.actual_position[0] == 0x01020304 - canfd::kHomeOffsetCount,
          "decode position: LE int32 minus home offset");
  }

  // 2) decode: statusword frame의 두 영역 — status(0~31B), error(32~63B).
  {
    canfd_frame rx{};
    rx.can_id = filters[3].can_id;
    rx.len = 64;
    put_le16(rx.data + 0,  0x0027);
    put_le16(rx.data + 32, 0x2310);
    StateFrames sf{};
    protocol.decode(rx, sf);
    check(sf.status_word[0] == 0x0027, "decode statusword: status section");
    check(sf.error_code[0]  == 0x2310, "decode statusword: error section");
  }

  // 3) encode: target_position LE int32 + effort 두 영역 target(0~31B)/max(32~63B).
  {
    CommandFrames cf{};
    cf.target_position[0] = 0x0A0B0C0D;
    cf.target_effort[0]   = 0x1122;
    cf.max_effort[0]  = 0x3344;
    TxFrames tx{};
    protocol.init_tx(tx);
    protocol.encode(cf, tx);
    check(get_le32(tx.target_position.data) == 0x0A0B0C0D + canfd::kHomeOffsetCount,
          "encode position: LE int32 plus home offset");
    check(get_le16(tx.target_effort.data + 0)  == 0x1122, "encode effort: target section");
    check(get_le16(tx.target_effort.data + 32) == 0x3344, "encode effort: max section");
  }

  // 4) encode scatter: SDK actuator i 의 값이 wire 슬롯 i 로 순서대로 나간다 (identity).
  {
    CommandFrames cf{};
    for (std::size_t i = 0; i < kActuatorCount; ++i)
      cf.target_position[i] = static_cast<std::int32_t>(1000 + i);
    TxFrames tx{};
    protocol.init_tx(tx);
    protocol.encode(cf, tx);
    bool ok = true;
    for (std::size_t i = 0; i < kActuatorCount; ++i)
      ok &= (get_le32(tx.target_position.data + i * 4) ==
             static_cast<std::int32_t>(1000 + i) + canfd::kHomeOffsetCount);
    check(ok, "encode scatter: SDK i -> wire slot i");
  }

  // 5) round-trip: encode 한 wire 바이트를 그대로 decode 하면 SDK 순서가 보존된다
  //    (encode/decode 가 같은 순서로 scatter/gather 하므로 왕복은 identity).
  {
    CommandFrames cf{};
    for (std::size_t i = 0; i < kActuatorCount; ++i)
      cf.target_position[i] = static_cast<std::int32_t>(3000 + i);
    TxFrames tx{};
    protocol.init_tx(tx);
    protocol.encode(cf, tx);
    canfd_frame rx{};
    rx.can_id = filters[0].can_id;  // state position
    rx.len = 64;
    std::memcpy(rx.data, tx.target_position.data, 64);
    StateFrames sf{};
    protocol.decode(rx, sf);
    bool ok = true;
    for (std::size_t i = 0; i < kActuatorCount; ++i)
      ok &= (sf.actual_position[i] == cf.target_position[i]);
    check(ok, "round-trip encode->decode preserves SDK order (left)");
  }

  if (g_failures == 0) {
    std::printf("byte_order_test: OK\n");
    return 0;
  }
  std::printf("byte_order_test: %d failure(s)\n", g_failures);
  return 1;
}
