// Fake aidin_hand2::canfd::Transport — a LINK-TIME seam.
//
// This file defines the SAME class (same header transport.hpp) as the real
// SocketCAN transport, but with a hardware-free body. The liveness test links
// this .cpp instead of comms/canfd/transport.cpp, so HandComms/HandCore run
// their real control thread against this fake — no socket, no interface, no
// privileges. Production code and CMake are untouched; the substitution is
// purely which .cpp gets compiled into the test target.
//
// The class layout (private members fd_, last_send_errno_ and the inline
// is_open()/last_send_errno()) is fixed by
// transport.hpp — we may only supply the out-of-line method bodies. All mock
// state therefore lives in file-scope globals below.

#include "hand_core/comms/canfd/transport.hpp"

#include <linux/can.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "hand_core/comms/canfd/protocol.hpp"          // frame layout constants / IDs (reused)
#include <aidin_hand2/types/error_codes.hpp>
#include "hand_core/comms/state_machine/cia402.hpp"  // status_word::kQuickStopActive
#include "tests/liveness/mock_transport.hpp"

namespace aidin_hand2::mock
{

namespace
{
std::atomic<int> g_mode{static_cast<int>(Mode::FramesNoQuickStop)};

// Flip-flop so each read()/probe drain (which loops `while (recv(frame))`) gets
// exactly ONE frame, then false. Without the false the caller's drain loop would
// spin forever.
std::atomic<bool> g_emitted_this_drain{false};

// Alternate which hand-side status ID we synthesize each drain. The fake does not
// know the configured HandSide, so it emits the left ID (0x224) on even drains and
// the right ID (0x124) on odd drains; the wrong-side frame decodes as Unknown and
// is harmless, while the correct-side frame decodes as Decoded within a cycle or
// two. This keeps the mock side-agnostic.
std::atomic<unsigned> g_drain_counter{0};

// Set by send() when a control word frame (left 0x213, right 0x113) carries 0x0002
// in its first slot, which is the quick stop write() forces.
std::atomic<bool> g_quick_stop_sent{false};

// CiA402 statusword written into every actuator slot: Operation Enabled
// (bits 0,1,2) + kQuickStopActive (bit5, active-LOW == "alive"). error_code = 0.
// Because bit5 stays HIGH, quick_stop_attained() is never satisfied — exactly the
// condition under test.
constexpr std::uint16_t kAliveStatusWord = 0x0027;  // == kOperationEnabledBits | kQuickStopActive

// Build a synthetic statusword state frame for the given side ID. Layout matches
// Protocol::decode of the status_word frame: 16 uint16 status words (bytes 0..31)
// followed by 16 uint16 error codes (bytes 32..63).
void fill_status_frame(canfd_frame& frame, std::uint32_t can_id) noexcept
{
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id;
  frame.len = 64;  // >= kLenStatus(64); decode rejects only when shorter
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    const std::uint16_t sw = kAliveStatusWord;
    std::memcpy(frame.data + i * 2, &sw, sizeof(sw));  // status word slot i
    // error code slot i (bytes 32..) left 0 = ActuatorFault::None
  }
}
}  // namespace

void set_mode(Mode mode) noexcept
{
  g_mode.store(static_cast<int>(mode), std::memory_order_relaxed);
  g_emitted_this_drain.store(false, std::memory_order_relaxed);
  g_drain_counter.store(0, std::memory_order_relaxed);
  g_quick_stop_sent.store(false, std::memory_order_relaxed);
}

Mode get_mode() noexcept
{
  return static_cast<Mode>(g_mode.load(std::memory_order_relaxed));
}

bool quick_stop_sent() noexcept
{
  return g_quick_stop_sent.load(std::memory_order_relaxed);
}

}  // namespace aidin_hand2::mock

namespace aidin_hand2::canfd
{

Transport::~Transport()
{
  close();
}

// open always succeeds — no interface bring-up, no socket. is_open() (inline in
// the header) keys off fd_ >= 0, so we just stamp a non-negative fake fd.
Status Transport::open(const std::string& /*interface*/)
{
  if (fd_ >= 0) {
    return {error_code::kInternalState, "mock transport already open"};
  }
  fd_ = 1;  // any non-negative value ⇒ is_open() == true
  last_send_errno_ = 0;
  return {};  // ok
}

void Transport::close()
{
  fd_ = -1;  // idempotent — repeat close is safe
}

// RX seam. Silence mode: never a frame. FramesNoQuickStop mode: one synthetic
// status frame per drain (flip-flop), alternating side ID so the mock need not
// know the configured HandSide.
bool Transport::recv(canfd_frame& frame)
{
  if (fd_ < 0) {
    return false;
  }
  if (mock::get_mode() == mock::Mode::Silence) {
    return false;
  }
  // FramesNoQuickStop
  if (mock::g_emitted_this_drain.exchange(true, std::memory_order_relaxed)) {
    mock::g_emitted_this_drain.store(false, std::memory_order_relaxed);  // end this drain
    return false;
  }
  const unsigned n = mock::g_drain_counter.fetch_add(1, std::memory_order_relaxed);
  // Left status ID 0x224, Right status ID 0x124 — see protocol.cpp CAN ID table.
  const std::uint32_t status_id = (n & 1u) ? 0x124u : 0x224u;
  mock::fill_status_frame(frame, status_id);
  return true;
}

// TX seam — always accepted (link is "alive"), so write_ok stays true and the
// comm-liveness measurement never flags a transmit failure.
bool Transport::send(const canfd_frame& frame)
{
  if (fd_ < 0) {
    return false;
  }
  const bool control_word = frame.can_id == 0x213u || frame.can_id == 0x113u;
  if (control_word && frame.data[0] == 0x02 && frame.data[1] == 0x00) {
    mock::g_quick_stop_sent.store(true, std::memory_order_relaxed);
  }
  last_send_errno_ = 0;
  return true;
}

bool Transport::set_id_filters(const std::vector<can_filter>& /*filters*/)
{
  return fd_ >= 0;
}

Status Transport::open_failure(const std::string& interface, int /*err*/)
{
  // Unreachable in the mock (open never fails), but the symbol is declared in the
  // header, so provide a definition to keep the class complete.
  return {error_code::kInterfaceOpenFailed, "mock transport open failed: " + interface};
}

}  // namespace aidin_hand2::canfd
