#pragma once

// Link-time seam control for the fake canfd::Transport (mock_transport.cpp).
//
// The liveness test links mock_transport.cpp INSTEAD of the real
// comms/canfd/transport.cpp, so the very same class aidin_hand2::canfd::Transport
// is provided by a hardware-free implementation. This header lets the test pick
// which failure the fake simulates BEFORE it drives connect().
//
// This header is test-only — production code never includes it and never knows
// the seam exists (no virtuals, RT hot path unchanged).

namespace aidin_hand2::mock
{

enum class Mode {
  // (i) state frames DO arrive (so connect()/run() succeed and observation is
  //     live), but statusword bit5 (kQuickStopActive, active-LOW) stays HIGH
  //     forever — the drives never report quick-stop-attained. This forces the
  //     stop-confirmation window to expire so stop()/close() must unblock on
  //     their OWN bounded timeout rather than on a confirmation that never comes.
  FramesNoQuickStop,
  // (ii) plain silence — open() succeeds but recv() never yields a frame, so
  //      connect() must give up on its own first-frame timeout (bounded).
  Silence,
};

// Set the fake's behavior. Call before connect(). Process-global; scenarios in
// the liveness test run sequentially (one live transport at a time).
void set_mode(Mode mode) noexcept;
Mode get_mode() noexcept;

}  // namespace aidin_hand2::mock
