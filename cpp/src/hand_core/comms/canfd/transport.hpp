#pragma once

#include <linux/can.h>
#include <linux/can/error.h>

#include <cstdint>
#include <string>
#include <vector>

#include "types/status.hpp"

namespace aidin_hand2::canfd
{

// Owns one SocketCAN raw socket and moves frames, one instance per thread
class Transport {
 public:
  // --------------------------------- Socket ---------------------------------

  Transport() = default;
  ~Transport();

  Transport(const Transport&) = delete;
  Transport& operator=(const Transport&) = delete;

  // Binds to an interface that is already up
  [[nodiscard]] Status open(const std::string& interface);

  // Safe to call repeatedly
  void close();

  // True while the socket is bound
  [[nodiscard]] bool is_open() const noexcept { return fd_ >= 0; }

  // Interface index at bind time, fixed after open
  // A re-created interface gets a different index, which is how a needed re-open is spotted
  [[nodiscard]] int bound_interface_index() const noexcept { return bound_interface_index_; }

  // --------------------------------- Frames ---------------------------------

  // Non-blocking, one frame, false when nothing arrived
  [[nodiscard]] bool recv(canfd_frame& frame);

  // Non-blocking, one frame, false when the TX queue is full or the send failed
  [[nodiscard]] bool send(const canfd_frame& frame);

  // errno of the last failed send, 0 when none failed
  // Kept so a non-RT thread can tell ENOBUFS from ENETDOWN
  [[nodiscard]] int last_send_errno() const noexcept { return last_send_errno_; }

  // -------------------------------- Filters ---------------------------------

  // Installs the kernel CAN ID filter, an empty vector removing it
  [[nodiscard]] bool set_id_filters(const std::vector<can_filter>& filters);

  // OR of CAN_ERR_* values, 0 blocking every error frame
  [[nodiscard]] bool set_error_mask(can_err_mask_t mask);

 private:
  // Turns an open() errno into a Status carrying the cause and what to do
  static Status open_failure(const std::string& interface, int err);

  // CAN raw socket, -1 when closed
  int fd_{-1};

  // Only written on failure, so a successful send costs nothing
  int last_send_errno_{0};

  // Fixed after open
  int bound_interface_index_{0};
};

}  // namespace aidin_hand2::canfd