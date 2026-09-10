// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, a server that lets something outside drive the lifecycle
//
// create -> loop { publish a state change -> apply one incoming command -> answer it }
// Nothing here decides when to run or stop: the client does, so the loop never calls run() on
// its own. A command that fails comes back as a reason, not a retry, and the state is published
// on its own schedule because the hand can reach Faulted while no command is in flight
//
// Run as: 19_external_server [right|left] [interface=can0] [port=5555]
// Needs a real hand. Drive it with 19_external_client, or by hand:
//   nc localhost 5555
//   connect / run / home / grip 0.4 / stop / disconnect / reconnect / quit
// Answers are "ok" or "err <ErrorCode>: <message>", and a lifecycle change arrives as
// "state <lifecycle> <homing_state>"

#include <netinet/in.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include <aidin_hand2/aidin_hand2.hpp>

using namespace aidin_hand2;

namespace
{
// grip drives the flexion joints only; thumb j2 and every other finger's j1 are abduction
constexpr std::array<bool, kActiveJointCount> kIsAbduction = {
    false, false, true,  false,   // thumb   j0 j1 j2 j3
    true,  false, false,          // index   j1 j2 j3
    true,  false, false,          // middle
    true,  false, false,          // ring
    true,  false, false};         // baby

std::atomic<bool> g_shutdown{false};
std::string       g_rx;

int open_listener(int port)
{
  const int fd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
  if (fd < 0) return -1;

  int on = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

  sockaddr_in address{};
  address.sin_family      = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port        = htons(static_cast<uint16_t>(port));

  if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0 ||
      ::listen(fd, 1) < 0) {
    ::close(fd);
    return -1;
  }
  return fd;
}

// Pulls one newline-terminated line out of the receive buffer. False while none is complete
bool read_line(int fd, std::string& line)
{
  char buffer[256];
  const ssize_t received = ::recv(fd, buffer, sizeof(buffer), MSG_DONTWAIT);
  if (received > 0) g_rx.append(buffer, static_cast<std::size_t>(received));

  const std::size_t end = g_rx.find('\n');
  if (end == std::string::npos) return false;

  line = g_rx.substr(0, end);
  g_rx.erase(0, end + 1);
  if (!line.empty() && line.back() == '\r') line.pop_back();
  return true;
}

void send_line(int fd, const std::string& text)
{
  const std::string payload = text + "\n";
  ::send(fd, payload.data(), payload.size(), MSG_NOSIGNAL);
}

// Turns one line into one SDK call. A failure leaves as an Exception, and the caller
// hands the reason back to the client
void apply_command(Hand& hand, const std::string& line)
{
  const std::size_t space = line.find(' ');
  const std::string verb  = line.substr(0, space);

  if (verb == "connect")    { hand.connect();            return; }
  if (verb == "disconnect") { hand.disconnect();         return; }
  if (verb == "run")        { hand.run();                return; }
  if (verb == "stop")       { hand.stop();               return; }
  if (verb == "home")       { hand.home();               return; }
  if (verb == "reconnect")  { hand.reconnect();          return; }
  if (verb == "quit")       { g_shutdown.store(true);    return; }

  if (verb == "grip" && space != std::string::npos) {
    const double angle = std::atof(line.c_str() + space + 1);
    JointPositionCommand command;
    for (std::size_t joint = 0; joint < kActiveJointCount; ++joint) {
      command.target[joint] = kIsAbduction[joint] ? 0.0 : angle;
    }
    hand.set_command(command);
    return;
  }

  throw Exception(ErrorCode::InvalidArgument, "unknown command: " + line);
}

void wait_next_period()
{
  static auto next = std::chrono::steady_clock::now();
  next += std::chrono::milliseconds(20);
  std::this_thread::sleep_until(next);
}
}  // namespace

int main(int argc, char** argv)
{
  std::signal(SIGINT, [](int) { g_shutdown.store(true); });

  const HandSide side = (argc > 1 && std::string(argv[1]) == "left") ? HandSide::Left : HandSide::Right;
  const int      port = argc > 3 ? std::atoi(argv[3]) : 5555;

  const int listener = open_listener(port);
  if (listener < 0) {
    std::fprintf(stderr, "cannot listen on port %d\n", port);
    return 1;
  }
  int client = -1;

  try {
    HandConfig config{argc > 2 ? argv[2] : "can0", side};
    config.control_rate   = 500;     // Hz
    config.auto_home      = false;   // the client decides when to home
    config.auto_reconnect = false;   // and when to recover

    // The destructor confirms the quick stop and closes the link on the way out
    HandManager manager;
    Hand hand = manager.create(config);

    auto last_lifecycle = HandLifecycle::Disconnected;
    auto last_homing    = HomingState::NotRun;

    std::printf("listening on 127.0.0.1:%d\n", port);

    bool greet_client = false;

    while (!g_shutdown.load()) {
      if (client < 0) {
        client = ::accept(listener, nullptr, nullptr);
        greet_client = client >= 0;   // a fresh client has not seen the state yet
      }

      // Published apart from the commands, because the loop can latch Faulted while the
      // client is idle. get_diagnostics() keeps reading in Faulted and never throws
      const Diagnostics diag = hand.get_diagnostics();
      if (client >= 0 && (greet_client || diag.lifecycle != last_lifecycle ||
                          diag.homing_state != last_homing)) {
        send_line(client, std::string("state ") + to_string(diag.lifecycle) + " " +
                              to_string(diag.homing_state));
        last_lifecycle = diag.lifecycle;
        last_homing    = diag.homing_state;
        greet_client   = false;
      }

      std::string line;
      if (client >= 0 && read_line(client, line)) {
        try {
          apply_command(hand, line);
          send_line(client, "ok");
        } catch (const Exception& error) {
          // No retry here. Whether to ask again is the client's call
          send_line(client, std::string("err ") + to_string(error.code()) + ": " + error.what());
        }
      }
      wait_next_period();
    }
  } catch (const Exception&) {
    if (client >= 0) ::close(client);
    ::close(listener);
    return 1;   // the SDK logged the cause before throwing
  }

  if (client >= 0) ::close(client);
  ::close(listener);
  return 0;
}
