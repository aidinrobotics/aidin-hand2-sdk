// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, terminal client for 19_external_server
//
// Sends one command per line over TCP and prints what the server reports back.
// The server holds the Hand; this program only decides what to ask for, which is
// the point of the externally managed lifecycle pattern.
//
// Run the server first, then: 19_external_client [host=127.0.0.1] [port=5555]

#include <arpa/inet.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <string>

namespace
{
constexpr double kGripStep = 0.05;   // rad per key press
constexpr double kGripMax  = 0.60;   // beyond the reachable range the server clamps anyway

// SIGINT only asks the loop below to stop. It matters more here than in most of the examples:
// the terminal is in raw mode, and dying inside the loop would leave it that way
std::atomic<bool> g_shutdown{false};

termios g_saved_termios{};
bool    g_termios_saved = false;

// Raw mode so a single key press arrives without waiting for Enter
bool enter_raw_mode()
{
  if (tcgetattr(STDIN_FILENO, &g_saved_termios) != 0) return false;
  g_termios_saved = true;

  termios raw = g_saved_termios;
  raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
  raw.c_cc[VMIN]  = 0;
  raw.c_cc[VTIME] = 0;
  return tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
}

void restore_terminal()
{
  if (g_termios_saved) tcsetattr(STDIN_FILENO, TCSANOW, &g_saved_termios);
}

int connect_to(const char* host, int port)
{
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) return -1;

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port   = htons(static_cast<uint16_t>(port));
  if (::inet_pton(AF_INET, host, &address.sin_addr) != 1) {
    ::close(fd);
    return -1;
  }
  if (::connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
    ::close(fd);
    return -1;
  }
  return fd;
}

void send_line(int fd, const std::string& text)
{
  const std::string payload = text + "\n";
  ::send(fd, payload.data(), payload.size(), MSG_NOSIGNAL);
}

// One status line, redrawn in place
void redraw(const std::string& state, double grip, const std::string& note)
{
  std::printf("\r\033[K[%s]  grip=%.2f rad  %s", state.c_str(), grip, note.c_str());
  std::fflush(stdout);
}

// Maps a key to the command the server understands. Returns an empty string when the
// key does nothing, so the caller can skip the send
std::string command_for(char key, double& grip)
{
  switch (key) {
    case 'c': return "connect";
    case 'd': return "disconnect";
    case 'r': return "run";
    case 's': return "stop";
    case 'h': return "home";
    case 'n': return "reconnect";
    case '[':
      grip = std::max(0.0, grip - kGripStep);
      return "grip " + std::to_string(grip);
    case ']':
      grip = std::min(kGripMax, grip + kGripStep);
      return "grip " + std::to_string(grip);
    default: return {};
  }
}
}  // namespace

int main(int argc, char** argv)
{
  std::signal(SIGINT, [](int) { g_shutdown.store(true); });

  const char* host = argc > 1 ? argv[1] : "127.0.0.1";
  const int   port = argc > 2 ? std::atoi(argv[2]) : 5555;

  const int server = connect_to(host, port);
  if (server < 0) {
    std::fprintf(stderr, "cannot reach %s:%d — start 19_external_server first\n", host, port);
    return 1;
  }
  if (!enter_raw_mode()) {
    std::fprintf(stderr, "cannot switch the terminal to raw mode\n");
    ::close(server);
    return 1;
  }

  std::printf("c connect  d disconnect  r run  s stop  h home  n reconnect  [ / ] grip  q quit\n");

  std::string state = "unknown";
  std::string note;
  std::string rx;
  double grip = 0.0;
  bool   quit = false;
  redraw(state, grip, note);

  while (!quit && !g_shutdown.load()) {
    pollfd watched[2] = {{STDIN_FILENO, POLLIN, 0}, {server, POLLIN, 0}};
    // poll() returns EINTR when the signal lands mid-wait, so a negative return is checked
    // against the flag rather than treated as a failure on its own
    if (::poll(watched, 2, 100) < 0 && !g_shutdown.load()) break;

    // 1) Key press -> command
    if (watched[0].revents & POLLIN) {
      char key = 0;
      if (::read(STDIN_FILENO, &key, 1) == 1) {
        if (key == 'q') {
          send_line(server, "quit");
          quit = true;
        } else if (const std::string command = command_for(key, grip); !command.empty()) {
          send_line(server, command);
          note = "sent " + command;
          redraw(state, grip, note);
        }
      }
    }

    // 2) Server line -> state or result
    if (watched[1].revents & POLLIN) {
      char buffer[256];
      const ssize_t received = ::recv(server, buffer, sizeof(buffer), 0);
      if (received <= 0) break;   // the server is gone
      rx.append(buffer, static_cast<std::size_t>(received));

      for (std::size_t end = rx.find('\n'); end != std::string::npos; end = rx.find('\n')) {
        std::string line = rx.substr(0, end);
        rx.erase(0, end + 1);
        if (!line.empty() && line.back() == '\r') line.pop_back();

        if (line.rfind("state ", 0) == 0) {
          state = line.substr(6);
        } else {
          note = line;   // ok, or err <ErrorCode>: <message>
        }
        redraw(state, grip, note);
      }
    }
  }

  // Ctrl-C took the same exit as 'q', so tell the server too: it holds the Hand, and leaving
  // it to notice the closed socket would leave the hand under whatever command it last got
  if (g_shutdown.load()) send_line(server, "quit");

  restore_terminal();
  std::printf("\n");
  ::close(server);
  return 0;
}
