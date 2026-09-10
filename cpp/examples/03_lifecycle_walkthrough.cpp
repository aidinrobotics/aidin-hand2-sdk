// Copyright (c) AIDIN ROBOTICS Inc.
// SPDX-License-Identifier: Apache-2.0

// AIDIN Hand Gen2 SDK, calling the lifecycle by hand to see which calls the state accepts
//
// Pick a transition by number and watch what happens. Every call is offered from every state,
// so the refusals are as much the point as the transitions: run() before connect(), reconnect()
// when nothing has faulted, set_command() before homing. The reason the SDK gives is printed
// as it comes back.
//
// A single keypress is the whole input — no Enter — and the key guide and state stay pinned to
// the bottom of the terminal, redrawn about ten times a second, so the state is live even while
// nothing is typed. Pull the CAN cable out and it turns to Faulted on its own about 100 ms after
// RX stops, and from there reconnect() is the only call that gets you out.
//
// Run as: 03_lifecycle_walkthrough [right|left] [interface=can0]
// Needs a real hand. run() and home() enable the drives — keep the surroundings clear, and
// remember that home() drives every finger into its hard stop. Keys 8 and 9 move the fingers.

#include <atomic>
#include <cstddef>
#include <cstdio>
#include <csignal>
#include <mutex>
#include <string>
#include <vector>

#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <aidin_hand2/aidin_hand2.hpp>

using namespace aidin_hand2;

namespace
{
// SIGINT only asks the loop below to stop. The HandManager destructor is what confirms the
// quick stop and closes the link, so the point is to leave the loop and reach that destructor.
std::atomic<bool> g_shutdown{false};

// ---------------------------------- Terminal ---------------------------------

// ICANON off is what makes a single keypress arrive without Enter, and ECHO off keeps the key
// itself from being printed into the middle of the status block. ISIG stays on, so Ctrl-C still
// reaches the handler above. The destructor puts the terminal back however the loop ends —
// including on the way out of an exception — which matters because a shell left in this mode is
// unusable. Piped or redirected stdin is left alone: there is no terminal to put into raw mode.
class RawKeys
{
public:
  RawKeys() : restore_(::isatty(STDIN_FILENO) == 1)
  {
    if (!restore_) return;
    ::tcgetattr(STDIN_FILENO, &saved_);
    termios raw = saved_;
    raw.c_lflag &= ~(static_cast<tcflag_t>(ICANON) | static_cast<tcflag_t>(ECHO));
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;
    ::tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    std::printf("\033[?25l");  // hide the cursor: there is no prompt left for it to sit at
  }

  ~RawKeys()
  {
    if (!restore_) return;
    std::printf("\033[?25h");
    std::fflush(stdout);
    ::tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
  }

  RawKeys(const RawKeys&)            = delete;
  RawKeys& operator=(const RawKeys&) = delete;

private:
  bool    restore_;
  termios saved_{};
};

// ----------------------------------- Logging ---------------------------------

// The SDK's console sink writes to stderr from whichever thread logged, and the control thread
// logs on its own schedule — a fault is detected and reported without anyone asking. A line
// arriving between the block being drawn and being rewound to would put the cursor arithmetic
// below out by however many lines it took, so the console sink is turned off and every line is
// taken through set_log_callback() instead.
//
// Where a line then goes depends on whether the block is up. While it is, the line is queued
// here and the main loop prints it once it has taken the block down. While a call is running the
// block is already down and the cursor is in ordinary scrolling territory, so the line is printed
// straight away instead — which is the only way to watch a long call talk. home() takes seconds
// and reports each finger as it reaches its stop, and queueing that would mean reading the whole
// story after it was over.
std::mutex               g_log_mutex;
std::vector<std::string> g_log_lines;
std::atomic<bool>        g_log_direct{false};

// The cap applies to the queue only: a hand that faults every cycle can log faster than the loop
// below drains, and an example is not worth an unbounded queue.
constexpr std::size_t kMaxQueuedLogs = 200;

// Called on whichever thread logged, the control thread included, so the work here is one string
// and one short lock. The direct branch does reach printf, which is what the SDK's own console
// sink does from those same threads, and stdout locks per call so a line cannot come out shredded.
void queue_log_line(LogLevel level, const std::string& message)
{
  const char* tag = "log";
  switch (level) {
    case LogLevel::Trace: tag = "trace"; break;
    case LogLevel::Debug: tag = "debug"; break;
    case LogLevel::Info: tag = "info"; break;
    case LogLevel::Warn: tag = "warn"; break;
    case LogLevel::Error: tag = "error"; break;
    case LogLevel::Critical: tag = "critical"; break;
    case LogLevel::Off: return;
  }
  // Warnings and errors get a colour, since those are the ones worth looking up from the keys.
  const char* color = (level >= LogLevel::Error) ? "\033[31m" : (level == LogLevel::Warn ? "\033[33m" : "\033[90m");

  const std::string line = std::string("  ") + color + "[" + tag + "] " + message + "\033[0m";

  std::lock_guard<std::mutex> lock(g_log_mutex);
  if (g_log_direct.load()) {
    std::printf("%s\n", line.c_str());
    std::fflush(stdout);
    return;
  }
  if (g_log_lines.size() >= kMaxQueuedLogs) return;
  g_log_lines.push_back(line);
}

// Direct printing is switched on for exactly as long as a call is running. Anything queued
// before it goes out first, so turning it on cannot jump a new line ahead of an older one.
class DirectLogs
{
public:
  DirectLogs() { g_log_direct.store(true); }
  ~DirectLogs() { g_log_direct.store(false); }

  DirectLogs(const DirectLogs&)            = delete;
  DirectLogs& operator=(const DirectLogs&) = delete;
};

// --------------------------------- Status block -------------------------------

// The block is a fixed number of lines so it can be rewound to and rewritten in place. Change
// what draw_status() prints and this has to change with it, or the redraw lands in the wrong
// place.
constexpr int kStatusLines = 7;

// Dim rules rather than a closed box: nothing here has to line up with a right-hand edge, so
// the wording below can be changed without the padding having to be counted out again.
constexpr const char* kRule = "───────────────────────────────────────────────────";

constexpr const char* kDim   = "\033[90m";
constexpr const char* kBold  = "\033[1m";
constexpr const char* kReset = "\033[0m";

// A colour per lifecycle state, so which one is up reads before the word does: green while
// control is running, red the moment something faults.
const char* color_of(HandLifecycle lifecycle)
{
  switch (lifecycle) {
    case HandLifecycle::Disconnected: return "\033[90m";  // grey — nothing open yet
    case HandLifecycle::Connected: return "\033[36m";     // cyan — link up, drives off
    case HandLifecycle::Running: return "\033[1;32m";     // green — control loop live
    case HandLifecycle::Stopped: return "\033[33m";       // yellow — quick stop confirmed
    case HandLifecycle::Faulted: return "\033[1;31m";     // red — reconnect() only
  }
  return kReset;
}

const char* color_of(HomingState state)
{
  switch (state) {
    case HomingState::NotRun: return "\033[90m";
    case HomingState::InProgress: return "\033[33m";
    case HomingState::Succeeded: return "\033[1;32m";
    case HomingState::Failed: return "\033[1;31m";
  }
  return kReset;
}

// Rewind over the block and clear from there down, so whatever gets printed next starts on a
// clean line instead of landing on top of half a stale status line.
void erase_status(bool& drawn)
{
  if (!drawn) return;
  std::printf("\033[%dA\033[J", kStatusLines);
  drawn = false;
}

// Redraw the block in place. \033[<n>A walks the cursor back to the first of its lines and
// \033[K clears each one before it is rewritten, so a shorter line does not leave a tail behind.
void draw_status(const Diagnostics& diagnostics, bool& drawn)
{
  if (drawn) std::printf("\033[%dA", kStatusLines);

  // deadline_misses and nan_command_count are only worth the eye's attention once they move off
  // zero, so they stay grey until they do.
  const char* miss_color     = diagnostics.deadline_misses > 0 ? "\033[33m" : kDim;
  const char* rejected_color = diagnostics.nan_command_count > 0 ? "\033[33m" : kDim;

  std::printf("\033[K %s── keys %s%s\n", kDim, kRule, kReset);
  std::printf("\033[K   %s1%s connect   %s2%s run   %s3%s stop   %s4%s home   %s5%s reconnect   %s6%s disconnect\n",
              kBold, kReset, kBold, kReset, kBold, kReset, kBold, kReset, kBold, kReset, kBold, kReset);
  std::printf("\033[K   %sset_command%s  %s7%s Idle   %s8%s joints curl   %s9%s joints zero      %s0%s quit\n",
              kDim, kReset, kBold, kReset, kBold, kReset, kBold, kReset, kBold, kReset);
  std::printf("\033[K %s── state %s%s\n", kDim, kRule, kReset);
  std::printf("\033[K   %slifecycle%s %s● %-12s%s   %shoming%s %s● %-10s%s\n",
              kDim, kReset, color_of(diagnostics.lifecycle), to_string(diagnostics.lifecycle), kReset,
              kDim, kReset, color_of(diagnostics.homing_state), to_string(diagnostics.homing_state), kReset);
  std::printf("\033[K   %scycles%s %-10llu %smisses%s %s%-6llu%s %speriod%s %5.2f ms  %scompute%s %5.2f ms  "
              "%srejected%s %s%llu%s\n",
              kDim, kReset, static_cast<unsigned long long>(diagnostics.control_cycles),
              kDim, kReset, miss_color, static_cast<unsigned long long>(diagnostics.deadline_misses), kReset,
              kDim, kReset, diagnostics.last_period_ms,
              kDim, kReset, diagnostics.last_compute_ms,
              kDim, kReset, rejected_color,
              static_cast<unsigned long long>(diagnostics.nan_command_count), kReset);
  std::printf("\033[K %s─────────%s%s\n", kDim, kRule, kReset);
  std::fflush(stdout);

  drawn = true;
}

// Take the block down, print whatever the control thread logged since the last pass, and leave
// the block to be drawn again at the top of the loop. Called before anything else is printed,
// so a log line and a refusal always come out in the order they happened.
void flush_queued_logs(bool& drawn)
{
  std::vector<std::string> lines;
  {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    if (g_log_lines.empty()) return;
    lines.swap(g_log_lines);
  }
  erase_status(drawn);
  for (const std::string& line : lines) std::printf("%s\n", line.c_str());
}
}  // namespace

int main(int argc, char** argv)
{
  std::signal(SIGINT, [](int) { g_shutdown.store(true); });

  const HandSide side = (argc > 1 && std::string(argv[1]) == "left") ? HandSide::Left : HandSide::Right;

  try {
    // Both halves matter: the console sink has to go, or it writes to stderr from the control
    // thread straight through the block, and the callback has to be set, or the fault the
    // example is built around is never explained. The callback sink is fed from Debug up
    // whatever the console level is, so Debug and Trace are dropped here rather than there.
    set_log_to_console(false);
    set_log_callback([](LogLevel level, const std::string& message) {
      if (level >= LogLevel::Info) queue_log_line(level, message);
    });

    HandConfig config{argc > 2 ? argv[2] : "can0", side};
    config.control_rate   = 500;    // Hz
    config.auto_home      = false;  // home() is on the menu, so run() should not do it for you
    config.auto_reconnect = false;  // recovery stays manual here, or Faulted would clear itself

    HandManager manager;
    Hand hand = manager.create(config);

    // Key 8. The 16 entries are the active joints, in the order the whole SDK uses: the thumb's
    // four first, then three per long finger, whose first entry is its abduction joint. So this
    // is every abduction at 0 and every long-finger flexion at 1.0 rad — the fingers curl in
    // without spreading. Key 9 is the same command with every entry at 0, which is the open
    // hand, and going back and forth between the two is the cheapest way to see that control is
    // really running.
    JointPositionCommand curl;
    curl.target = {
        0.0, 0.5, 0.0, 0.5,  // thumb   j0 j1 j2 j3
        0.0, 1.0, 1.0,       // index   j1 j2 j3
        0.0, 1.0, 1.0,       // middle
        0.0, 1.0, 1.0,       // ring
        0.0, 1.0, 1.0};      // baby

    JointPositionCommand zero;
    zero.target.fill(0.0);

    std::printf("\nEvery number is accepted whatever the state is. What the SDK does with it —\n");
    std::printf("the transition or the refusal — is printed above the block below, which keeps\n");
    std::printf("the keys and the live state in view. One keypress is enough; no Enter.\n");
    std::printf("0 leaves, and so does Ctrl-C.\n\n");

    const RawKeys raw_keys;

    // stdin is polled instead of read straight through, so the block below can be redrawn while
    // nothing is being typed. That is what makes a fault show up without a keypress.
    struct pollfd stdin_poll = {STDIN_FILENO, POLLIN, 0};

    bool status_drawn = false;

    while (!g_shutdown.load()) {
      // Anything the control thread logged since the last pass comes out first, so a fault is
      // explained above the block that is about to show Faulted.
      flush_queued_logs(status_drawn);

      // Read the state fresh every pass: it moves on its own when the control loop faults.
      draw_status(hand.get_diagnostics(), status_drawn);

      if (poll(&stdin_poll, 1, 100) <= 0) continue;  // nothing typed — redraw and wait again

      // One read takes whatever the key produced. Arrows and function keys arrive as a two- or
      // three-byte escape sequence, and swallowing the whole burst here keeps the tail bytes
      // from being mistaken for menu numbers on the next passes.
      char       buffer[8] = {0};
      const auto count     = ::read(STDIN_FILENO, buffer, sizeof(buffer));
      if (count <= 0) break;  // Ctrl-D, or stdin closed
      const char choice = buffer[0];

      if (choice == '\033') continue;                                   // arrow or function key
      if (choice == '\n' || choice == '\r' || choice == ' ') continue;  // Enter is not needed
      if (choice == '0' || choice == 'q') break;

      erase_status(status_drawn);       // hand the screen over to this round's output
      flush_queued_logs(status_drawn);  // and clear the queue before direct printing starts

      // The outcome is held back rather than printed where it is decided. The call below logs
      // as it goes and those lines print themselves, so the one line saying how the call ended
      // belongs after them — a home() that reports five fingers and then succeeded reads in
      // that order, rather than announcing the result before the story.
      std::string outcome;

      const DirectLogs direct_logs;  // for the length of the call, log lines print themselves

      try {
        switch (choice) {
          case '1':
            // Disconnected only. Called in Connected it is a no-op that logs what it skipped;
            // in Running or Stopped it is refused, because a rebuild has to go through
            // disconnect() first.
            hand.connect();
            outcome = "connect() returned";
            break;

          case '2':
            // Connected or Stopped. It blocks until the drives report Operation Enabled, so a
            // return means the hardware got there. In Faulted it is refused.
            hand.run();
            outcome = "run() returned — the drives confirmed the enable";
            break;

          case '3':
            // Running only. It blocks until the drives reach quick stop, and resets the stored
            // command to Idle. In Connected there is no control to stop, so it is refused.
            hand.stop();
            outcome = "stop() returned — the drives confirmed the quick stop";
            break;

          case '4':
            // Connected, Running or Stopped. It enables the drives if they are not already, so
            // there is no need to call run() first. Every finger travels to its hard stop.
            hand.home();
            outcome = "home() returned — homing_state is now Succeeded";
            break;

          case '5':
            // Faulted only. Outside Faulted it is refused on purpose: a fault should not be
            // cleared without its cause being seen. It resets homing_state to NotRun and does
            // not resume control, so run() is still needed afterwards.
            hand.reconnect();
            outcome = "reconnect() returned — control has to be asked for again";
            break;

          case '6':
            // Anything but Disconnected. It confirms the quick stop before closing, and when it
            // cannot confirm it throws and leaves the socket open rather than dropping a hand
            // that may still hold torque.
            hand.disconnect();
            outcome = "disconnect() returned";
            break;

          case '7':
            // Running only, and every command other than Idle also needs homing_state to be
            // Succeeded. Idle sends an effort of 0 while the drives stay on, so a joint moved
            // by hand still meets some resistance.
            hand.set_command(Idle{});
            outcome = "set_command(Idle) returned";
            break;

          case '8':
          case '9': {
            // Running and homed, which is the pair of conditions key 7 does not have to meet:
            // try either of these straight after run() and the refusal names the homing state.
            // The return says only that the command was stored — the control loop sends it from
            // the next cycle on, and the fingers are still on their way when this prints.
            const bool                  is_curl = (choice == '8');
            const JointPositionCommand& pose    = is_curl ? curl : zero;
            hand.set_command(pose);
            outcome = std::string("set_command(JointPositionCommand) returned — ") +
                      (is_curl ? "curling in" : "opening back out");
            break;
          }

          default:
            outcome = std::string("no such command: '") + choice + "'";
            break;
        }
      } catch (const Exception& error) {
        // This is the interesting half. The code says what kind of failure it is and the
        // message says why, and the SDK has already logged both at error level — which is why
        // the same text comes out just above this line, tagged [error], from the callback.
        outcome = std::string("refused — ") + to_string(error.code()) + ": " + error.what();
      }

      flush_queued_logs(status_drawn);  // whatever the SDK logged while the call was running
      std::printf("  %s\n", outcome.c_str());
      std::printf("\n");  // a blank line between this round and the block that returns below
    }

    flush_queued_logs(status_drawn);
    erase_status(status_drawn);
    std::printf("leaving — the manager destructor stops the hand and closes the link\n");

    // The block is down for good, so the console sink can have the terminal back. It has to
    // get it back before this scope ends: the manager is destroyed on the way out and logs
    // while it stops the hand and closes the link, and with the queue no longer being drained
    // none of that would be seen.
    set_log_callback(nullptr);
    set_log_to_console(true);
  } catch (const Exception& error) {
    // Only create() throws out here. Everything inside the loop is caught above so that a
    // refusal does not end the session.
    std::printf("\nfailed to start: %s: %s\n", to_string(error.code()), error.what());
    return 1;
  }

  return 0;
}
