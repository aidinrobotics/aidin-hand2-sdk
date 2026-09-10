#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <spdlog/common.h>
#include <spdlog/logger.h>
#include <spdlog/pattern_formatter.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>

// Also declares stderr_color_sink_mt
#include <spdlog/sinks/stdout_color_sinks.h>

#include <aidin_hand2/logging/logging.hpp>
#include <aidin_hand2/types/description.hpp>

// spdlog wiring for logging.cpp, kept out of the API file

namespace aidin_hand2::log_sinks
{

// %E.%F is epoch seconds.nanoseconds, %* and %~ are the custom flags below
inline constexpr const char* kConsolePattern = "[%E.%F] [aidin_hand2] %^%*%$ %~";
inline constexpr const char* kFilePattern    = "[%E.%F] [aidin_hand2] [%l] %v";

// A fault storm writes about 200 KiB/s, so 300 MiB keeps roughly 25 minutes of one. 50 MiB per
// file stays comfortable for grep and less
inline constexpr std::size_t kRotatingFileMaxBytes = 50 * 1024 * 1024;
inline constexpr std::size_t kRotatingFileCount    = 5;

inline spdlog::level::level_enum to_spdlog_level(LogLevel level)
{
  switch (level) {
    case LogLevel::Trace: return spdlog::level::trace;
    case LogLevel::Debug: return spdlog::level::debug;
    case LogLevel::Info: return spdlog::level::info;
    case LogLevel::Warn: return spdlog::level::warn;
    case LogLevel::Error: return spdlog::level::err;
    case LogLevel::Critical: return spdlog::level::critical;
    case LogLevel::Off: return spdlog::level::off;
  }
  return spdlog::level::info;
}

inline LogLevel from_spdlog_level(spdlog::level::level_enum level)
{
  switch (level) {
    case spdlog::level::trace: return LogLevel::Trace;
    case spdlog::level::debug: return LogLevel::Debug;
    case spdlog::level::info: return LogLevel::Info;
    case spdlog::level::warn: return LogLevel::Warn;
    case spdlog::level::err: return LogLevel::Error;
    case spdlog::level::critical: return LogLevel::Critical;
    default: return LogLevel::Off;
  }
}

inline const char* hand_prefix(HandSide side)
{
  return side == HandSide::Left ? "[left] " : "[right] ";
}

// Renders "[level]" padded on the right so messages start at the same column
class BracketedLevelFlag : public spdlog::custom_flag_formatter
{
 public:
  void format(const spdlog::details::log_msg& message, const std::tm&,
              spdlog::memory_buf_t& destination) override
  {
    // Width of "[critical]"
    constexpr std::size_t kFieldWidth = 10;
    const auto level_name = spdlog::level::to_string_view(message.level);
    destination.push_back('[');
    destination.append(level_name.data(), level_name.data() + level_name.size());
    destination.push_back(']');
    for (std::size_t written = level_name.size() + 2; written < kFieldWidth; ++written) {
      destination.push_back(' ');
    }
  }
  std::unique_ptr<custom_flag_formatter> clone() const override
  {
    return spdlog::details::make_unique<BracketedLevelFlag>();
  }
};

// Indents the lines after an explicit newline so they align with the message column
class HangingIndentMessageFlag : public spdlog::custom_flag_formatter
{
 public:
  void format(const spdlog::details::log_msg& message, const std::tm&,
              spdlog::memory_buf_t& destination) override
  {
    // 22 timestamp + 1 + 13 logger name + 1 + 10 level field + 1
    constexpr std::size_t kPrefixWidth = 48;
    for (const char character : message.payload) {
      destination.push_back(character);
      if (character == '\n') {
        for (std::size_t i = 0; i < kPrefixWidth; ++i) destination.push_back(' ');
      }
    }
  }
  std::unique_ptr<custom_flag_formatter> clone() const override
  {
    return spdlog::details::make_unique<HangingIndentMessageFlag>();
  }
};

// Passes the level and the unformatted message to the callback, skipping the formatter
class CallbackSink : public spdlog::sinks::base_sink<std::mutex>
{
 public:
  explicit CallbackSink(LogCallback callback) : callback_(std::move(callback)) {}

 protected:
  void sink_it_(const spdlog::details::log_msg& message) override
  {
    callback_(from_spdlog_level(message.level),
              std::string(message.payload.data(), message.payload.size()));
  }
  void flush_() override {}

 private:
  LogCallback callback_;
};

struct LoggerState {
  std::mutex mutex;
  std::shared_ptr<spdlog::sinks::stderr_color_sink_mt> console_sink;
  std::shared_ptr<spdlog::sinks::rotating_file_sink_mt> file_sink;
  std::shared_ptr<CallbackSink> callback_sink;
  std::shared_ptr<spdlog::logger> logger;
  LogLevel console_level{LogLevel::Info};
  bool console_enabled{true};

  LoggerState()
  {
    console_sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);
    auto console_formatter = std::make_unique<spdlog::pattern_formatter>();
    console_formatter->add_flag<BracketedLevelFlag>('*')
        .add_flag<HangingIndentMessageFlag>('~')
        .set_pattern(kConsolePattern);
    console_sink->set_formatter(std::move(console_formatter));
    logger = std::make_shared<spdlog::logger>("aidin_hand2", console_sink);

    // Levels are filtered per sink, so the logger itself lets everything through
    logger->set_level(spdlog::level::trace);
    logger->flush_on(spdlog::level::warn);
  }
};

inline LoggerState& logger_state()
{
  static LoggerState state;
  return state;
}

}  // namespace aidin_hand2::log_sinks
