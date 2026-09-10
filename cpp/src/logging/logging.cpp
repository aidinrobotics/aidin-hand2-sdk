#include <aidin_hand2/logging/logging.hpp>

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>

#include "logging/log.hpp"
#include "logging/log_sinks.hpp"

namespace aidin_hand2
{

using namespace log_sinks;

// -------------------------------- Public API --------------------------------

void set_log_level(LogLevel level)
{
  LoggerState& state = logger_state();
  std::lock_guard<std::mutex> lock(state.mutex);
  state.console_level = level;
  if (state.console_enabled) {
    state.console_sink->set_level(to_spdlog_level(level));
  }
}

void set_log_to_console(bool enabled)
{
  LoggerState& state = logger_state();
  std::lock_guard<std::mutex> lock(state.mutex);
  state.console_enabled = enabled;
  state.console_sink->set_level(enabled ? to_spdlog_level(state.console_level) : spdlog::level::off);
}

void set_log_to_file(const std::string& path)
{
  LoggerState& state = logger_state();
  std::lock_guard<std::mutex> lock(state.mutex);
  try {
    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        path, kRotatingFileMaxBytes, kRotatingFileCount);
    file_sink->set_level(spdlog::level::debug);
    file_sink->set_pattern(kFilePattern);
    auto& sinks = state.logger->sinks();
    if (state.file_sink) {
      sinks.erase(std::remove(sinks.begin(), sinks.end(), state.file_sink), sinks.end());
    }
    state.file_sink = std::move(file_sink);
    sinks.push_back(state.file_sink);
  } catch (const spdlog::spdlog_ex& open_failure) {
    state.logger->warn("failed to open log file {}: {}", path, open_failure.what());
  }
}

void set_log_callback(LogCallback callback)
{
  LoggerState& state = logger_state();
  std::lock_guard<std::mutex> lock(state.mutex);
  auto& sinks = state.logger->sinks();
  if (state.callback_sink) {
    sinks.erase(std::remove(sinks.begin(), sinks.end(), state.callback_sink), sinks.end());
    state.callback_sink.reset();
  }
  if (!callback) {
    return;
  }
  state.callback_sink = std::make_shared<CallbackSink>(std::move(callback));
  state.callback_sink->set_level(spdlog::level::debug);
  sinks.push_back(state.callback_sink);
}

void flush_log()
{
  logger_state().logger->flush();
}

// ------------------------------- Internal API -------------------------------

void log_debug(const std::string& message) { logger_state().logger->debug(message); }
void log_debug(HandSide side, const std::string& message) { logger_state().logger->debug("{}{}", hand_prefix(side), message); }

void log_info(const std::string& message) { logger_state().logger->info(message); }
void log_info(HandSide side, const std::string& message) { logger_state().logger->info("{}{}", hand_prefix(side), message); }

void log_warn(const std::string& message) { logger_state().logger->warn(message); }
void log_warn(HandSide side, const std::string& message) { logger_state().logger->warn("{}{}", hand_prefix(side), message); }

void log_critical(const std::string& message) { logger_state().logger->critical(message); }
void log_critical(HandSide side, const std::string& message) { logger_state().logger->critical("{}{}", hand_prefix(side), message); }

void log_exception(ErrorCode code, const std::string& message)
{
  logger_state().logger->error("[exception] {}: {}", to_string(code), message);
}

void log_exception(HandSide side, ErrorCode code, const std::string& message)
{
  logger_state().logger->error("{}[exception] {}: {}", hand_prefix(side), to_string(code), message);
}

void log_warn_status(ErrorCode code, const std::string& message)
{
  logger_state().logger->warn("{}: {}", to_string(code), message);
}

}  // namespace aidin_hand2
