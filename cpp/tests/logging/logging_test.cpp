// 목적: 로깅 배선 smoke — public 함수 실기능 + 파일 sink(debug까지) + callback sink + 포맷 +
//   to_string(ErrorCode). 콘솔은 캡처가 번거로워 파일 sink 로 검증한다 (파일은 debug 까지
//   항상 기록이 규약).

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <aidin_hand2/logging/logging.hpp>
#include <aidin_hand2/types/error.hpp>

#include "logging/log.hpp"

using namespace aidin_hand2;

namespace
{

int g_failures = 0;

void check(bool ok, const char* what)
{
  if (!ok) { std::printf("  FAIL: %s\n", what); ++g_failures; }
}

}  // namespace

int main()
{
  // to_string(ErrorCode)
  check(std::string(to_string(ErrorCode::InterfaceUnavailable)) == "InterfaceUnavailable", "to_string InterfaceUnavailable");
  check(std::string(to_string(ErrorCode::None)) == "None", "to_string None");

  // 파일 sink 활성 후 각 채널 기록 → flush → 내용 확인.
  const char* log_path = "logging_test_output.log";
  std::remove(log_path);
  set_log_to_file(log_path);

  log_info("lifecycle sample");
  log_warn_status(ErrorCode::InterfaceUnavailable, "swallowed status sample");
  log_exception(ErrorCode::WrongCallOrder, "exception sample");  // error + [exception] 마커
  flush_log();

  std::ifstream file(log_path);
  check(file.good(), "log file created");
  std::stringstream buffer;
  buffer << file.rdbuf();
  const std::string content = buffer.str();

  check(content.find("[aidin_hand2]") != std::string::npos, "tag in output");
  check(content.find("lifecycle sample") != std::string::npos, "info written");
  check(content.find("Unavailable: swallowed status sample") != std::string::npos,
        "status log = Name: message");
  check(content.find("[exception] WrongCallOrder: exception sample") != std::string::npos,
        "exception marker written");

  std::remove(log_path);

  // callback sink — (레벨, 포맷 없는 본문) 수신, debug 까지 전달, 교체·해제.
  std::vector<std::pair<LogLevel, std::string>> received;
  set_log_callback([&received](LogLevel level, const std::string& message) {
    received.emplace_back(level, message);
  });

  log_debug("callback debug sample");
  log_warn("callback warn sample");
  check(received.size() == 2, "callback received both");
  check(received.size() == 2 && received[0].first == LogLevel::Debug, "callback debug level");
  check(received.size() == 2 && received[0].second == "callback debug sample",
        "callback raw body (no prefix)");
  check(received.size() == 2 && received[1].first == LogLevel::Warn, "callback warn level");

  // 재호출 = 교체 — 이전 callback 으로는 더 이상 안 간다.
  std::vector<std::pair<LogLevel, std::string>> replaced;
  set_log_callback([&replaced](LogLevel level, const std::string& message) {
    replaced.emplace_back(level, message);
  });
  log_info("after replace");
  check(received.size() == 2, "old callback detached after replace");
  check(replaced.size() == 1 && replaced[0].second == "after replace", "new callback attached");

  // nullptr = 해제.
  set_log_callback(nullptr);
  log_info("after unset");
  check(replaced.size() == 1, "callback detached after nullptr");

  std::printf("logging test: %s\n", g_failures == 0 ? "OK" : "FAIL");
  return g_failures == 0 ? 0 : 1;
}
