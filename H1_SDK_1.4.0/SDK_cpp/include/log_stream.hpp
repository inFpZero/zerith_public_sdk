#pragma once
#include "log.hpp"

#include <sstream>

namespace logx {

class LogStream {
 public:
  explicit LogStream(spdlog::level::level_enum lvl,
                     const char* file = nullptr,
                     int line = 0,
                     const char* func = nullptr,
                     bool event = false)
      : level_(lvl), file_(file), line_(line), func_(func), event_(event) {}
  ~LogStream() { emit(); }

  template <class T>
  LogStream& operator<<(const T& v) {
    oss_ << v;
    return *this;
  }

  using Manip = std::ostream& (*)(std::ostream&);
  LogStream& operator<<(Manip m) {
    m(oss_);
    if (m == static_cast<Manip>(std::endl<char, std::char_traits<char>>)) {
      emit();
    }
    return *this;
  }

 private:
  void emit() {
    const std::string s = oss_.str();
    if (!s.empty()) {
      if (file_) {
        spdlog::source_loc loc{file_, line_, func_ ? func_ : "<unknown>"};
        if (event_) {
          auto logger = event_logger();
          if (logger) {
            logger->log(loc, spdlog::level::info, "{}", s);
            spdlog::log(loc, spdlog::level::info, "[EVENT] {}", s);
          }
        } else {
          spdlog::log(loc, level_, "{}", s);
        }
      } else {
        if (event_) {
          auto logger = event_logger();
          if (logger) {
            logger->log(spdlog::level::info, "{}", s);
            spdlog::log(spdlog::level::info, "[EVENT] {}", s);
          }
        } else {
          spdlog::log(level_, "{}", s);
        }
      }
      oss_.str(std::string{});
      oss_.clear();
    }
  }

  std::ostringstream oss_;
  spdlog::level::level_enum level_;
  const char* file_;
  int line_;
  const char* func_;
  bool event_;
};

// 便捷宏：传入文件/行/函数以启用 [%s:%#]
#define LOGT() ::logx::LogStream(spdlog::level::trace, __FILE__, __LINE__, SPDLOG_FUNCTION)
#define LOGD() ::logx::LogStream(spdlog::level::debug, __FILE__, __LINE__, SPDLOG_FUNCTION)
#define LOGI() ::logx::LogStream(spdlog::level::info, __FILE__, __LINE__, SPDLOG_FUNCTION)
#define LOGW() ::logx::LogStream(spdlog::level::warn, __FILE__, __LINE__, SPDLOG_FUNCTION)
#define LOGE() ::logx::LogStream(spdlog::level::err, __FILE__, __LINE__, SPDLOG_FUNCTION)
#define LOGC() ::logx::LogStream(spdlog::level::critical, __FILE__, __LINE__, SPDLOG_FUNCTION)
#define LOG_EVENT_STREAM() ::logx::LogStream(spdlog::level::info, __FILE__, __LINE__, SPDLOG_FUNCTION, true)

}  // namespace logx
