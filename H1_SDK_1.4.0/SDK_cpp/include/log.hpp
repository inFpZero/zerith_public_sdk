#pragma once
#include <spdlog/spdlog.h>
#include <spdlog/fmt/fmt.h>

#include <memory>
#include <utility>

namespace logx {
// 只填写模块名，日志固定写入 /var/log/robot/<module_name>/normal/。
void init_once(const char* module_name = "app");

void init(const char* module_name = "app",
          size_t max_bytes = 10 * 1024 * 1024,
          size_t max_files = 5,
          bool async = false);

bool is_initialized();

// EVENT 使用独立 logger 和独立文件 sink；它不是 spdlog 原生的数值等级。
std::shared_ptr<spdlog::logger> event_logger();

void set_level(spdlog::level::level_enum lvl);
void flush_on(spdlog::level::level_enum lvl);

template <typename... Args>
inline void event_log(spdlog::source_loc loc,
                      spdlog::format_string_t<Args...> fmt,
                      Args&&... args) {
  auto logger = event_logger();
  if (logger) {
    const auto message = spdlog::fmt_lib::format(fmt, std::forward<Args>(args)...);
    logger->log(loc, spdlog::level::info, "{}", message);
    // EVENT 同步镜像到普通 Logger，并加标记，便于在 normal 文件中检索。
    spdlog::log(loc, spdlog::level::info, "[EVENT] {}", message);
  }
}
}  // namespace logx

// —— 可选：保留 fmt 风格接口（*_F 后缀） ——
// 用法：LOGI_F("value={}", 123);
#define LOGT_F(...) SPDLOG_TRACE(__VA_ARGS__)
#define LOGD_F(...) SPDLOG_DEBUG(__VA_ARGS__)
#define LOGI_F(...) SPDLOG_INFO(__VA_ARGS__)
#define LOGW_F(...) SPDLOG_WARN(__VA_ARGS__)
#define LOGE_F(...) SPDLOG_ERROR(__VA_ARGS__)
#define LOGC_F(...) SPDLOG_CRITICAL(__VA_ARGS__)

// EVENT 日志写入按模块名自动生成的独立目录，同时镜像到普通日志文件。
// 用法：LOG_EVENT("fault_id={}", fault_id);
#define LOG_EVENT(...) \
  ::logx::event_log({__FILE__, __LINE__, SPDLOG_FUNCTION}, __VA_ARGS__)
