#pragma once
#include <spdlog/spdlog.h>

#include <memory>

namespace logx {
void init(const char* filename = "app.log",
          size_t max_bytes = 10 * 1024 * 1024,
          size_t max_files = 5,
          bool async = false);

// 只初始化一次；后续再次调用直接返回
void init_once(const char* filename = "app.log",
               size_t max_bytes = 10 * 1024 * 1024,
               size_t max_files = 5,
               bool async = false);

bool is_initialized();
inline void set_level(spdlog::level::level_enum lvl) { spdlog::set_level(lvl); }
inline void flush_on(spdlog::level::level_enum lvl) { spdlog::flush_on(lvl); }
}  // namespace logx

// —— 可选：保留 fmt 风格接口（*_F 后缀） ——
// 用法：LOGI_F("value={}", 123);
#define LOGT_F(...) SPDLOG_TRACE(__VA_ARGS__)
#define LOGD_F(...) SPDLOG_DEBUG(__VA_ARGS__)
#define LOGI_F(...) SPDLOG_INFO(__VA_ARGS__)
#define LOGW_F(...) SPDLOG_WARN(__VA_ARGS__)
#define LOGE_F(...) SPDLOG_ERROR(__VA_ARGS__)
#define LOGC_F(...) SPDLOG_CRITICAL(__VA_ARGS__)