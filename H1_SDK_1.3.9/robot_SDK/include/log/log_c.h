#ifndef LOG_C_H
#define LOG_C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>

void logx_c_trace(const char* file, int line, const char* func, const char* fmt, ...);
void logx_c_debug(const char* file, int line, const char* func, const char* fmt, ...);
void logx_c_info(const char* file, int line, const char* func, const char* fmt, ...);
void logx_c_warn(const char* file, int line, const char* func, const char* fmt, ...);
void logx_c_error(const char* file, int line, const char* func, const char* fmt, ...);
void logx_c_critical(const char* file, int line, const char* func, const char* fmt, ...);

/* 便捷宏，传入文件/行/函数 */
#define LOGX_TRACE(fmt, ...) logx_c_trace(__FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)
#define LOGX_DEBUG(fmt, ...) logx_c_debug(__FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)
#define LOGX_INFO(fmt, ...)  logx_c_info(__FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)
#define LOGX_WARN(fmt, ...)  logx_c_warn(__FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)
#define LOGX_ERROR(fmt, ...) logx_c_error(__FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)
#define LOGX_CRITICAL(fmt, ...) logx_c_critical(__FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif // LOG_C_H