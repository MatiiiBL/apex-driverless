#include "apex/core/log.hpp"

#include <atomic>
#include <cstdio>

namespace apex {

namespace {

std::atomic<LogLevel> g_level{LogLevel::Info};

const char* level_name(LogLevel level) {
    switch (level) {
        case LogLevel::Debug:
            return "debug";
        case LogLevel::Info:
            return "info";
        case LogLevel::Warn:
            return "warn";
        case LogLevel::Error:
            return "error";
        case LogLevel::Off:
            break;
    }
    return "";
}

}  // namespace

void set_log_level(LogLevel level) {
    g_level.store(level, std::memory_order_relaxed);
}

LogLevel log_level() {
    return g_level.load(std::memory_order_relaxed);
}

void log_message(LogLevel level, std::string_view text) {
    if (level < log_level() || level == LogLevel::Off) return;
    std::fprintf(stderr, "[apex %s] %.*s\n", level_name(level), static_cast<int>(text.size()),
                 text.data());
}

}  // namespace apex
