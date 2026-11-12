#pragma once

#include <format>
#include <string_view>
#include <utility>

// Minimal leveled logging to stderr. Library code logs sparingly (mode changes, recoveries,
// failures), never per message in a hot loop.
namespace apex {

enum class LogLevel { Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4 };

void set_log_level(LogLevel level);
[[nodiscard]] LogLevel log_level();
void log_message(LogLevel level, std::string_view text);

template <class... Args>
void log(LogLevel level, std::format_string<Args...> fmt, Args&&... args) {
    if (level < log_level()) return;
    log_message(level, std::format(fmt, std::forward<Args>(args)...));
}

}  // namespace apex
