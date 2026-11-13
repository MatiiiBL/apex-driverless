#pragma once

#include <span>
#include <string_view>

// The `apex` subcommands. Each receives the arguments after its name (without --help, which
// main() handles), returns the process exit code, and throws cli::UsageError for a bad command
// line or another std::exception for a failure.
namespace apex::cli {

struct Command {
    std::string_view name;
    std::string_view summary;  // one line, for `apex --help`
    std::string_view help;     // the full text, for `apex <name> --help`
    int (*run)(std::span<const std::string_view> args);
};

extern const Command kTrackCommand;
extern const Command kRunCommand;
extern const Command kSweepCommand;
extern const Command kReplayCommand;

}  // namespace apex::cli
