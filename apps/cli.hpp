#pragma once

#include <cstdint>
#include <filesystem>
#include <initializer_list>
#include <map>
#include <optional>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

#include <nlohmann/json_fwd.hpp>

// Command-line plumbing for the `apex` tool: option parsing and file I/O with error messages
// that name the file or option at fault.
namespace apex::cli {

// A command line that does not parse. main() prints it with a pointer to --help and exits 2.
class UsageError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// The options after the command word: `--name value` pairs and bare `--flag`s, in any order.
// Positional arguments are not used by any command and are rejected.
class Options {
public:
    // Throws UsageError for an option not in `valued` or `flags`, a repeated option, or a
    // valued option without its value.
    Options(std::span<const std::string_view> args, std::initializer_list<std::string_view> valued,
            std::initializer_list<std::string_view> flags = {});

    [[nodiscard]] bool has(std::string_view name) const;
    [[nodiscard]] std::optional<std::string> get(std::string_view name) const;
    // Throws UsageError if the option was not given.
    [[nodiscard]] std::string required(std::string_view name) const;
    // A non-negative integer option; throws UsageError if it is not one.
    [[nodiscard]] std::optional<std::uint64_t> get_unsigned(std::string_view name) const;

private:
    std::map<std::string, std::string, std::less<>> values_;
    std::set<std::string, std::less<>> flags_;
};

// Reads and parses a JSON file; throws std::runtime_error naming the file if it cannot be
// opened or parsed.
[[nodiscard]] nlohmann::json read_json(const std::filesystem::path& path);
// Writes `text` to `path`, creating its parent directories; throws std::runtime_error naming
// the file if it cannot.
void write_file(const std::filesystem::path& path, std::string_view text);

}  // namespace apex::cli
