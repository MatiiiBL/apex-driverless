#include "cli.hpp"

#include <algorithm>
#include <charconv>
#include <fstream>

#include <nlohmann/json.hpp>

namespace apex::cli {

Options::Options(std::span<const std::string_view> args,
                 std::initializer_list<std::string_view> valued,
                 std::initializer_list<std::string_view> flags) {
    const auto contains = [](std::initializer_list<std::string_view> list, std::string_view name) {
        return std::find(list.begin(), list.end(), name) != list.end();
    };
    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string_view arg = args[i];
        if (!arg.starts_with("--")) {
            throw UsageError("unexpected argument '" + std::string(arg) + "'");
        }
        const std::string_view name = arg.substr(2);
        if (has(name)) throw UsageError("--" + std::string(name) + " given twice");
        if (contains(flags, name)) {
            flags_.emplace(name);
        } else if (contains(valued, name)) {
            if (i + 1 >= args.size()) throw UsageError("--" + std::string(name) + " needs a value");
            values_.emplace(name, args[++i]);
        } else {
            throw UsageError("unknown option '" + std::string(arg) + "'");
        }
    }
}

bool Options::has(std::string_view name) const {
    return values_.contains(name) || flags_.contains(name);
}

std::optional<std::string> Options::get(std::string_view name) const {
    const auto it = values_.find(name);
    if (it == values_.end()) return std::nullopt;
    return it->second;
}

std::string Options::required(std::string_view name) const {
    std::optional<std::string> value = get(name);
    if (!value) throw UsageError("--" + std::string(name) + " is required");
    return *value;
}

std::optional<std::uint64_t> Options::get_unsigned(std::string_view name) const {
    const std::optional<std::string> text = get(name);
    if (!text) return std::nullopt;
    std::uint64_t value = 0;
    const char* end = text->data() + text->size();
    const auto [ptr, ec] = std::from_chars(text->data(), end, value);
    if (ec != std::errc() || ptr != end) {
        throw UsageError("--" + std::string(name) + " must be a non-negative integer, not '" +
                         *text + "'");
    }
    return value;
}

nlohmann::json read_json(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open " + path.string());
    try {
        return nlohmann::json::parse(in);
    } catch (const nlohmann::json::parse_error& e) {
        throw std::runtime_error(path.string() + ": " + e.what());
    }
}

void write_file(const std::filesystem::path& path, std::string_view text) {
    if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("cannot write " + path.string());
    out << text;
    out.close();
    if (!out) throw std::runtime_error("failed writing " + path.string());
}

}  // namespace apex::cli
