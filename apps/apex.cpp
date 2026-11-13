// The `apex` command-line tool: generate tracks, run the car in the simulator, run experiment
// sweeps and record replays (docs/design.md, section 5.6; docs/evaluation.md for the
// experiments).

#include <array>
#include <exception>
#include <iostream>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "cli.hpp"
#include "commands.hpp"

namespace {

using apex::cli::Command;

const std::array<const Command*, 1> kCommands = {&apex::cli::kTrackCommand};

void print_usage(std::ostream& out) {
    out << "apex: an autonomous Formula Student Driverless stack and its simulator.\n\n"
           "Usage: apex <command> [options]\n\nCommands:\n";
    for (const Command* command : kCommands) {
        out << "  " << command->name << std::string(10 - command->name.size(), ' ')
            << command->summary << "\n";
    }
    out << "\nRun 'apex <command> --help' for a command's options.\n";
}

bool wants_help(std::span<const std::string_view> args) {
    for (const std::string_view arg : args) {
        if (arg == "--help" || arg == "-h") return true;
    }
    return false;
}

}  // namespace

int main(int argc, char** argv) {
    const std::vector<std::string_view> args(argv + 1, argv + argc);
    if (args.empty()) {
        print_usage(std::cerr);
        return 2;
    }
    if (args[0] == "--help" || args[0] == "-h" || args[0] == "help") {
        print_usage(std::cout);
        return 0;
    }
    const Command* command = nullptr;
    for (const Command* candidate : kCommands) {
        if (candidate->name == args[0]) command = candidate;
    }
    if (command == nullptr) {
        std::cerr << "apex: unknown command '" << args[0] << "'\n\n";
        print_usage(std::cerr);
        return 2;
    }

    const std::span<const std::string_view> rest = std::span(args).subspan(1);
    if (wants_help(rest)) {
        std::cout << command->help;
        return 0;
    }
    try {
        return command->run(rest);
    } catch (const apex::cli::UsageError& e) {
        std::cerr << "apex " << command->name << ": " << e.what() << "\nRun 'apex " << command->name
                  << " --help' for usage.\n";
        return 2;
    } catch (const std::exception& e) {
        std::cerr << "apex " << command->name << ": " << e.what() << "\n";
        return 1;
    }
}
