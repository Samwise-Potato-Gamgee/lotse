// Replays one delayed run in the terminal so robots can be watched waiting.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "lotse/benchmark/safety_benchmark.hpp"
#include "lotse/core/executor.hpp"
#include "lotse/core/map.hpp"
#include "lotse/core/prioritized_planner.hpp"

namespace {

/// The command line options of the demo tool.
struct Options {
    std::string mapPath = "maps/warehouse_small.map";
    int fleetSize = 6;
    double delayProbability = 0.25;
    std::uint32_t seed = 7;
    bool animate = false;
};

/// Reads the value after an option, or throws when it is missing.
/// @param argv the argument vector
/// @param argc the argument count
/// @param index the index of the option itself
std::string optionValue(char** argv, int argc, int index) {
    if (index + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + argv[index]);
    }
    return argv[index + 1];
}

/// Parses the command line options.
/// @param argc argument count
/// @param argv argument vector
[[nodiscard]] Options parseOptions(int argc, char** argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string flag = argv[index];
        if (flag == "--map") {
            options.mapPath = optionValue(argv, argc, index);
            ++index;
        } else if (flag == "--fleet") {
            options.fleetSize = std::stoi(optionValue(argv, argc, index));
            ++index;
        } else if (flag == "--delay") {
            options.delayProbability = std::stod(optionValue(argv, argc, index));
            ++index;
        } else if (flag == "--seed") {
            options.seed = static_cast<std::uint32_t>(std::stoul(optionValue(argv, argc, index)));
            ++index;
        } else if (flag == "--animate") {
            options.animate = true;
        } else {
            throw std::runtime_error("unknown option " + flag);
        }
    }
    return options;
}

/// Returns the display symbol for a robot index.
/// @param index zero based robot index
char robotSymbol(int index) {
    if (index < 10) {
        return static_cast<char>('0' + index);
    }
    return static_cast<char>('a' + (index - 10));
}

/// Renders one frame as character rows, obstacles '#' and robots as symbols.
/// @param map the warehouse grid
/// @param frame the robot positions
[[nodiscard]] std::vector<std::string> renderFrame(const lotse::core::Map& map,
                                                   const lotse::judge::Frame& frame) {
    std::vector<std::string> rows(static_cast<std::size_t>(map.height()),
                                  std::string(static_cast<std::size_t>(map.width()), '.'));
    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            if (!map.isFree(lotse::core::GridCell{x, y})) {
                rows[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)] = '#';
            }
        }
    }
    for (std::size_t robot = 0; robot < frame.size(); ++robot) {
        const lotse::core::GridCell cell = frame[robot];
        char& slot = rows[static_cast<std::size_t>(cell.y)][static_cast<std::size_t>(cell.x)];
        if (slot == '.') {
            slot = robotSymbol(static_cast<int>(robot));
        } else if (slot != '#') {
            slot = 'X';
        }
    }
    return rows;
}

/// Prints one frame with its step number.
/// @param rows the rendered map
/// @param step the execution step
void printFrame(const std::vector<std::string>& rows, int step) {
    std::cout << "step " << step << '\n';
    for (const std::string& row : rows) {
        std::cout << row << '\n';
    }
    std::cout << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parseOptions(argc, argv);
        const std::optional<lotse::core::Map> map = lotse::core::Map::fromFile(options.mapPath);
        if (!map.has_value()) {
            std::fprintf(stderr, "cannot read map %s\n", options.mapPath.c_str());
            return EXIT_FAILURE;
        }
        const std::optional<std::vector<lotse::core::Agent>> fleet =
            lotse::benchmark::makeFleet(*map, options.fleetSize, options.seed);
        if (!fleet.has_value()) {
            std::fprintf(stderr, "map %s has too few free cells for %d robots\n",
                         options.mapPath.c_str(), options.fleetSize);
            return EXIT_FAILURE;
        }

        lotse::core::ExecutionConfig config;
        config.seed = options.seed;
        config.delayProbability = options.delayProbability;

        lotse::core::PrioritizedPlanner planner;
        const lotse::core::ExecutionResult result =
            lotse::core::execute(*map, *fleet, planner, config);

        std::cout << "map " << map->name() << ", " << options.fleetSize << " robots, delay "
                  << options.delayProbability << ", seed " << options.seed << '\n';
        std::cout << "finished " << (result.finished ? "yes" : "no") << ", deadlock "
                  << (result.deadlock ? "yes" : "no") << ", step limit "
                  << (result.stepLimitHit ? "yes" : "no") << ", initial plan failed "
                  << (result.initialPlanFailed ? "yes" : "no") << '\n';
        std::cout << "steps " << result.stepsUsed << ", replans " << result.replans
                  << ", replan failures " << result.replanFailures << ", collisions "
                  << result.verdict.collisions << ", swaps " << result.verdict.swaps << "\n\n";

        for (std::size_t step = 0; step < result.log.frames.size(); ++step) {
            if (options.animate) {
                std::cout << "\033[2J\033[H";
            }
            printFrame(renderFrame(*map, result.log.frames[step]), static_cast<int>(step));
            if (options.animate) {
                std::this_thread::sleep_for(std::chrono::milliseconds(150));
            }
        }
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "replay_demo failed: %s\n", error.what());
        return EXIT_FAILURE;
    }
}
