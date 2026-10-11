// Runs the liveness benchmark: the real-time FleetManager against the grid
// plant over many seeded runs, with and without the one-step gap planning
// option, and writes the raw CSV plus a short stall summary.

#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "lotse/benchmark/liveness_benchmark.hpp"
#include "lotse/core/map.hpp"

namespace {

/// The command line options of the liveness benchmark tool.
struct Options {
    std::vector<std::string> mapPaths;
    std::vector<int> fleetSizes;
    std::vector<double> cellSeconds;
    int seeds = 10;
    int seedStart = 0;
    int orders = 12;
    double slowdown = 0.0;
    double stop = 0.0;
    double horizonSeconds = 600.0;
    double deadlockSeconds = 120.0;
    double stallSeconds = 120.0;
    double tickSeconds = 0.05;
    bool backgroundPlanning = false;
    bool oneStepGap = true;
    std::string outDir = "results";
    std::string tag;
    int jobs = 0;
};

/// Splits a comma separated list into fields.
/// @param text the list to split
[[nodiscard]] std::vector<std::string> split(const std::string& text) {
    std::vector<std::string> fields;
    std::string current;
    for (const char symbol : text) {
        if (symbol == ',') {
            if (!current.empty()) {
                fields.push_back(current);
            }
            current.clear();
        } else {
            current.push_back(symbol);
        }
    }
    if (!current.empty()) {
        fields.push_back(current);
    }
    return fields;
}

/// Reads the value after an option, or throws when it is missing.
/// @param argv the argument vector
/// @param argc the argument count
/// @param index the index of the option itself
[[nodiscard]] std::string optionValue(char** argv, int argc, int index) {
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
    for (int index = 1; index < argc; index += 2) {
        const std::string flag = argv[index];
        if (flag == "--maps") {
            options.mapPaths = split(optionValue(argv, argc, index));
        } else if (flag == "--fleet") {
            for (const std::string& field : split(optionValue(argv, argc, index))) {
                options.fleetSizes.push_back(std::stoi(field));
            }
        } else if (flag == "--cell-seconds") {
            for (const std::string& field : split(optionValue(argv, argc, index))) {
                options.cellSeconds.push_back(std::stod(field));
            }
        } else if (flag == "--seeds") {
            options.seeds = std::stoi(optionValue(argv, argc, index));
        } else if (flag == "--seed-start") {
            options.seedStart = std::stoi(optionValue(argv, argc, index));
        } else if (flag == "--horizon") {
            options.horizonSeconds = std::stod(optionValue(argv, argc, index));
        } else if (flag == "--deadlock") {
            options.deadlockSeconds = std::stod(optionValue(argv, argc, index));
        } else if (flag == "--stall") {
            options.stallSeconds = std::stod(optionValue(argv, argc, index));
        } else if (flag == "--tick") {
            options.tickSeconds = std::stod(optionValue(argv, argc, index));
        } else if (flag == "--orders") {
            options.orders = std::stoi(optionValue(argv, argc, index));
        } else if (flag == "--slowdown") {
            options.slowdown = std::stod(optionValue(argv, argc, index));
        } else if (flag == "--stop") {
            options.stop = std::stod(optionValue(argv, argc, index));
        } else if (flag == "--background") {
            options.backgroundPlanning = optionValue(argv, argc, index) == "on";
        } else if (flag == "--gap") {
            options.oneStepGap = optionValue(argv, argc, index) == "on";
        } else if (flag == "--out-dir") {
            options.outDir = optionValue(argv, argc, index);
        } else if (flag == "--tag") {
            options.tag = optionValue(argv, argc, index);
        } else if (flag == "--jobs") {
            options.jobs = std::stoi(optionValue(argv, argc, index));
        } else {
            throw std::runtime_error("unknown option " + flag);
        }
    }
    return options;
}

/// Returns the number of worker threads to use.
/// @param requested the requested job count, zero for all cores
int resolveJobs(int requested) {
    if (requested > 0) {
        return requested;
    }
    const unsigned int cores = std::thread::hardware_concurrency();
    return cores == 0U ? 1 : static_cast<int>(cores);
}

/// Runs scenarios first..last, storing each outcome in place.
/// @param scenarios all scenarios
/// @param maps the loaded maps by name
/// @param outcomes output vector, same size as scenarios
/// @param first first scenario index
/// @param last one past the last scenario index
/// @param oneStepGap true to plan with a one-step gap
void runRange(const std::vector<lotse::benchmark::LivenessScenario>& scenarios,
              const std::unordered_map<std::string, lotse::core::Map>& maps,
              std::vector<lotse::benchmark::LivenessOutcome>& outcomes, std::size_t first,
              std::size_t last, bool oneStepGap) {
    try {
        for (std::size_t index = first; index < last; ++index) {
            const lotse::benchmark::LivenessScenario& scenario = scenarios[index];
            outcomes[index] = lotse::benchmark::runLivenessScenario(maps.at(scenario.mapName),
                                                                    scenario, oneStepGap);
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr, "liveness worker failed: %s\n", error.what());
    }
}

/// Runs every scenario in parallel and returns the outcomes.
/// @param scenarios all scenarios
/// @param maps the loaded maps by name
/// @param jobs number of worker threads
/// @param oneStepGap true to plan with a one-step gap
[[nodiscard]] std::vector<lotse::benchmark::LivenessOutcome> runAll(
    const std::vector<lotse::benchmark::LivenessScenario>& scenarios,
    const std::unordered_map<std::string, lotse::core::Map>& maps, int jobs, bool oneStepGap) {
    std::vector<lotse::benchmark::LivenessOutcome> outcomes(scenarios.size());
    const auto requested = static_cast<std::size_t>(jobs < 1 ? 1 : jobs);
    const std::size_t workers = requested < scenarios.size() ? requested : scenarios.size();
    std::vector<std::thread> threads;
    threads.reserve(workers);
    for (std::size_t worker = 0; worker < workers; ++worker) {
        const std::size_t first = (scenarios.size() * worker) / workers;
        const std::size_t last = (scenarios.size() * (worker + 1)) / workers;
        threads.emplace_back([&scenarios, &maps, &outcomes, first, last, oneStepGap]() {
            runRange(scenarios, maps, outcomes, first, last, oneStepGap);
        });
    }
    for (std::thread& thread : threads) {
        thread.join();
    }
    return outcomes;
}

/// Writes one raw results CSV, header first.
/// @param path file to write
/// @param outcomes the runs to write
[[nodiscard]] bool writeCsvFile(const std::string& path,
                                const std::vector<lotse::benchmark::LivenessOutcome>& outcomes) {
    std::ofstream csv{path};
    if (!csv.is_open()) {
        return false;
    }
    csv << lotse::benchmark::livenessCsvHeader() << '\n';
    for (const lotse::benchmark::LivenessOutcome& outcome : outcomes) {
        csv << lotse::benchmark::toLivenessCsvRow(outcome) << '\n';
    }
    return true;
}

/// Writes a text file, creating it fresh.
/// @param path file to write
/// @param text the contents
[[nodiscard]] bool writeTextFile(const std::string& path, const std::string& text) {
    std::ofstream out{path};
    if (!out.is_open()) {
        return false;
    }
    out << text;
    std::cout << text;
    return true;
}

/// Builds the short liveness summary.
/// @param outcomes every run of one mode
/// @param oneStepGap whether the one-step gap option was on
[[nodiscard]] std::string buildSummary(
    const std::vector<lotse::benchmark::LivenessOutcome>& outcomes, bool oneStepGap) {
    lotse::benchmark::LivenessTotals totals;
    for (const lotse::benchmark::LivenessOutcome& outcome : outcomes) {
        lotse::benchmark::accumulateLiveness(totals, outcome);
    }
    std::ostringstream text;
    text << "liveness benchmark\n";
    text << "  one-step gap: " << (oneStepGap ? "on" : "off") << '\n';
    int deadlockedRuns = 0;
    for (const lotse::benchmark::LivenessOutcome& outcome : outcomes) {
        if (outcome.deadlocked) {
            ++deadlockedRuns;
        }
    }
    text << "  runs " << totals.runs << ", completed " << totals.completedRuns << ", stalled "
         << totals.stalledRuns << ", deadlocked " << deadlockedRuns << ", runs with collision "
         << totals.collisionRuns << ", runs with swap " << totals.swapRuns << '\n';
    text << "  orders completed " << totals.ordersCompleted << " / " << totals.ordersTotal << '\n';
    for (const lotse::benchmark::LivenessOutcome& outcome : outcomes) {
        if (outcome.stalled) {
            text << "  stall: map=" << outcome.scenario.mapName
                 << " fleet=" << outcome.scenario.fleetSize
                 << " cell_seconds=" << outcome.scenario.cellSeconds
                 << " seed=" << outcome.scenario.seed << " at " << outcome.seconds
                 << " s deadlocked=" << (outcome.deadlocked ? 1 : 0) << "\n";
            text << "    " << outcome.detail << '\n';
        }
    }
    return text.str();
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parseOptions(argc, argv);
        if (options.mapPaths.empty() || options.fleetSizes.empty() || options.cellSeconds.empty()) {
            std::fprintf(stderr, "--maps, --fleet and --cell-seconds are required\n");
            return EXIT_FAILURE;
        }

        std::unordered_map<std::string, lotse::core::Map> maps;
        for (const std::string& path : options.mapPaths) {
            const std::optional<lotse::core::Map> map = lotse::core::Map::fromFile(path);
            if (!map.has_value()) {
                std::fprintf(stderr, "cannot read map %s\n", path.c_str());
                return EXIT_FAILURE;
            }
            maps.emplace(path, *map);
        }

        const std::vector<std::string> mapNames = options.mapPaths;
        std::vector<lotse::benchmark::LivenessScenario> scenarios =
            lotse::benchmark::livenessScenarios(mapNames, options.fleetSizes, options.cellSeconds,
                                                options.seeds);
        for (lotse::benchmark::LivenessScenario& scenario : scenarios) {
            scenario.orderCount = options.orders;
            scenario.slowdownProbability = options.slowdown;
            scenario.stopProbability = options.stop;
            scenario.horizonSeconds = options.horizonSeconds;
            scenario.deadlockPatienceSeconds = options.deadlockSeconds;
            scenario.stallSeconds = options.stallSeconds;
            scenario.tickSeconds = options.tickSeconds;
            scenario.backgroundPlanning = options.backgroundPlanning;
            scenario.seed += static_cast<std::uint32_t>(options.seedStart);
        }
        const int jobs = resolveJobs(options.jobs);
        std::cout << "running " << scenarios.size() << " liveness scenarios on " << jobs
                  << " threads, gap " << (options.oneStepGap ? "on" : "off") << "\n";

        const std::vector<lotse::benchmark::LivenessOutcome> outcomes =
            runAll(scenarios, maps, jobs, options.oneStepGap);
        if (!writeCsvFile(options.outDir + "/liveness" + options.tag + "_runs.csv", outcomes) ||
            !writeTextFile(options.outDir + "/liveness" + options.tag + "_summary.txt",
                           buildSummary(outcomes, options.oneStepGap))) {
            std::fprintf(stderr, "cannot write liveness results into %s\n", options.outDir.c_str());
            return EXIT_FAILURE;
        }
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "liveness_benchmark failed: %s\n", error.what());
        return EXIT_FAILURE;
    }
}
