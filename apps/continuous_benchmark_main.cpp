// Runs the continuous-mode safety benchmark (with the pass-order control) and
// the throughput benchmark, writing the raw results and short summaries.

#include <cstddef>
#include <cstdint>
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

#include "lotse/benchmark/continuous_benchmark.hpp"
#include "lotse/core/map.hpp"

namespace {

/// The command line options of the continuous benchmark tool.
struct Options {
    std::string mode = "both";
    std::string planner = "prioritized";
    std::vector<std::string> mapPaths;
    std::vector<int> fleetSizes;
    std::vector<double> delayRates;
    int seeds = 5;
    int steps = 1500;
    int arrivalInterval = 1;
    int ageingInterval = 25;
    std::string outDir = "results";
    std::string tag;
    int jobs = 0;
};

/// Splits a comma separated list into fields.
/// @param text the list to split
/// @return the fields, empty ones removed
std::vector<std::string> split(const std::string& text) {
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
    for (int index = 1; index < argc; index += 2) {
        const std::string flag = argv[index];
        if (flag == "--maps") {
            options.mapPaths = split(optionValue(argv, argc, index));
        } else if (flag == "--fleet") {
            for (const std::string& field : split(optionValue(argv, argc, index))) {
                options.fleetSizes.push_back(std::stoi(field));
            }
        } else if (flag == "--delays") {
            for (const std::string& field : split(optionValue(argv, argc, index))) {
                options.delayRates.push_back(std::stod(field));
            }
        } else if (flag == "--seeds") {
            options.seeds = std::stoi(optionValue(argv, argc, index));
        } else if (flag == "--steps") {
            options.steps = std::stoi(optionValue(argv, argc, index));
        } else if (flag == "--arrival") {
            options.arrivalInterval = std::stoi(optionValue(argv, argc, index));
        } else if (flag == "--ageing") {
            options.ageingInterval = std::stoi(optionValue(argv, argc, index));
        } else if (flag == "--out-dir") {
            options.outDir = optionValue(argv, argc, index);
        } else if (flag == "--tag") {
            options.tag = optionValue(argv, argc, index);
        } else if (flag == "--jobs") {
            options.jobs = std::stoi(optionValue(argv, argc, index));
        } else if (flag == "--mode") {
            options.mode = optionValue(argv, argc, index);
        } else if (flag == "--planner") {
            options.planner = optionValue(argv, argc, index);
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
/// @param passOrderEnabled false for the control run
/// @param plannerName the planner to use
void runRange(const std::vector<lotse::benchmark::ContinuousScenario>& scenarios,
              const std::unordered_map<std::string, lotse::core::Map>& maps,
              std::vector<lotse::benchmark::ContinuousOutcome>& outcomes, std::size_t first,
              std::size_t last, bool passOrderEnabled, const std::string& plannerName) {
    try {
        for (std::size_t index = first; index < last; ++index) {
            const lotse::benchmark::ContinuousScenario& scenario = scenarios[index];
            const lotse::core::Map& map = maps.at(scenario.mapName);
            outcomes[index] = lotse::benchmark::runContinuousScenario(
                map, scenario, passOrderEnabled, plannerName);
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr, "continuous worker failed: %s\n", error.what());
    }
}

/// Runs every scenario in parallel and returns the outcomes.
/// @param scenarios all scenarios
/// @param maps the loaded maps by name
/// @param jobs number of worker threads
/// @param passOrderEnabled false for the control run
/// @param plannerName the planner to use
[[nodiscard]] std::vector<lotse::benchmark::ContinuousOutcome> runAll(
    const std::vector<lotse::benchmark::ContinuousScenario>& scenarios,
    const std::unordered_map<std::string, lotse::core::Map>& maps, int jobs, bool passOrderEnabled,
    const std::string& plannerName) {
    std::vector<lotse::benchmark::ContinuousOutcome> outcomes(scenarios.size());
    const auto requested = static_cast<std::size_t>(jobs < 1 ? 1 : jobs);
    const std::size_t workers = requested < scenarios.size() ? requested : scenarios.size();
    std::vector<std::thread> threads;
    threads.reserve(workers);
    for (std::size_t worker = 0; worker < workers; ++worker) {
        const std::size_t first = (scenarios.size() * worker) / workers;
        const std::size_t last = (scenarios.size() * (worker + 1)) / workers;
        threads.emplace_back(
            [&scenarios, &maps, &outcomes, first, last, passOrderEnabled, &plannerName]() {
                runRange(scenarios, maps, outcomes, first, last, passOrderEnabled, plannerName);
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
                                const std::vector<lotse::benchmark::ContinuousOutcome>& outcomes) {
    std::ofstream csv{path};
    if (!csv.is_open()) {
        return false;
    }
    csv << lotse::benchmark::continuousCsvHeader() << '\n';
    for (const lotse::benchmark::ContinuousOutcome& outcome : outcomes) {
        csv << lotse::benchmark::toContinuousCsvRow(outcome) << '\n';
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

/// Builds the continuous safety summary for both modes.
/// @param passOutcomes the runs with the pass-order rule on
/// @param controlOutcomes the runs with the rule off
[[nodiscard]] std::string buildSafetySummary(
    const std::vector<lotse::benchmark::ContinuousOutcome>& passOutcomes,
    const std::vector<lotse::benchmark::ContinuousOutcome>& controlOutcomes) {
    lotse::benchmark::ContinuousTotals passTotals;
    lotse::benchmark::ContinuousTotals controlTotals;
    for (const lotse::benchmark::ContinuousOutcome& outcome : passOutcomes) {
        lotse::benchmark::accumulateContinuous(passTotals, outcome);
    }
    for (const lotse::benchmark::ContinuousOutcome& outcome : controlOutcomes) {
        lotse::benchmark::accumulateContinuous(controlTotals, outcome);
    }

    std::ostringstream text;
    text << "continuous safety benchmark\n";
    text << "  pass-order: runs " << passTotals.runs << ", deadlocks " << passTotals.deadlocks
         << ", runs w/ collision or swap " << passTotals.runsWithViolations << ", collisions "
         << passTotals.collisionEvents << ", swaps " << passTotals.swapEvents << '\n';
    text << "  longest wait " << passTotals.longestWaitSteps << " steps, orders completed "
         << passTotals.ordersCompleted << ", planning calls " << passTotals.planningCalls
         << ", planning failures " << passTotals.planningFailures << ", fallbacks "
         << passTotals.fallbacks << " (timeouts " << passTotals.fallbackTimeouts << ")\n";
    text << "  control:    runs " << controlTotals.runs << ", deadlocks " << controlTotals.deadlocks
         << ", runs w/ collision or swap " << controlTotals.runsWithViolations << ", collisions "
         << controlTotals.collisionEvents << ", swaps " << controlTotals.swapEvents << '\n';
    text << '\n';
    text << lotse::benchmark::formatContinuousDelayTable(
        lotse::benchmark::groupContinuousByDelay(passOutcomes),
        lotse::benchmark::groupContinuousByDelay(controlOutcomes));
    return text.str();
}

/// Splits outcomes into the no-delay part and the delayed part.
/// @param outcomes all outcomes
/// @param delayed true to keep the delayed runs, false for the zero-delay runs
[[nodiscard]] std::vector<lotse::benchmark::ContinuousOutcome> filterDelay(
    const std::vector<lotse::benchmark::ContinuousOutcome>& outcomes, bool delayed) {
    std::vector<lotse::benchmark::ContinuousOutcome> kept;
    for (const lotse::benchmark::ContinuousOutcome& outcome : outcomes) {
        const bool isDelayed = outcome.scenario.delayProbability > 0.0;
        if (isDelayed == delayed) {
            kept.push_back(outcome);
        }
    }
    return kept;
}

/// Builds the throughput summary.
/// @param outcomes every throughput run
[[nodiscard]] std::string buildThroughputSummary(
    const std::vector<lotse::benchmark::ContinuousOutcome>& outcomes) {
    std::ostringstream text;
    text << "continuous throughput benchmark\n";
    text << "  assumption: one grid step is 1.0 second\n";
    text << lotse::benchmark::formatThroughputTable(
        lotse::benchmark::groupContinuousByFleet(filterDelay(outcomes, false)),
        lotse::benchmark::groupContinuousByFleet(filterDelay(outcomes, true)));
    text << '\n';
    text << lotse::benchmark::formatThroughputByMap(outcomes);
    return text.str();
}

/// Builds the throughput comparison of prioritized planning and CBS.
/// @param prioritized the throughput runs with prioritized planning
/// @param cbs the throughput runs with CBS and fallback
[[nodiscard]] std::string buildThroughputComparison(
    const std::vector<lotse::benchmark::ContinuousOutcome>& prioritized,
    const std::vector<lotse::benchmark::ContinuousOutcome>& cbs) {
    return lotse::benchmark::formatThroughputComparison(
        lotse::benchmark::groupContinuousByFleet(prioritized),
        lotse::benchmark::groupContinuousByFleet(cbs));
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parseOptions(argc, argv);
        if (options.mapPaths.empty() || options.fleetSizes.empty() || options.delayRates.empty()) {
            std::fprintf(stderr, "--maps, --fleet and --delays are required\n");
            return EXIT_FAILURE;
        }

        std::unordered_map<std::string, lotse::core::Map> maps;
        std::vector<std::string> mapNames;
        for (const std::string& path : options.mapPaths) {
            const std::optional<lotse::core::Map> map = lotse::core::Map::fromFile(path);
            if (!map.has_value()) {
                std::fprintf(stderr, "cannot read map %s\n", path.c_str());
                return EXIT_FAILURE;
            }
            mapNames.push_back(path);
            maps.emplace(path, *map);
        }

        const std::vector<lotse::benchmark::ContinuousScenario> scenarios =
            lotse::benchmark::continuousScenarios(mapNames, options.fleetSizes, options.delayRates,
                                                  options.seeds, options.steps,
                                                  options.arrivalInterval, options.ageingInterval);
        const int jobs = resolveJobs(options.jobs);
        std::cout << "running " << scenarios.size() << " continuous scenarios per mode on " << jobs
                  << " threads with planner " << options.planner << "\n";

        if (options.mode == "safety" || options.mode == "both") {
            const std::vector<lotse::benchmark::ContinuousOutcome> passOutcomes =
                runAll(scenarios, maps, jobs, true, options.planner);
            const std::vector<lotse::benchmark::ContinuousOutcome> controlOutcomes =
                runAll(scenarios, maps, jobs, false, options.planner);
            if (!writeCsvFile(options.outDir + "/continuous_safety" + options.tag + "_runs.csv",
                              passOutcomes) ||
                !writeCsvFile(options.outDir + "/continuous_control" + options.tag + "_runs.csv",
                              controlOutcomes) ||
                !writeTextFile(options.outDir + "/continuous_safety" + options.tag + "_summary.txt",
                               buildSafetySummary(passOutcomes, controlOutcomes))) {
                std::fprintf(stderr, "cannot write continuous safety results into %s\n",
                             options.outDir.c_str());
                return EXIT_FAILURE;
            }
        }

        if (options.mode == "throughput" || options.mode == "both") {
            const std::vector<lotse::benchmark::ContinuousOutcome> outcomes =
                runAll(scenarios, maps, jobs, true, options.planner);
            if (!writeCsvFile(options.outDir + "/throughput" + options.tag + "_runs.csv",
                              outcomes) ||
                !writeTextFile(options.outDir + "/throughput" + options.tag + "_summary.txt",
                               buildThroughputSummary(outcomes))) {
                std::fprintf(stderr, "cannot write throughput results into %s\n",
                             options.outDir.c_str());
                return EXIT_FAILURE;
            }
        }

        if (options.mode == "throughput-compare") {
            const std::vector<lotse::benchmark::ContinuousOutcome> prioritized =
                runAll(scenarios, maps, jobs, true, "prioritized");
            const std::vector<lotse::benchmark::ContinuousOutcome> cbs =
                runAll(scenarios, maps, jobs, true, "cbs");
            if (!writeCsvFile(options.outDir + "/throughput_prioritized_runs.csv", prioritized) ||
                !writeCsvFile(options.outDir + "/throughput_cbs_runs.csv", cbs) ||
                !writeTextFile(options.outDir + "/throughput_planner_comparison.txt",
                               buildThroughputComparison(prioritized, cbs))) {
                std::fprintf(stderr, "cannot write the throughput comparison into %s\n",
                             options.outDir.c_str());
                return EXIT_FAILURE;
            }
        }
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "continuous_benchmark failed: %s\n", error.what());
        return EXIT_FAILURE;
    }
}
