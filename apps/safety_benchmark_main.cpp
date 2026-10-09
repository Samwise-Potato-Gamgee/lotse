// Runs the safety benchmark in both modes and writes the raw results and the
// combined summary.

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

#include "lotse/benchmark/safety_benchmark.hpp"
#include "lotse/core/map.hpp"

namespace {

/// The command line options of the benchmark tool.
struct Options {
    std::vector<std::string> mapPaths;
    std::vector<int> fleetSizes;
    std::vector<double> delayRates;
    int seeds = 10;
    std::string outDir = "results";
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
        } else if (flag == "--out-dir") {
            options.outDir = optionValue(argv, argc, index);
        } else if (flag == "--jobs") {
            options.jobs = std::stoi(optionValue(argv, argc, index));
        } else {
            throw std::runtime_error("unknown option " + flag);
        }
    }
    return options;
}

/// Returns the number of threads to use.
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
void runRange(const std::vector<lotse::benchmark::Scenario>& scenarios,
              const std::unordered_map<std::string, lotse::core::Map>& maps,
              std::vector<lotse::benchmark::RunOutcome>& outcomes, std::size_t first,
              std::size_t last, bool passOrderEnabled) {
    try {
        for (std::size_t index = first; index < last; ++index) {
            const lotse::benchmark::Scenario& scenario = scenarios[index];
            const lotse::core::Map& map = maps.at(scenario.mapName);
            outcomes[index] = lotse::benchmark::runScenario(map, scenario, passOrderEnabled);
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr, "benchmark worker failed: %s\n", error.what());
    }
}

/// Runs every scenario in parallel and returns the outcomes.
/// @param scenarios all scenarios
/// @param maps the loaded maps by name
/// @param jobs number of worker threads
/// @param passOrderEnabled false for the control run
[[nodiscard]] std::vector<lotse::benchmark::RunOutcome> runAll(
    const std::vector<lotse::benchmark::Scenario>& scenarios,
    const std::unordered_map<std::string, lotse::core::Map>& maps, int jobs,
    bool passOrderEnabled) {
    std::vector<lotse::benchmark::RunOutcome> outcomes(scenarios.size());
    const auto requested = static_cast<std::size_t>(jobs < 1 ? 1 : jobs);
    const std::size_t workers = requested < scenarios.size() ? requested : scenarios.size();
    std::vector<std::thread> threads;
    threads.reserve(workers);
    for (std::size_t worker = 0; worker < workers; ++worker) {
        const std::size_t first = (scenarios.size() * worker) / workers;
        const std::size_t last = (scenarios.size() * (worker + 1)) / workers;
        threads.emplace_back([&scenarios, &maps, &outcomes, first, last, passOrderEnabled]() {
            runRange(scenarios, maps, outcomes, first, last, passOrderEnabled);
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
                                const std::vector<lotse::benchmark::RunOutcome>& outcomes) {
    std::ofstream csv{path};
    if (!csv.is_open()) {
        return false;
    }
    csv << lotse::benchmark::csvHeader() << '\n';
    for (const lotse::benchmark::RunOutcome& outcome : outcomes) {
        csv << lotse::benchmark::toCsvRow(outcome) << '\n';
    }
    return true;
}

/// Builds the combined summary text for both modes.
/// @param passOutcomes the runs with the pass-order rule on
/// @param controlOutcomes the runs with the rule off
/// @param scenarioCount distinct map, fleet and seed combinations
/// @param delayLevels distinct delay levels
[[nodiscard]] std::string buildSummary(
    const std::vector<lotse::benchmark::RunOutcome>& passOutcomes,
    const std::vector<lotse::benchmark::RunOutcome>& controlOutcomes, int scenarioCount,
    int delayLevels) {
    lotse::benchmark::Totals passTotals;
    lotse::benchmark::Totals controlTotals;
    for (const lotse::benchmark::RunOutcome& outcome : passOutcomes) {
        lotse::benchmark::accumulate(passTotals, outcome);
    }
    for (const lotse::benchmark::RunOutcome& outcome : controlOutcomes) {
        lotse::benchmark::accumulate(controlTotals, outcome);
    }

    std::ostringstream text;
    text << lotse::benchmark::formatSummary("pass-order enabled", passTotals, scenarioCount,
                                            delayLevels);
    text << '\n';
    text << lotse::benchmark::formatSummary("pass-order disabled (control)", controlTotals,
                                            scenarioCount, delayLevels);
    text << '\n';
    text << lotse::benchmark::formatDelayTable(lotse::benchmark::groupByDelay(passOutcomes),
                                               lotse::benchmark::groupByDelay(controlOutcomes));
    return text.str();
}

/// Writes the two raw CSVs and the combined summary file.
/// @param passOutcomes the runs with the pass-order rule on
/// @param controlOutcomes the runs with the rule off
/// @param outDir directory to write into
/// @param scenarioCount distinct map, fleet and seed combinations
/// @param delayLevels distinct delay levels
[[nodiscard]] bool writeResults(const std::vector<lotse::benchmark::RunOutcome>& passOutcomes,
                                const std::vector<lotse::benchmark::RunOutcome>& controlOutcomes,
                                const std::string& outDir, int scenarioCount, int delayLevels) {
    if (!writeCsvFile(outDir + "/safety_runs.csv", passOutcomes)) {
        return false;
    }
    if (!writeCsvFile(outDir + "/safety_control_runs.csv", controlOutcomes)) {
        return false;
    }

    const std::string text =
        buildSummary(passOutcomes, controlOutcomes, scenarioCount, delayLevels);
    std::ofstream summary{outDir + "/safety_summary.txt"};
    if (!summary.is_open()) {
        return false;
    }
    summary << text;
    std::cout << text;
    return true;
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

        const std::vector<lotse::benchmark::Scenario> scenarios = lotse::benchmark::safetyScenarios(
            mapNames, options.fleetSizes, options.delayRates, options.seeds);
        const int scenarioCount = lotse::benchmark::countBaseScenarios(scenarios);
        const int delayLevels = lotse::benchmark::countDelayLevels(scenarios);
        const int jobs = resolveJobs(options.jobs);
        std::cout << "running " << scenarios.size() << " scenarios in both modes on " << jobs
                  << " threads\n";

        const std::vector<lotse::benchmark::RunOutcome> passOutcomes =
            runAll(scenarios, maps, jobs, true);
        const std::vector<lotse::benchmark::RunOutcome> controlOutcomes =
            runAll(scenarios, maps, jobs, false);

        if (!writeResults(passOutcomes, controlOutcomes, options.outDir, scenarioCount,
                          delayLevels)) {
            std::fprintf(stderr, "cannot write results into %s\n", options.outDir.c_str());
            return EXIT_FAILURE;
        }
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "safety_benchmark failed: %s\n", error.what());
        return EXIT_FAILURE;
    }
}
