// Runs the public MAPF benchmark instances through prioritized planning and
// CBS, writes the raw comparison CSV and a short summary, and reports the one
// stage 2 Lotse scenario that prioritized planning could not solve.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "lotse/benchmark/mapf_benchmark.hpp"
#include "lotse/benchmark/safety_benchmark.hpp"
#include "lotse/core/cbs.hpp"
#include "lotse/core/map.hpp"
#include "lotse/core/prioritized_planner.hpp"

namespace {

/// The command line options of the comparison tool.
struct Options {
    std::string dataDir = ".bench-data/mapf";
    std::vector<std::string> mapNames;
    std::vector<int> robotCounts;
    int scenarios = 10;
    int timeLimitMs = 5000;
    int jobs = 0;
    std::string outDir = "results";
    bool lotseHard = false;
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
        if (flag == "--data-dir") {
            options.dataDir = optionValue(argv, argc, index);
            ++index;
        } else if (flag == "--maps") {
            options.mapNames = split(optionValue(argv, argc, index));
            ++index;
        } else if (flag == "--robots") {
            for (const std::string& field : split(optionValue(argv, argc, index))) {
                options.robotCounts.push_back(std::stoi(field));
            }
            ++index;
        } else if (flag == "--scenarios") {
            options.scenarios = std::stoi(optionValue(argv, argc, index));
            ++index;
        } else if (flag == "--time-limit-ms") {
            options.timeLimitMs = std::stoi(optionValue(argv, argc, index));
            ++index;
        } else if (flag == "--jobs") {
            options.jobs = std::stoi(optionValue(argv, argc, index));
            ++index;
        } else if (flag == "--out-dir") {
            options.outDir = optionValue(argv, argc, index);
            ++index;
        } else if (flag == "--lotse-hard") {
            options.lotseHard = true;
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

/// One planner call to run.
struct Task {
    std::size_t instance = 0;
    std::string planner;
};

/// One benchmark instance: a map, a fleet and its scenario number.
struct Instance {
    std::string mapName;
    int robotCount = 0;
    int scenario = 0;
    const lotse::core::Map* map = nullptr;
    std::vector<lotse::core::Agent> agents;
};

/// Returns true when every start and goal is a free cell.
/// @param map the warehouse grid
/// @param agents the fleet to check
[[nodiscard]] bool agentsAreValid(const lotse::core::Map& map,
                                  const std::vector<lotse::core::Agent>& agents) {
    return std::ranges::all_of(agents, [&map](const lotse::core::Agent& agent) {
        return map.isFree(agent.start) && map.isFree(agent.goal);
    });
}

/// Runs one planner call and turns it into a comparison row.
/// @param instance the benchmark instance
/// @param planner "prioritized" or "cbs"
/// @param timeLimitMs the per-call time limit for CBS
[[nodiscard]] lotse::benchmark::ComparisonRow runTask(const Instance& instance,
                                                      const std::string& planner, int timeLimitMs) {
    lotse::benchmark::ComparisonRow row;
    row.mapName = instance.mapName;
    row.robotCount = instance.robotCount;
    row.scenario = instance.scenario;
    row.planner = planner;

    if (planner == "cbs") {
        lotse::core::CbsPlanner cbs(lotse::core::CbsOptions{timeLimitMs, 0, 0, 2000000});
        const lotse::benchmark::PlannerRun run =
            lotse::benchmark::runPlanner(*instance.map, instance.agents, cbs);
        row.solved = run.solved;
        row.timedOut = run.timedOut;
        row.cost = run.cost;
        row.seconds = run.seconds;
        return row;
    }
    lotse::core::PrioritizedPlanner prioritized(lotse::core::PrioritizedOptions{
        lotse::core::AgentOrder::FarthestFirst, 0, 0, 8, timeLimitMs});
    const lotse::benchmark::PlannerRun run =
        lotse::benchmark::runPlanner(*instance.map, instance.agents, prioritized);
    row.solved = run.solved;
    row.timedOut = run.timedOut;
    row.cost = run.cost;
    row.seconds = run.seconds;
    return row;
}

/// Runs every task in parallel and returns the rows in task order.
/// @param tasks the planner calls
/// @param instances the benchmark instances
/// @param jobs number of worker threads
/// @param timeLimitMs the per-call time limit for CBS
[[nodiscard]] std::vector<lotse::benchmark::ComparisonRow> runAll(
    const std::vector<Task>& tasks, const std::vector<Instance>& instances, int jobs,
    int timeLimitMs) {
    std::vector<lotse::benchmark::ComparisonRow> rows(tasks.size());
    const auto requested = static_cast<std::size_t>(jobs < 1 ? 1 : jobs);
    const std::size_t workers = requested < tasks.size() ? requested : tasks.size();
    std::vector<std::thread> threads;
    threads.reserve(workers);
    for (std::size_t worker = 0; worker < workers; ++worker) {
        const std::size_t first = (tasks.size() * worker) / workers;
        const std::size_t last = (tasks.size() * (worker + 1)) / workers;
        threads.emplace_back([&tasks, &instances, &rows, first, last, timeLimitMs]() {
            try {
                for (std::size_t index = first; index < last; ++index) {
                    rows[index] = runTask(instances[tasks[index].instance], tasks[index].planner,
                                          timeLimitMs);
                }
            } catch (const std::exception& error) {
                std::fprintf(stderr, "comparison worker failed: %s\n", error.what());
            }
        });
    }
    for (std::thread& thread : threads) {
        thread.join();
    }
    return rows;
}

/// Runs the one stage 2 scenario prioritized planning could not solve.
/// @param timeLimitMs the per-call time limit
/// @return the text report
[[nodiscard]] std::string runLotseHardScenario(int timeLimitMs) {
    std::ostringstream text;
    const std::optional<lotse::core::Map> map =
        lotse::core::Map::fromFile("maps/warehouse_aisles.map");
    if (!map.has_value()) {
        return "lotse hard scenario: maps/warehouse_aisles.map not found\n";
    }
    const std::optional<std::vector<lotse::core::Agent>> fleet =
        lotse::benchmark::makeFleet(*map, 20, 4);
    if (!fleet.has_value()) {
        return "lotse hard scenario: could not build the fleet\n";
    }
    text << "stage 2 hard scenario: warehouse_aisles, 20 robots, seed 4\n";

    lotse::core::PrioritizedPlanner prioritized(lotse::core::PrioritizedOptions{
        lotse::core::AgentOrder::FarthestFirst, 0, 0, 8, timeLimitMs});
    const lotse::benchmark::PlannerRun first =
        lotse::benchmark::runPlanner(*map, *fleet, prioritized);
    text << "  prioritized: solved " << (first.solved ? "yes" : "no") << ", time " << std::fixed
         << std::setprecision(3) << first.seconds << " s\n";

    lotse::core::CbsPlanner cbs(lotse::core::CbsOptions{timeLimitMs, 0, 0, 2000000});
    const lotse::benchmark::PlannerRun second = lotse::benchmark::runPlanner(*map, *fleet, cbs);
    text << "  cbs: solved " << (second.solved ? "yes" : "no") << ", timed out "
         << (second.timedOut ? "yes" : "no") << ", cost " << second.cost << ", time " << std::fixed
         << std::setprecision(3) << second.seconds << " s\n";
    return text.str();
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Options options = parseOptions(argc, argv);
        if (options.mapNames.empty()) {
            options.mapNames = {"random-32-32-20", "room-32-32-4", "maze-32-32-2", "empty-32-32"};
        }
        if (options.robotCounts.empty()) {
            options.robotCounts = {5, 10, 15, 20, 25};
        }

        std::unordered_map<std::string, lotse::core::Map> maps;
        std::vector<Instance> instances;
        int skipped = 0;
        for (const std::string& mapName : options.mapNames) {
            const std::string mapPath = options.dataDir + "/maps/" + mapName + ".map";
            const std::optional<lotse::core::Map> map = lotse::core::Map::fromFile(mapPath);
            if (!map.has_value()) {
                std::fprintf(stderr, "cannot read map %s\n", mapPath.c_str());
                return EXIT_FAILURE;
            }
            maps.emplace(mapName, *map);
            const lotse::core::Map& stored = maps.at(mapName);
            for (const int robotCount : options.robotCounts) {
                for (int scenario = 1; scenario <= options.scenarios; ++scenario) {
                    const std::string scenPath = options.dataDir + "/scen-random/" + mapName +
                                                 "-random-" + std::to_string(scenario) + ".scen";
                    const std::vector<lotse::core::Agent> agents =
                        lotse::benchmark::readScen(scenPath, robotCount);
                    if (static_cast<int>(agents.size()) < robotCount ||
                        !agentsAreValid(stored, agents)) {
                        ++skipped;
                        continue;
                    }
                    instances.push_back(Instance{mapName, robotCount, scenario, &stored, agents});
                }
            }
        }
        if (instances.empty()) {
            std::fprintf(stderr, "no valid instances under %s\n", options.dataDir.c_str());
            return EXIT_FAILURE;
        }

        std::vector<Task> tasks;
        for (std::size_t index = 0; index < instances.size(); ++index) {
            tasks.push_back(Task{index, "prioritized"});
            tasks.push_back(Task{index, "cbs"});
        }
        const int jobs = resolveJobs(options.jobs);
        std::cout << "running " << tasks.size() << " planner calls on " << instances.size()
                  << " instances on " << jobs << " threads (skipped " << skipped << ", time limit "
                  << options.timeLimitMs << " ms)\n";

        const std::vector<lotse::benchmark::ComparisonRow> rows =
            runAll(tasks, instances, jobs, options.timeLimitMs);

        std::ofstream csv{options.outDir + "/planner_comparison_runs.csv"};
        if (csv.is_open()) {
            csv << lotse::benchmark::comparisonCsvHeader() << '\n';
            for (const lotse::benchmark::ComparisonRow& row : rows) {
                csv << lotse::benchmark::toComparisonCsvRow(row) << '\n';
            }
        } else {
            std::fprintf(stderr, "cannot write the comparison CSV into %s\n",
                         options.outDir.c_str());
            return EXIT_FAILURE;
        }
        const std::string summary = lotse::benchmark::formatComparisonSummary(rows);
        std::ofstream summaryFile{options.outDir + "/planner_comparison_summary.txt"};
        if (summaryFile.is_open()) {
            summaryFile << summary;
        }
        std::cout << summary;

        if (options.lotseHard) {
            const std::string hard = runLotseHardScenario(60000);
            std::ofstream hardFile{options.outDir + "/planner_comparison_lotse_hard.txt"};
            if (hardFile.is_open()) {
                hardFile << hard;
            }
            std::cout << hard;
        }
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "planner_comparison failed: %s\n", error.what());
        return EXIT_FAILURE;
    }
}
