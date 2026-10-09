// Builds and runs the safety benchmark: fixed scenarios, random starts and
// goals per seed, delays, and per-run outcomes for the raw CSV and the summary.

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "lotse/core/map.hpp"
#include "lotse/core/plan.hpp"

namespace lotse::benchmark {

/// One point in the benchmark grid: a map, a fleet size, a delay rate and a seed.
struct Scenario {
    std::string mapName;
    int fleetSize = 0;
    double delayProbability = 0.0;
    std::uint32_t seed = 0;
};

/// The measured outcome of one scenario.
struct RunOutcome {
    Scenario scenario{};
    bool passOrderEnabled = true;
    bool finished = false;
    bool deadlock = false;
    bool stepLimitHit = false;
    bool initialPlanFailed = false;
    int stepsUsed = 0;
    int replans = 0;
    int replanFailures = 0;
    int collisions = 0;
    int swaps = 0;
};

/// Totals across many runs, used for the human-readable summary.
struct Totals {
    int runs = 0;
    int executedRuns = 0;
    int finishedRuns = 0;
    int runsWithViolations = 0;
    int runsWithCollisions = 0;
    int runsWithSwaps = 0;
    int collisionEvents = 0;
    int swapEvents = 0;
    int deadlocks = 0;
    int stepLimitHits = 0;
    int initialPlanFailures = 0;
    int replans = 0;
    int replanFailures = 0;
};

/// Totals for one delay level.
struct DelayTotals {
    double delay = 0.0;
    Totals totals;
};

/// Builds every scenario from the given maps, fleet sizes, delay rates and seeds.
/// @param mapNames the maps to test
/// @param fleetSizes the fleet sizes to test
/// @param delayRates the per-step delay probabilities, including zero
/// @param seedsPerCombination how many seeds each combination gets
[[nodiscard]] std::vector<Scenario> safetyScenarios(const std::vector<std::string>& mapNames,
                                                    const std::vector<int>& fleetSizes,
                                                    const std::vector<double>& delayRates,
                                                    int seedsPerCombination);

/// Counts the distinct map, fleet and seed combinations, that is the base
/// scenarios that are each run at every delay level.
/// @param scenarios the full scenario list
[[nodiscard]] int countBaseScenarios(const std::vector<Scenario>& scenarios);

/// Counts the distinct delay levels in the scenario list.
/// @param scenarios the full scenario list
[[nodiscard]] int countDelayLevels(const std::vector<Scenario>& scenarios);

/// Chooses distinct random start and goal cells for a fleet.
/// @param map the warehouse grid
/// @param fleetSize number of robots
/// @param seed random seed
/// @return one agent per robot, or nothing when the map has too few free cells
[[nodiscard]] std::optional<std::vector<core::Agent>> makeFleet(const core::Map& map, int fleetSize,
                                                                std::uint32_t seed);

/// Runs one scenario with prioritized planning.
/// @param map the warehouse grid
/// @param scenario the map, fleet size, delay rate and seed
/// @param passOrderEnabled false to run the control without the pass-order rule
[[nodiscard]] RunOutcome runScenario(const core::Map& map, const Scenario& scenario,
                                     bool passOrderEnabled);

/// Returns the header line of the raw results CSV.
[[nodiscard]] std::string csvHeader();

/// Formats one outcome as a CSV line.
/// @param outcome the run to format
[[nodiscard]] std::string toCsvRow(const RunOutcome& outcome);

/// Adds one outcome to the totals.
/// @param totals the totals to update
/// @param outcome the run to add
void accumulate(Totals& totals, const RunOutcome& outcome);

/// Groups outcomes by delay level, smallest delay first.
/// @param outcomes every run outcome of one mode
[[nodiscard]] std::vector<DelayTotals> groupByDelay(const std::vector<RunOutcome>& outcomes);

/// Formats the totals as a short human-readable summary.
/// @param title a heading, for example "pass-order enabled"
/// @param totals the totals to format
/// @param scenarioCount distinct map, fleet and seed combinations
/// @param delayLevelCount distinct delay levels
[[nodiscard]] std::string formatSummary(const std::string& title, const Totals& totals,
                                        int scenarioCount, int delayLevelCount);

/// Formats the per-delay table comparing both modes.
/// @param passRows per-delay totals with the pass-order rule on
/// @param controlRows per-delay totals with the rule off
[[nodiscard]] std::string formatDelayTable(const std::vector<DelayTotals>& passRows,
                                           const std::vector<DelayTotals>& controlRows);

}  // namespace lotse::benchmark
