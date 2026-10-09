// Builds and runs the continuous-mode benchmark: an order stream per scenario,
// delays, the pass-order control, and per-run outcomes for the raw CSV and the
// throughput and safety summaries.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "lotse/core/map.hpp"

namespace lotse::benchmark {

/// One point in the continuous benchmark grid.
struct ContinuousScenario {
    std::string mapName;
    int fleetSize = 0;
    double delayProbability = 0.0;
    std::uint32_t seed = 0;
    int steps = 2000;
    int arrivalInterval = 1;
    int priorityAgeingInterval = 25;
};

/// The measured outcome of one continuous run.
struct ContinuousOutcome {
    ContinuousScenario scenario{};
    bool passOrderEnabled = true;
    bool deadlock = false;
    bool allOrdersCompleted = false;
    int stepsRun = 0;
    int ordersReleased = 0;
    int ordersCompleted = 0;
    int droppedOrders = 0;
    int pendingAtEnd = 0;
    int longestWaitSteps = 0;
    int longestWaitPriority = 0;
    int medianWaitSteps = 0;
    int percentile95WaitSteps = 0;
    double meanWaitSteps = 0.0;
    long long waitTotalSteps = 0;
    int waitCount = 0;
    int collisions = 0;
    int swaps = 0;
    int planningCalls = 0;
    int planningFailures = 0;
    int fallbacks = 0;
    int fallbackTimeouts = 0;
};

/// Totals across many continuous runs.
struct ContinuousTotals {
    int runs = 0;
    int deadlocks = 0;
    int runsWithViolations = 0;
    int collisionEvents = 0;
    int swapEvents = 0;
    long long stepsRun = 0;
    long long ordersReleased = 0;
    long long ordersCompleted = 0;
    int longestWaitSteps = 0;
    int percentile95WaitSteps = 0;
    long long waitTotal = 0;
    long long waitCount = 0;
    int planningCalls = 0;
    int planningFailures = 0;
    int fallbacks = 0;
    int fallbackTimeouts = 0;

    /// Returns the mean wait over every released order.
    [[nodiscard]] double meanWaitSteps() const noexcept;

    /// Returns orders completed per 1,000 steps.
    [[nodiscard]] double ordersPerThousandSteps() const noexcept;

    /// Returns orders completed per hour, assuming one step is one second.
    [[nodiscard]] double ordersPerHour() const noexcept;
};

/// Totals for one fleet size and delay level.
struct FleetTotals {
    int fleetSize = 0;
    double delay = 0.0;
    ContinuousTotals totals;
};

/// Builds every scenario from the given maps, fleet sizes, delay rates and seeds.
/// @param mapNames the maps to test
/// @param fleetSizes the fleet sizes to test
/// @param delayRates the per-step delay probabilities
/// @param seedsPerCombination how many seeds each combination gets
/// @param steps the run horizon of every scenario
/// @param arrivalInterval the mean steps between two order arrivals
/// @param ageingInterval steps of waiting that add one priority point
[[nodiscard]] std::vector<ContinuousScenario> continuousScenarios(
    const std::vector<std::string>& mapNames, const std::vector<int>& fleetSizes,
    const std::vector<double>& delayRates, int seedsPerCombination, int steps, int arrivalInterval,
    int ageingInterval);

/// Chooses distinct random start cells for a fleet.
/// @param map the warehouse grid
/// @param fleetSize number of robots
/// @param seed random seed
[[nodiscard]] std::vector<core::GridCell> makeContinuousStarts(const core::Map& map, int fleetSize,
                                                               std::uint32_t seed);

/// Runs one continuous scenario.
/// @param map the warehouse grid
/// @param scenario the map, fleet size, delay rate, seed and horizon
/// @param passOrderEnabled false to run the control without the pass-order rule
/// @param plannerName "prioritized" or "cbs"
[[nodiscard]] ContinuousOutcome runContinuousScenario(const core::Map& map,
                                                      const ContinuousScenario& scenario,
                                                      bool passOrderEnabled,
                                                      const std::string& plannerName);

/// Returns the header line of the raw continuous CSV.
[[nodiscard]] std::string continuousCsvHeader();

/// Formats one continuous outcome as a CSV line.
/// @param outcome the run to format
[[nodiscard]] std::string toContinuousCsvRow(const ContinuousOutcome& outcome);

/// Adds one outcome to the totals.
/// @param totals the totals to update
/// @param outcome the run to add
void accumulateContinuous(ContinuousTotals& totals, const ContinuousOutcome& outcome);

/// Groups outcomes by delay level, smallest delay first.
/// @param outcomes every run outcome of one mode
[[nodiscard]] std::vector<FleetTotals> groupContinuousByDelay(
    const std::vector<ContinuousOutcome>& outcomes);

/// Groups outcomes by fleet size, smallest first.
/// @param outcomes every run outcome of one mode
[[nodiscard]] std::vector<FleetTotals> groupContinuousByFleet(
    const std::vector<ContinuousOutcome>& outcomes);

/// Formats the per-delay continuous safety table.
/// @param passRows per-delay totals with the pass-order rule on
/// @param controlRows per-delay totals with the rule off
[[nodiscard]] std::string formatContinuousDelayTable(const std::vector<FleetTotals>& passRows,
                                                     const std::vector<FleetTotals>& controlRows);

/// Formats the throughput table, one row per fleet size.
/// @param noDelayRows per-fleet totals without delays
/// @param delayRows per-fleet totals with delays
[[nodiscard]] std::string formatThroughputTable(const std::vector<FleetTotals>& noDelayRows,
                                                const std::vector<FleetTotals>& delayRows);

/// Formats the throughput comparison of two planners, one row per fleet size.
/// @param prioritizedRows per-fleet totals for prioritized planning
/// @param cbsRows per-fleet totals for CBS with fallback
[[nodiscard]] std::string formatThroughputComparison(
    const std::vector<FleetTotals>& prioritizedRows, const std::vector<FleetTotals>& cbsRows);

/// Formats the throughput table broken down by map and fleet size.
/// @param outcomes every throughput run
[[nodiscard]] std::string formatThroughputByMap(const std::vector<ContinuousOutcome>& outcomes);

}  // namespace lotse::benchmark
