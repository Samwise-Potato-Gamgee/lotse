// The liveness benchmark: runs the real-time FleetManager against the grid
// plant with variable seeded cell-crossing times, a manual clock and continuous
// orders, then flags a run slow when it misses its discrete stage-3 baseline by
// three times or more.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "lotse/core/fleet_manager.hpp"
#include "lotse/core/map.hpp"

namespace lotse::benchmark {

/// One point in the liveness benchmark grid.
struct LivenessScenario {
    std::string mapName;
    int fleetSize = 4;
    double cellSeconds = 1.0;
    double slowdownProbability = 0.0;
    double stopProbability = 0.0;
    double maxStopSeconds = 2.0;
    double tickSeconds = 0.05;
    int orderCount = 12;
    int arrivalIntervalTicks = 20;
    double windowSeconds = 6.0;
    double priorityAgeingSeconds = 3.75;
    double deadlockPatienceSeconds = 120.0;
    double stallSeconds = 120.0;
    double horizonSeconds = 600.0;
    bool backgroundPlanning = false;
    bool collectDiagnostics = false;
    std::uint32_t seed = 0;
};

/// The measured outcome of one liveness run.
struct LivenessOutcome {
    LivenessScenario scenario{};
    bool oneStepGap = false;
    bool completed = false;
    bool stalled = false;
    bool deadlocked = false;
    int ordersCompleted = 0;
    int ordersTotal = 0;
    int collisions = 0;
    int swaps = 0;
    long long ticks = 0;
    double seconds = 0.0;
    int planningCalls = 0;
    int planningFailures = 0;
    int planAdoptions = 0;
    int planRejections = 0;
    double baselineSeconds = 0.0;
    double ratio = 0.0;
    bool slow = false;
    core::ManagerDiagnostics diagnostics{};
    std::string detail;
};

/// Runs one liveness scenario with the real-time manager.
/// @param map the warehouse grid
/// @param scenario the map, fleet, crossing time and horizon
/// @param oneStepGap true to plan with a one-step gap after every vacated cell
[[nodiscard]] LivenessOutcome runLivenessScenario(const core::Map& map,
                                                  const LivenessScenario& scenario,
                                                  bool oneStepGap);

/// Builds the full scenario grid: every map, fleet size, crossing time and seed.
/// @param mapNames the maps to test
/// @param fleetSizes the fleet sizes to test
/// @param cellSeconds the per-cell crossing times to test
/// @param seedsPerCombination how many seeds each combination gets
[[nodiscard]] std::vector<LivenessScenario> livenessScenarios(
    const std::vector<std::string>& mapNames, const std::vector<int>& fleetSizes,
    const std::vector<double>& cellSeconds, int seedsPerCombination);

/// Returns the header line of the liveness raw CSV.
[[nodiscard]] std::string livenessCsvHeader();

/// Formats one outcome as a CSV line.
/// @param outcome the run to format
[[nodiscard]] std::string toLivenessCsvRow(const LivenessOutcome& outcome);

/// Totals across many liveness runs.
struct LivenessTotals {
    int runs = 0;
    int completedRuns = 0;
    int stalledRuns = 0;
    int slowRuns = 0;
    int collisionRuns = 0;
    int swapRuns = 0;
    long long ordersCompleted = 0;
    long long ordersTotal = 0;
};

/// Adds one outcome to the totals.
/// @param totals the totals to update
/// @param outcome the run to add
void accumulateLiveness(LivenessTotals& totals, const LivenessOutcome& outcome);

}  // namespace lotse::benchmark
