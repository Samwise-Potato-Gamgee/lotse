// Implements the continuous-mode benchmark declared in continuous_benchmark.hpp.

#include "lotse/benchmark/continuous_benchmark.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <map>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "lotse/core/cbs.hpp"
#include "lotse/core/continuous_runner.hpp"
#include "lotse/core/order.hpp"
#include "lotse/core/prioritized_planner.hpp"
#include "lotse/core/task_assigner.hpp"

namespace lotse::benchmark {
namespace {

/// The step-to-seconds assumption used for the orders-per-hour figure.
constexpr double kSecondsPerStep = 1.0;

/// Formats a double with a fixed number of decimals.
/// @param value the value to format
/// @param precision the number of decimals
[[nodiscard]] std::string formatFixed(double value, int precision) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}

/// Returns a map path without its directory or extension.
/// @param path the map path
[[nodiscard]] std::string shortMapName(const std::string& path) {
    const std::size_t slash = path.find_last_of('/');
    const std::string file = slash == std::string::npos ? path : path.substr(slash + 1);
    const std::size_t dot = file.find_last_of('.');
    return dot == std::string::npos ? file : file.substr(0, dot);
}

/// Groups outcomes by delay level, smallest first.
/// @param outcomes the runs to group
[[nodiscard]] std::vector<FleetTotals> groupByDelay(
    const std::vector<ContinuousOutcome>& outcomes) {
    std::vector<const ContinuousOutcome*> sorted;
    sorted.reserve(outcomes.size());
    for (const ContinuousOutcome& outcome : outcomes) {
        sorted.push_back(&outcome);
    }
    std::sort(sorted.begin(), sorted.end(),
              [](const ContinuousOutcome* lhs, const ContinuousOutcome* rhs) {
                  return lhs->scenario.delayProbability < rhs->scenario.delayProbability;
              });

    constexpr double kTolerance = 1e-9;
    std::vector<FleetTotals> rows;
    for (const ContinuousOutcome* outcome : sorted) {
        const double delay = outcome->scenario.delayProbability;
        if (rows.empty() || std::abs(delay - rows.back().delay) > kTolerance) {
            FleetTotals row;
            row.delay = delay;
            rows.push_back(row);
        }
        accumulateContinuous(rows.back().totals, *outcome);
    }
    return rows;
}

/// Groups outcomes by fleet size, smallest first.
/// @param outcomes the runs to group
[[nodiscard]] std::vector<FleetTotals> groupByFleet(
    const std::vector<ContinuousOutcome>& outcomes) {
    std::vector<const ContinuousOutcome*> sorted;
    sorted.reserve(outcomes.size());
    for (const ContinuousOutcome& outcome : outcomes) {
        sorted.push_back(&outcome);
    }
    std::sort(sorted.begin(), sorted.end(),
              [](const ContinuousOutcome* lhs, const ContinuousOutcome* rhs) {
                  return lhs->scenario.fleetSize < rhs->scenario.fleetSize;
              });

    std::vector<FleetTotals> rows;
    for (const ContinuousOutcome* outcome : sorted) {
        const int fleet = outcome->scenario.fleetSize;
        if (rows.empty() || fleet != rows.back().fleetSize) {
            FleetTotals row;
            row.fleetSize = fleet;
            rows.push_back(row);
        }
        accumulateContinuous(rows.back().totals, *outcome);
    }
    return rows;
}

}  // namespace

double ContinuousTotals::meanWaitSteps() const noexcept {
    if (waitCount == 0) {
        return 0.0;
    }
    return static_cast<double>(waitTotal) / static_cast<double>(waitCount);
}

double ContinuousTotals::ordersPerThousandSteps() const noexcept {
    if (stepsRun == 0) {
        return 0.0;
    }
    return (1000.0 * static_cast<double>(ordersCompleted)) / static_cast<double>(stepsRun);
}

double ContinuousTotals::ordersPerHour() const noexcept {
    const double seconds = static_cast<double>(stepsRun) * kSecondsPerStep;
    if (seconds <= 0.0) {
        return 0.0;
    }
    return (3600.0 * static_cast<double>(ordersCompleted)) / seconds;
}

std::vector<ContinuousScenario> continuousScenarios(const std::vector<std::string>& mapNames,
                                                    const std::vector<int>& fleetSizes,
                                                    const std::vector<double>& delayRates,
                                                    int seedsPerCombination, int steps,
                                                    int arrivalInterval, int ageingInterval) {
    std::vector<ContinuousScenario> scenarios;
    for (const std::string& mapName : mapNames) {
        for (const int fleetSize : fleetSizes) {
            for (const double delay : delayRates) {
                for (int seed = 0; seed < seedsPerCombination; ++seed) {
                    scenarios.push_back(ContinuousScenario{mapName, fleetSize, delay,
                                                           static_cast<std::uint32_t>(seed), steps,
                                                           arrivalInterval, ageingInterval});
                }
            }
        }
    }
    return scenarios;
}

std::vector<core::GridCell> makeContinuousStarts(const core::Map& map, int fleetSize,
                                                 std::uint32_t seed) {
    if (fleetSize <= 0) {
        return {};
    }
    std::vector<core::GridCell> cells = map.freeCells();
    const auto count = static_cast<std::size_t>(fleetSize);
    if (cells.size() < count) {
        throw std::runtime_error("continuous benchmark: the map has too few free cells");
    }
    std::mt19937 rng(seed);
    std::shuffle(cells.begin(), cells.end(), rng);
    cells.resize(count);
    return cells;
}

ContinuousOutcome runContinuousScenario(const core::Map& map, const ContinuousScenario& scenario,
                                        bool passOrderEnabled, const std::string& plannerName) {
    ContinuousOutcome outcome;
    outcome.scenario = scenario;
    outcome.passOrderEnabled = passOrderEnabled;

    const std::vector<core::GridCell> starts =
        makeContinuousStarts(map, scenario.fleetSize, scenario.seed);
    const int orderCount = scenario.steps / std::max(1, scenario.arrivalInterval) + 32;
    const core::OrderGeneratorOptions generator{orderCount, 0, scenario.arrivalInterval, 1, 3};
    const std::vector<core::Order> orders = core::generateOrders(map, generator, scenario.seed);

    core::ContinuousConfig config;
    config.seed = scenario.seed;
    config.delayProbability = scenario.delayProbability;
    config.maxSteps = scenario.steps;
    config.priorityAgeingInterval = scenario.priorityAgeingInterval;
    config.passOrderEnabled = passOrderEnabled;

    core::NearestRobotAssigner assigner;
    core::ContinuousResult result;
    const int horizon = core::continuousHorizon(map, scenario.fleetSize);
    if (plannerName == "prioritized") {
        core::PrioritizedPlanner planner(
            core::PrioritizedOptions{core::AgentOrder::FarthestFirst, 0, horizon});
        result = core::runContinuous(map, starts, orders, planner, assigner, config);
    } else if (plannerName == "cbs") {
        core::CbsPlanner planner(core::CbsOptions{500, 0, horizon, 20000});
        core::PrioritizedPlanner fallback(
            core::PrioritizedOptions{core::AgentOrder::FarthestFirst, 0, horizon});
        result = core::runContinuous(map, starts, orders, planner, assigner, config, &fallback);
    } else {
        throw std::runtime_error("continuous benchmark: unknown planner " + plannerName);
    }

    outcome.deadlock = result.deadlock;
    outcome.allOrdersCompleted = result.allOrdersCompleted;
    outcome.stepsRun = result.stepsRun;
    outcome.ordersReleased = result.ordersReleased;
    outcome.ordersCompleted = result.ordersCompleted;
    outcome.droppedOrders = result.droppedOrders;
    outcome.pendingAtEnd = result.pendingAtEnd;
    outcome.longestWaitSteps = result.longestWaitSteps;
    outcome.longestWaitPriority = result.longestWaitPriority;
    outcome.medianWaitSteps = result.medianWaitSteps;
    outcome.percentile95WaitSteps = result.percentile95WaitSteps;
    outcome.meanWaitSteps = result.meanWaitSteps;
    outcome.waitTotalSteps = result.waitTotalSteps;
    outcome.waitCount = result.waitCount;
    outcome.collisions = result.verdict.collisions;
    outcome.swaps = result.verdict.swaps;
    outcome.planningCalls = result.planningCalls;
    outcome.planningFailures = result.planningFailures;
    outcome.fallbacks = result.fallbacks;
    outcome.fallbackTimeouts = result.fallbackTimeouts;
    return outcome;
}

std::string continuousCsvHeader() {
    return "mode,map,fleet,delay,seed,steps,deadlock,all_completed,steps_run,orders_released,"
           "orders_completed,dropped,pending,longest_wait,longest_wait_priority,median_wait,"
           "p95_wait,mean_wait,wait_count,collisions,swaps,planning_calls,planning_failures,"
           "fallbacks,fallback_timeouts";
}

std::string toContinuousCsvRow(const ContinuousOutcome& outcome) {
    std::ostringstream out;
    out << (outcome.passOrderEnabled ? "pass_order" : "control") << ',' << outcome.scenario.mapName
        << ',' << outcome.scenario.fleetSize << ','
        << formatFixed(outcome.scenario.delayProbability, 3) << ',' << outcome.scenario.seed << ','
        << outcome.scenario.steps << ',' << (outcome.deadlock ? 1 : 0) << ','
        << (outcome.allOrdersCompleted ? 1 : 0) << ',' << outcome.stepsRun << ','
        << outcome.ordersReleased << ',' << outcome.ordersCompleted << ',' << outcome.droppedOrders
        << ',' << outcome.pendingAtEnd << ',' << outcome.longestWaitSteps << ','
        << outcome.longestWaitPriority << ',' << outcome.medianWaitSteps << ','
        << outcome.percentile95WaitSteps << ',' << formatFixed(outcome.meanWaitSteps, 2) << ','
        << outcome.waitCount << ',' << outcome.collisions << ',' << outcome.swaps << ','
        << outcome.planningCalls << ',' << outcome.planningFailures << ',' << outcome.fallbacks
        << ',' << outcome.fallbackTimeouts;
    return out.str();
}

void accumulateContinuous(ContinuousTotals& totals, const ContinuousOutcome& outcome) {
    ++totals.runs;
    if (outcome.deadlock) {
        ++totals.deadlocks;
    }
    if (outcome.collisions > 0 || outcome.swaps > 0) {
        ++totals.runsWithViolations;
    }
    totals.collisionEvents += outcome.collisions;
    totals.swapEvents += outcome.swaps;
    totals.stepsRun += outcome.stepsRun;
    totals.ordersReleased += outcome.ordersReleased;
    totals.ordersCompleted += outcome.ordersCompleted;
    totals.longestWaitSteps = std::max(totals.longestWaitSteps, outcome.longestWaitSteps);
    totals.percentile95WaitSteps =
        std::max(totals.percentile95WaitSteps, outcome.percentile95WaitSteps);
    totals.waitTotal += outcome.waitTotalSteps;
    totals.waitCount += outcome.waitCount;
    totals.planningCalls += outcome.planningCalls;
    totals.planningFailures += outcome.planningFailures;
    totals.fallbacks += outcome.fallbacks;
    totals.fallbackTimeouts += outcome.fallbackTimeouts;
}

std::vector<FleetTotals> groupContinuousByDelay(const std::vector<ContinuousOutcome>& outcomes) {
    return groupByDelay(outcomes);
}

std::vector<FleetTotals> groupContinuousByFleet(const std::vector<ContinuousOutcome>& outcomes) {
    return groupByFleet(outcomes);
}

std::string formatContinuousDelayTable(const std::vector<FleetTotals>& passRows,
                                       const std::vector<FleetTotals>& controlRows) {
    std::ostringstream out;
    out << "delay-level breakdown (bad = runs with at least one collision or swap)\n";
    out << "  delay | pass:runs deadlock bad coll swap longest_wait | ctrl:runs deadlock bad coll "
           "swap\n";
    const std::size_t count = std::min(passRows.size(), controlRows.size());
    for (std::size_t index = 0; index < count; ++index) {
        const ContinuousTotals& pass = passRows[index].totals;
        const ContinuousTotals& control = controlRows[index].totals;
        out << "  " << std::fixed << std::setprecision(3) << passRows[index].delay << " | "
            << std::setw(6) << pass.runs << " " << std::setw(8) << pass.deadlocks << " "
            << std::setw(5) << pass.runsWithViolations << " " << std::setw(6)
            << pass.collisionEvents << " " << std::setw(5) << pass.swapEvents << " "
            << std::setw(12) << pass.longestWaitSteps << " | " << std::setw(6) << control.runs
            << " " << std::setw(8) << control.deadlocks << " " << std::setw(5)
            << control.runsWithViolations << " " << std::setw(6) << control.collisionEvents << " "
            << std::setw(5) << control.swapEvents << '\n';
    }
    return out.str();
}

std::string formatThroughputTable(const std::vector<FleetTotals>& noDelayRows,
                                  const std::vector<FleetTotals>& delayRows) {
    std::ostringstream out;
    out << "throughput (orders completed per 1,000 steps and per hour)\n";
    out << "  fleet | no-delay:per1000 per_hour completed | delayed:per1000 per_hour completed | "
           "wait:mean p95 longest\n";
    const std::size_t count = std::min(noDelayRows.size(), delayRows.size());
    for (std::size_t index = 0; index < count; ++index) {
        const ContinuousTotals& fast = noDelayRows[index].totals;
        const ContinuousTotals& slow = delayRows[index].totals;
        out << "  " << std::setw(5) << noDelayRows[index].fleetSize << " | " << std::setw(9)
            << std::fixed << std::setprecision(1) << fast.ordersPerThousandSteps() << " "
            << std::setw(8) << fast.ordersPerHour() << " " << std::setw(9) << fast.ordersCompleted
            << " | " << std::setw(8) << slow.ordersPerThousandSteps() << " " << std::setw(8)
            << slow.ordersPerHour() << " " << std::setw(9) << slow.ordersCompleted << " | "
            << std::setw(6) << slow.meanWaitSteps() << " " << std::setw(5)
            << slow.percentile95WaitSteps << " " << std::setw(6) << slow.longestWaitSteps << '\n';
    }
    return out.str();
}

std::string formatThroughputComparison(const std::vector<FleetTotals>& prioritizedRows,
                                       const std::vector<FleetTotals>& cbsRows) {
    std::ostringstream out;
    out << "continuous throughput comparison, no delays (orders per 1,000 steps)\n";
    out << "  fleet | prioritized:per1000 completed wait | cbs:per1000 completed wait fallbacks "
           "failures\n";
    const std::size_t count = std::min(prioritizedRows.size(), cbsRows.size());
    for (std::size_t index = 0; index < count; ++index) {
        const ContinuousTotals& base = prioritizedRows[index].totals;
        const ContinuousTotals& cbs = cbsRows[index].totals;
        out << "  " << std::setw(5) << prioritizedRows[index].fleetSize << " | " << std::setw(9)
            << std::fixed << std::setprecision(1) << base.ordersPerThousandSteps() << " "
            << std::setw(9) << base.ordersCompleted << " " << std::setw(6) << base.meanWaitSteps()
            << " | " << std::setw(8) << cbs.ordersPerThousandSteps() << " " << std::setw(9)
            << cbs.ordersCompleted << " " << std::setw(6) << cbs.meanWaitSteps() << " "
            << std::setw(9) << cbs.fallbacks << " " << std::setw(8) << cbs.planningFailures << '\n';
    }
    return out.str();
}

std::string formatThroughputByMap(const std::vector<ContinuousOutcome>& outcomes) {
    using Key = std::tuple<std::string, int>;
    std::map<Key, std::pair<ContinuousTotals, ContinuousTotals>> grouped;
    for (const ContinuousOutcome& outcome : outcomes) {
        auto& entry = grouped[Key{outcome.scenario.mapName, outcome.scenario.fleetSize}];
        if (outcome.scenario.delayProbability > 0.0) {
            accumulateContinuous(entry.second, outcome);
        } else {
            accumulateContinuous(entry.first, outcome);
        }
    }

    std::ostringstream out;
    out << "throughput by map (orders per 1,000 steps, no delay / delay)\n";
    out << "  map | fleet | no-delay per1000 | delayed per1000 | wait:mean p95 longest\n";
    for (const auto& [key, totals] : grouped) {
        const auto& [mapName, fleet] = key;
        out << "  " << std::setw(18) << std::left << shortMapName(mapName) << std::right << " | "
            << std::setw(5) << fleet << " | " << std::setw(16) << std::fixed << std::setprecision(1)
            << totals.first.ordersPerThousandSteps() << " | " << std::setw(14)
            << totals.second.ordersPerThousandSteps() << " | " << std::setw(6)
            << totals.second.meanWaitSteps() << " " << std::setw(5)
            << totals.second.percentile95WaitSteps << " " << std::setw(7)
            << totals.second.longestWaitSteps << '\n';
    }
    return out.str();
}

}  // namespace lotse::benchmark
