// Implements the liveness benchmark declared in liveness_benchmark.hpp.

#include "lotse/benchmark/liveness_benchmark.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "lotse/benchmark/continuous_benchmark.hpp"
#include "lotse/core/clock.hpp"
#include "lotse/core/continuous_runner.hpp"
#include "lotse/core/fleet_manager.hpp"
#include "lotse/core/order.hpp"
#include "lotse/core/prioritized_planner.hpp"
#include "lotse/core/task_assigner.hpp"
#include "lotse/sim/grid_simulator.hpp"

namespace lotse::benchmark {
namespace {

/// Describes which orders are open and where every robot is waiting.
/// @param manager the stalled manager
[[nodiscard]] std::string describeStall(const core::FleetManager& manager) {
    std::ostringstream out;
    int pending = 0;
    int assigned = 0;
    int inProgress = 0;
    int completed = 0;
    for (const core::ManagedOrder& order : manager.orders()) {
        if (order.state == core::OrderState::Pending) {
            ++pending;
        } else if (order.state == core::OrderState::Assigned) {
            ++assigned;
        } else if (order.state == core::OrderState::InProgress) {
            ++inProgress;
        } else if (order.state == core::OrderState::Completed) {
            ++completed;
        }
    }
    const core::ThroughputStats stats = manager.throughput();
    out << "pending=" << pending << " assigned=" << assigned << " in_progress=" << inProgress
        << " completed=" << completed << " planning_calls=" << stats.planningCalls
        << " planning_failures=" << stats.planningFailures << " adoptions=" << stats.planAdoptions
        << " rejections=" << stats.planRejections << ';';
    for (const core::RobotStatus& status : manager.robotStatuses()) {
        out << " r" << status.id << " at(" << status.cell.x << ',' << status.cell.y << ") move("
            << status.target.x << ',' << status.target.y << ") p=" << status.progress << ';';
    }
    out << " routes: " << manager.routeDump();
    return out.str();
}

}  // namespace

LivenessOutcome runLivenessScenario(const core::Map& map, const LivenessScenario& scenario,
                                    bool oneStepGap) {
    LivenessOutcome outcome;
    outcome.scenario = scenario;
    outcome.oneStepGap = oneStepGap;
    outcome.ordersTotal = scenario.orderCount;

    std::vector<core::GridCell> starts;
    if (static_cast<int>(map.parkingCells().size()) >= scenario.fleetSize) {
        starts.assign(map.parkingCells().begin(), map.parkingCells().begin() + scenario.fleetSize);
    } else {
        starts = makeContinuousStarts(map, scenario.fleetSize, scenario.seed);
    }
    sim::GridRobotSideConfig sim;
    sim.cellSeconds = scenario.cellSeconds;
    sim.seed = scenario.seed;
    sim.slowdownProbability = scenario.slowdownProbability;
    sim.minSpeedFactor = 0.4;
    sim.slowestSpeedFactor = 1.0;
    sim.stopProbability = scenario.stopProbability;
    sim.maxStopSeconds = scenario.maxStopSeconds;
    sim::GridRobotSide side(starts, sim);

    core::ManualClock clock;
    core::NearestRobotAssigner assigner;
    core::PrioritizedPlanner planner(core::PrioritizedOptions{
        core::AgentOrder::FarthestFirst, 0, core::continuousHorizon(map, scenario.fleetSize), 8,
        500, oneStepGap});
    core::FleetManagerConfig config;
    config.windowSeconds = scenario.windowSeconds;
    config.priorityAgeingSeconds = scenario.priorityAgeingSeconds;
    config.deadlockPatienceSeconds = scenario.deadlockPatienceSeconds;
    config.backgroundPlanning = scenario.backgroundPlanning;
    config.passOrderEnabled = true;
    config.diagnostics = scenario.collectDiagnostics;
    core::FleetManager manager(map, side, planner, assigner, clock, config);

    const core::OrderGeneratorOptions generator{scenario.orderCount, 0,
                                                scenario.arrivalIntervalTicks, 0, 3};
    const std::vector<core::Order> orders = core::generateOrders(map, generator, scenario.seed);

    if (map.parkingCells().size() >= static_cast<std::size_t>(scenario.fleetSize)) {
        std::vector<core::Order> baselineOrders = orders;
        for (core::Order& order : baselineOrders) {
            const double arrivalSeconds =
                static_cast<double>(order.arrivalStep) * scenario.tickSeconds;
            order.arrivalStep =
                static_cast<int>(std::llround(arrivalSeconds / scenario.cellSeconds));
        }
        core::ContinuousConfig baselineConfig;
        baselineConfig.seed = scenario.seed;
        baselineConfig.maxSteps = 200000;
        baselineConfig.passOrderEnabled = true;
        core::NearestRobotAssigner baselineAssigner;
        core::PrioritizedPlanner baselinePlanner(core::PrioritizedOptions{
            core::AgentOrder::FarthestFirst, 0, core::continuousHorizon(map, scenario.fleetSize)});
        const core::ContinuousResult baseline = core::runContinuous(
            map, starts, baselineOrders, baselinePlanner, baselineAssigner, baselineConfig);
        outcome.baselineSeconds = static_cast<double>(baseline.stepsRun) * scenario.cellSeconds;
    }

    double horizon = scenario.horizonSeconds;
    if (outcome.baselineSeconds > 0.0) {
        horizon = std::max(horizon, 3.0 * outcome.baselineSeconds + scenario.tickSeconds);
    }
    const auto maxTicks = static_cast<long long>(std::ceil(horizon / scenario.tickSeconds));
    std::size_t nextOrder = 0;
    for (long long tick = 0; tick < maxTicks; ++tick) {
        clock.advance(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(scenario.tickSeconds)));
        while (nextOrder < orders.size() && orders[nextOrder].arrivalStep <= tick) {
            const core::Order& order = orders[nextOrder];
            manager.submitOrder(order.pickup, order.dropoff, order.priority);
            ++nextOrder;
        }
        manager.tick(scenario.tickSeconds);
        outcome.ticks = tick + 1;
        outcome.seconds = static_cast<double>(tick + 1) * scenario.tickSeconds;
        const core::ThroughputStats stats = manager.throughput();
        outcome.ordersCompleted = stats.ordersCompleted;
        if (stats.ordersCompleted >= scenario.orderCount) {
            outcome.completed = true;
            break;
        }
        if (manager.deadlocked()) {
            break;
        }
    }

    const judge::Report report = manager.safetyReport();
    outcome.collisions = report.collisions;
    outcome.swaps = report.swaps;
    const core::ThroughputStats finalStats = manager.throughput();
    outcome.planningCalls = finalStats.planningCalls;
    outcome.planningFailures = finalStats.planningFailures;
    outcome.planAdoptions = finalStats.planAdoptions;
    outcome.planRejections = finalStats.planRejections;
    outcome.deadlocked = manager.deadlocked();
    outcome.stalled = !outcome.completed;
    outcome.diagnostics = manager.diagnostics();
    outcome.ratio = outcome.baselineSeconds > 0.0 ? outcome.seconds / outcome.baselineSeconds : 0.0;
    outcome.slow = !outcome.completed || (outcome.baselineSeconds > 0.0 &&
                                          outcome.seconds > 3.0 * outcome.baselineSeconds);
    if (outcome.stalled) {
        outcome.detail = describeStall(manager);
    }
    return outcome;
}

std::vector<LivenessScenario> livenessScenarios(const std::vector<std::string>& mapNames,
                                                const std::vector<int>& fleetSizes,
                                                const std::vector<double>& cellSeconds,
                                                int seedsPerCombination) {
    std::vector<LivenessScenario> scenarios;
    for (const std::string& mapName : mapNames) {
        for (const int fleetSize : fleetSizes) {
            for (const double crossing : cellSeconds) {
                for (int seed = 0; seed < seedsPerCombination; ++seed) {
                    LivenessScenario scenario;
                    scenario.mapName = mapName;
                    scenario.fleetSize = fleetSize;
                    scenario.cellSeconds = crossing;
                    scenario.seed = static_cast<std::uint32_t>(seed);
                    scenarios.push_back(std::move(scenario));
                }
            }
        }
    }
    return scenarios;
}

std::string livenessCsvHeader() {
    return "map,fleet,cell_seconds,slowdown,stop,seed,one_step_gap,orders_completed,orders_total,"
           "completed,stalled,slow,deadlocked,collisions,swaps,ticks,seconds,baseline_seconds,"
           "ratio,planning_calls,planning_failures,plan_adoptions,plan_rejections";
}

std::string toLivenessCsvRow(const LivenessOutcome& outcome) {
    std::ostringstream out;
    out << outcome.scenario.mapName << ',' << outcome.scenario.fleetSize << ','
        << outcome.scenario.cellSeconds << ',' << outcome.scenario.slowdownProbability << ','
        << outcome.scenario.stopProbability << ',' << outcome.scenario.seed << ','
        << (outcome.oneStepGap ? 1 : 0) << ',' << outcome.ordersCompleted << ','
        << outcome.ordersTotal << ',' << (outcome.completed ? 1 : 0) << ','
        << (outcome.stalled ? 1 : 0) << ',' << (outcome.slow ? 1 : 0) << ','
        << (outcome.deadlocked ? 1 : 0) << ',' << outcome.collisions << ',' << outcome.swaps << ','
        << outcome.ticks << ',' << outcome.seconds << ',' << outcome.baselineSeconds << ','
        << outcome.ratio << ',' << outcome.planningCalls << ',' << outcome.planningFailures << ','
        << outcome.planAdoptions << ',' << outcome.planRejections;
    return out.str();
}

void accumulateLiveness(LivenessTotals& totals, const LivenessOutcome& outcome) {
    ++totals.runs;
    if (outcome.completed) {
        ++totals.completedRuns;
    }
    if (outcome.stalled) {
        ++totals.stalledRuns;
    }
    if (outcome.slow) {
        ++totals.slowRuns;
    }
    if (outcome.collisions > 0) {
        ++totals.collisionRuns;
    }
    if (outcome.swaps > 0) {
        ++totals.swapRuns;
    }
    totals.ordersCompleted += outcome.ordersCompleted;
    totals.ordersTotal += outcome.ordersTotal;
}

}  // namespace lotse::benchmark
