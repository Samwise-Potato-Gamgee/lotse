// Continuous operation: a stream of orders arrives, free robots are assigned,
// each robot does pickup then drop-off then parks, and the fleet is replanned
// over a short moving window under the pass-order rule.

#pragma once

#include <cstdint>
#include <vector>

#include "lotse/core/map.hpp"
#include "lotse/core/order.hpp"
#include "lotse/core/planner.hpp"
#include "lotse/core/task_assigner.hpp"
#include "lotse/judge/judge.hpp"

namespace lotse::core {

/// Tuning for one continuous run.
struct ContinuousConfig {
    std::uint32_t seed = 0;
    double delayProbability = 0.0;
    int maxSteps = 2000;
    int deadlockPatience = 300;
    int stuckPatience = 8;
    int replanCooldown = 1;
    int windowSteps = 40;
    int priorityAgeingInterval = 25;
    bool passOrderEnabled = true;
};

/// What one continuous run produced.
struct ContinuousResult {
    bool deadlock = false;
    bool allOrdersCompleted = false;
    int stepsRun = 0;
    int ordersReleased = 0;
    int ordersCompleted = 0;
    int droppedOrders = 0;
    int pendingAtEnd = 0;
    int longestWaitSteps = 0;
    int longestWaitPriority = 0;
    int longestWaitOrderId = -1;
    int medianWaitSteps = 0;
    int percentile95WaitSteps = 0;
    double meanWaitSteps = 0.0;
    long long waitTotalSteps = 0;
    int waitCount = 0;
    int planningCalls = 0;
    int planningFailures = 0;
    int fallbacks = 0;
    int fallbackTimeouts = 0;
    judge::RunLog log{};
    judge::Report verdict{};
};

/// Runs a continuous order stream for one fleet.
/// @param map the warehouse grid; must have a parking cell per robot
/// @param robotStarts the start cell of every robot, indexed by robot id
/// @param orders the order stream, sorted by arrival step
/// @param planner the primary fleet planner
/// @param assigner the task assigner
/// @param config delays, horizon, window and replanning tuning
/// @param fallback optional second planner used when the primary fails
[[nodiscard]] ContinuousResult runContinuous(const Map& map,
                                             const std::vector<GridCell>& robotStarts,
                                             const std::vector<Order>& orders, Planner& planner,
                                             TaskAssigner& assigner, const ContinuousConfig& config,
                                             Planner* fallback = nullptr);

}  // namespace lotse::core
