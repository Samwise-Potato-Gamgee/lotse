// Executes a plan step by step under random delays, enforcing the pass-order
// rule and replanning when a robot falls behind or the fleet gets stuck.

#pragma once

#include <cstdint>
#include <vector>

#include "lotse/core/map.hpp"
#include "lotse/core/plan.hpp"
#include "lotse/core/planner.hpp"
#include "lotse/judge/judge.hpp"

namespace lotse::core {

/// Tuning for one execution run: delays, replanning and stop conditions.
struct ExecutionConfig {
    std::uint32_t seed = 0;
    double delayProbability = 0.0;
    int replanBehindSlack = 8;
    int replanCooldown = 5;
    int stuckPatience = 8;
    int deadlockPatience = 300;
    int maxSteps = 5000;
    bool passOrderEnabled = true;
};

/// What one execution run produced.
struct ExecutionResult {
    bool finished = false;
    bool deadlock = false;
    bool stepLimitHit = false;
    bool initialPlanFailed = false;
    int stepsUsed = 0;
    int replans = 0;
    int replanFailures = 0;
    judge::RunLog log{};
    judge::Report verdict{};
};

/// Runs a fleet from its starts to its goals with delays and replanning.
/// @param map the warehouse grid
/// @param agents the fleet, one start and one goal per robot
/// @param planner the planner used for the first plan and for every replan
/// @param config delays, replanning thresholds and stop conditions
/// @return the trajectory, the judge verdict and the counters
[[nodiscard]] ExecutionResult execute(const Map& map, const std::vector<Agent>& agents,
                                      Planner& planner, const ExecutionConfig& config);

}  // namespace lotse::core
