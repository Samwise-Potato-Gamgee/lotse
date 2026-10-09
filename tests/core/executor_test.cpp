// Unit tests for the delay-aware executor: pass-order rule, delays and control.

#include "lotse/core/executor.hpp"

#include <cstdint>
#include <vector>

#include "lotse/core/prioritized_planner.hpp"

#include <gtest/gtest.h>

namespace lotse::core {
namespace {

constexpr const char* kBottleneck = "height 3\nwidth 5\nmap\n.....\n##.##\n.....\n";

// Two robots that must cross one single-cell opening in opposite directions.
std::vector<Agent> twoCrossingRobots() {
    return {{0, GridCell{0, 0}, GridCell{4, 2}}, {1, GridCell{4, 0}, GridCell{0, 2}}};
}

// Without delays the fleet follows the plan and finishes safely.
TEST(ExecutorTest, ExecutesPlanWithoutDelays) {
    const Map map = Map::fromText(kBottleneck);
    ExecutionConfig config;
    config.seed = 0;
    config.delayProbability = 0.0;
    PrioritizedPlanner planner;
    const ExecutionResult result = execute(map, twoCrossingRobots(), planner, config);
    EXPECT_TRUE(result.finished);
    EXPECT_FALSE(result.deadlock);
    EXPECT_FALSE(result.stepLimitHit);
    EXPECT_FALSE(result.initialPlanFailed);
    EXPECT_TRUE(result.verdict.safe());
}

// Random delays never produce a collision or a deadlock.
TEST(ExecutorTest, StaysSafeWithDelays) {
    const Map map = Map::fromText(kBottleneck);
    PrioritizedPlanner planner;
    for (std::uint32_t seed = 0; seed < 20; ++seed) {
        ExecutionConfig config;
        config.seed = seed;
        config.delayProbability = 0.3;
        const ExecutionResult result = execute(map, twoCrossingRobots(), planner, config);
        EXPECT_TRUE(result.verdict.safe()) << "seed " << seed;
        EXPECT_FALSE(result.deadlock) << "seed " << seed;
    }
}

// With the pass-order rule switched off, delays produce collisions.
TEST(ExecutorTest, ControlWithoutPassOrderCollides) {
    const Map map = Map::fromText(kBottleneck);
    PrioritizedPlanner planner;
    int violations = 0;
    for (std::uint32_t seed = 0; seed < 40; ++seed) {
        ExecutionConfig config;
        config.seed = seed;
        config.delayProbability = 0.5;
        config.passOrderEnabled = false;
        const ExecutionResult result = execute(map, twoCrossingRobots(), planner, config);
        violations += result.verdict.collisions + result.verdict.swaps;
    }
    EXPECT_GT(violations, 0);
}

}  // namespace
}  // namespace lotse::core
