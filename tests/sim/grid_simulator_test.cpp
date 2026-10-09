// Unit tests for the independent judge and the exact grid simulator.

#include "lotse/sim/grid_simulator.hpp"

#include <optional>
#include <vector>

#include "lotse/core/prioritized_planner.hpp"
#include "lotse/judge/judge.hpp"

#include <gtest/gtest.h>

namespace lotse {
namespace {

// Detects two robots in the same cell.
TEST(JudgeTest, DetectsCollision) {
    judge::RunLog log;
    log.agentCount = 2;
    log.frames.push_back(judge::Frame{{0, 0}, {1, 0}});
    log.frames.push_back(judge::Frame{{1, 0}, {1, 0}});
    const judge::Report report = judge::analyse(log);
    EXPECT_EQ(report.collisions, 1);
    EXPECT_EQ(report.swaps, 0);
    EXPECT_FALSE(report.safe());
}

// Detects two robots that exchange cells in one step.
TEST(JudgeTest, DetectsSwap) {
    judge::RunLog log;
    log.agentCount = 2;
    log.frames.push_back(judge::Frame{{0, 0}, {1, 0}});
    log.frames.push_back(judge::Frame{{1, 0}, {0, 0}});
    const judge::Report report = judge::analyse(log);
    EXPECT_EQ(report.collisions, 0);
    EXPECT_EQ(report.swaps, 1);
    EXPECT_FALSE(report.safe());
}

// Accepts a trajectory with no violation.
TEST(JudgeTest, AcceptsSafeTrajectory) {
    judge::RunLog log;
    log.agentCount = 2;
    log.frames.push_back(judge::Frame{{0, 0}, {0, 1}});
    log.frames.push_back(judge::Frame{{1, 0}, {1, 1}});
    const judge::Report report = judge::analyse(log);
    EXPECT_TRUE(report.safe());
}

// The exact simulator turns a conflict-free plan into a safe trajectory.
TEST(GridSimulatorTest, FollowsPlanSafely) {
    const core::Map map = core::Map::fromText("height 3\nwidth 5\nmap\n.....\n##.##\n.....\n");
    const std::vector<core::Agent> agents{{0, core::GridCell{0, 0}, core::GridCell{4, 2}},
                                          {1, core::GridCell{4, 0}, core::GridCell{0, 2}}};
    core::PrioritizedPlanner planner;
    const std::optional<core::Plan> plan = planner.plan(map, agents);
    ASSERT_TRUE(plan.has_value());
    const core::Plan& planned = plan.value();
    const judge::RunLog log = sim::followPlanExactly(planned, planned.makespan());
    EXPECT_TRUE(judge::analyse(log).safe());
    EXPECT_EQ(log.agentCount, 2);
    EXPECT_EQ(log.frames.size(), static_cast<std::size_t>(planned.makespan()) + 1U);
}

}  // namespace
}  // namespace lotse
