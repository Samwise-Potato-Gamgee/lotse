// Unit tests for continuous operation: order completion, safety under delays,
// the control run, and idle robots parking.

#include "lotse/core/continuous_runner.hpp"

#include <cstdint>
#include <optional>
#include <vector>

#include "lotse/core/order.hpp"
#include "lotse/core/prioritized_planner.hpp"
#include "lotse/core/task_assigner.hpp"

#include <gtest/gtest.h>

namespace lotse::core {
namespace {

constexpr const char* kLoop =
    "name loop\n"
    "height 3\nwidth 9\nmap\n"
    ".........\n"
    ".........\n"
    ".........\n"
    "stations\n"
    "pickup0 0 0\n"
    "pickup1 1 0\n"
    "dropoff0 8 0\n"
    "dropoff1 7 0\n"
    "parking\n"
    "0 2\n"
    "1 2\n"
    "2 2\n"
    "3 2\n"
    "4 2\n"
    "5 2\n"
    "6 2\n"
    "7 2\n"
    "8 2\n";

constexpr const char* kBottleneck =
    "name bottleneck\n"
    "height 5\nwidth 5\nmap\n"
    ".....\n"
    "##.##\n"
    ".....\n"
    ".....\n"
    ".....\n"
    "stations\n"
    "pickup0 0 0\n"
    "pickup1 1 0\n"
    "dropoff0 4 0\n"
    "dropoff1 3 0\n"
    "parking\n"
    "0 4\n"
    "1 4\n"
    "2 4\n"
    "3 4\n"
    "4 4\n";

/// Runs one continuous scenario and returns the result.
/// @param map the warehouse grid
/// @param starts the robot start cells
/// @param orders the order stream
/// @param delay per-step delay probability
/// @param seed random seed
/// @param passOrder whether the pass-order rule is on
[[nodiscard]] ContinuousResult run(const Map& map, const std::vector<GridCell>& starts,
                                   const std::vector<Order>& orders, double delay,
                                   std::uint32_t seed, bool passOrder = true) {
    const int horizon = continuousHorizon(map, static_cast<int>(starts.size()));
    PrioritizedPlanner planner{PrioritizedOptions{AgentOrder::FarthestFirst, 0, horizon}};
    NearestRobotAssigner assigner;
    ContinuousConfig config;
    config.seed = seed;
    config.delayProbability = delay;
    config.maxSteps = 400;
    config.passOrderEnabled = passOrder;
    return runContinuous(map, starts, orders, planner, assigner, config);
}

// A small order stream is delivered without delays and without violations.
TEST(ContinuousRunnerTest, CompletesOrdersWithoutDelays) {
    const Map map = Map::fromText(kLoop);
    const std::vector<GridCell> starts{{0, 2}, {1, 2}};
    const std::vector<Order> orders{{0, GridCell{0, 0}, GridCell{8, 0}, 0, 0},
                                    {1, GridCell{1, 0}, GridCell{7, 0}, 0, 0},
                                    {2, GridCell{0, 0}, GridCell{7, 0}, 0, 0},
                                    {3, GridCell{1, 0}, GridCell{8, 0}, 0, 0}};
    const ContinuousResult result = run(map, starts, orders, 0.0, 0);
    EXPECT_TRUE(result.allOrdersCompleted);
    EXPECT_FALSE(result.deadlock);
    EXPECT_EQ(result.ordersCompleted, 4);
    EXPECT_EQ(result.droppedOrders, 0);
    EXPECT_TRUE(result.verdict.safe());
}

// Random delays never produce a collision or a deadlock.
TEST(ContinuousRunnerTest, StaysSafeWithDelays) {
    const Map map = Map::fromText(kLoop);
    const std::vector<GridCell> starts{{0, 2}, {1, 2}};
    const std::vector<Order> orders{{0, GridCell{0, 0}, GridCell{8, 0}, 0, 0},
                                    {1, GridCell{1, 0}, GridCell{7, 0}, 1, 0},
                                    {2, GridCell{0, 0}, GridCell{7, 0}, 2, 0},
                                    {3, GridCell{1, 0}, GridCell{8, 0}, 3, 0},
                                    {4, GridCell{0, 0}, GridCell{8, 0}, 4, 0}};
    for (std::uint32_t seed = 0; seed < 20; ++seed) {
        const ContinuousResult result = run(map, starts, orders, 0.3, seed);
        EXPECT_TRUE(result.verdict.safe()) << "seed " << seed;
        EXPECT_FALSE(result.deadlock) << "seed " << seed;
    }
}

// With the pass-order rule switched off, delays produce collisions.
TEST(ContinuousRunnerTest, ControlWithoutPassOrderCollides) {
    const Map map = Map::fromText(kBottleneck);
    const std::vector<GridCell> starts{{0, 4}, {1, 4}};
    const std::vector<Order> orders{{0, GridCell{0, 0}, GridCell{4, 0}, 0, 0},
                                    {1, GridCell{1, 0}, GridCell{3, 0}, 0, 0},
                                    {2, GridCell{0, 0}, GridCell{3, 0}, 0, 0},
                                    {3, GridCell{1, 0}, GridCell{4, 0}, 0, 0}};
    int violations = 0;
    for (std::uint32_t seed = 0; seed < 40; ++seed) {
        const ContinuousResult result = run(map, starts, orders, 0.6, seed, false);
        violations += result.verdict.collisions + result.verdict.swaps;
    }
    EXPECT_GT(violations, 0);
}

// A robot with no order drives to its parking cell and stops there.
TEST(ContinuousRunnerTest, IdleRobotsEndParked) {
    const Map map = Map::fromText(kLoop);
    const std::vector<GridCell> starts{{4, 0}, {4, 0}};
    const std::vector<Order> orders;
    const ContinuousResult result = run(map, starts, orders, 0.0, 0);
    ASSERT_FALSE(result.log.frames.empty());
    const judge::Frame& last = result.log.frames.back();
    ASSERT_EQ(last.size(), 2U);
    EXPECT_EQ(last[0], map.parkingCells().at(0));
    EXPECT_EQ(last[1], map.parkingCells().at(1));
}

}  // namespace
}  // namespace lotse::core
