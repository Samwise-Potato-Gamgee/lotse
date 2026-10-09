// Unit tests for the space-time search and prioritized planning.

#include <optional>
#include <string>
#include <vector>

#include "lotse/benchmark/safety_benchmark.hpp"
#include "lotse/core/prioritized_planner.hpp"
#include "lotse/core/space_time_search.hpp"
#include "lotse/sim/grid_simulator.hpp"

#include <gtest/gtest.h>

namespace lotse::core {
namespace {

constexpr const char* kOpenRow = "height 1\nwidth 5\nmap\n.....\n";
constexpr const char* kBottleneck = "height 3\nwidth 5\nmap\n.....\n##.##\n.....\n";

const SearchLimits kLimits{40, 200000};

// Finds the shortest route on an empty row.
TEST(SpaceTimeSearchTest, FindsShortestRoute) {
    const Map map = Map::fromText(kOpenRow);
    const Agent agent{0, GridCell{0, 0}, GridCell{4, 0}};
    const Reservations reserved;
    const std::optional<Route> route = searchRoute(map, agent, reserved, kLimits);
    ASSERT_TRUE(route.has_value());
    const Route& found = route.value();
    EXPECT_EQ(found.steps(), 4);
    EXPECT_EQ(found.cellAt(0), (GridCell{0, 0}));
    EXPECT_EQ(found.cellAt(4), (GridCell{4, 0}));
}

// Avoids a reserved move by waiting one step.
TEST(SpaceTimeSearchTest, AvoidsReservedMove) {
    const Map map = Map::fromText(kOpenRow);
    const Agent agent{0, GridCell{0, 0}, GridCell{2, 0}};
    Reservations reserved;
    reserved.reserveMove(GridCell{0, 0}, GridCell{1, 0}, 0);
    const std::optional<Route> route = searchRoute(map, agent, reserved, kLimits);
    ASSERT_TRUE(route.has_value());
    EXPECT_EQ(route.value().steps(), 3);
}

// Avoids a reserved cell by arriving later.
TEST(SpaceTimeSearchTest, AvoidsReservedCell) {
    const Map map = Map::fromText(kOpenRow);
    const Agent agent{0, GridCell{0, 0}, GridCell{3, 0}};
    Reservations reserved;
    reserved.reserveCell(GridCell{2, 0}, 2);
    const std::optional<Route> route = searchRoute(map, agent, reserved, kLimits);
    ASSERT_TRUE(route.has_value());
    const Route& found = route.value();
    EXPECT_GE(found.steps(), 4);
    EXPECT_NE(found.cellAt(2), (GridCell{2, 0}));
}

// Returns nothing when the goal is unreachable.
TEST(SpaceTimeSearchTest, FailsOnBlockedGoal) {
    const Map map = Map::fromText("height 1\nwidth 3\nmap\n.#.\n");
    const Agent agent{0, GridCell{0, 0}, GridCell{1, 0}};
    const Reservations reserved;
    EXPECT_FALSE(searchRoute(map, agent, reserved, kLimits).has_value());
}

// Plans two robots through one bottleneck without conflict.
TEST(PrioritizedPlannerTest, PlansThroughBottleneck) {
    const Map map = Map::fromText(kBottleneck);
    const std::vector<Agent> agents{{0, GridCell{0, 0}, GridCell{4, 2}},
                                    {1, GridCell{4, 0}, GridCell{0, 2}}};
    PrioritizedPlanner planner;
    const std::optional<Plan> plan = planner.plan(map, agents);
    ASSERT_TRUE(plan.has_value());
    const Plan& planned = plan.value();
    EXPECT_EQ(planned.agentCount(), 2);
    EXPECT_EQ(planner.name(), "prioritized");

    const judge::RunLog log = sim::followPlanExactly(planned, planned.makespan());
    EXPECT_TRUE(judge::analyse(log).safe());
}

// Reports failure when a robot's goal is an obstacle.
TEST(PrioritizedPlannerTest, FailsWhenGoalIsBlocked) {
    const Map map = Map::fromText("height 1\nwidth 3\nmap\n.#.\n");
    const std::vector<Agent> agents{{0, GridCell{0, 0}, GridCell{1, 0}}};
    PrioritizedPlanner planner;
    EXPECT_FALSE(planner.plan(map, agents).has_value());
}

// Blocks a route through a cell that is reserved at every plan step.
TEST(PrioritizedPlannerTest, BlocksForeverReservedCell) {
    const Map map = Map::fromText(kOpenRow);
    const Agent agent{0, GridCell{0, 0}, GridCell{4, 0}};
    Reservations reserved;
    for (int time = 0; time <= kLimits.horizon; ++time) {
        reserved.reserveCell(GridCell{2, 0}, time);
    }
    EXPECT_FALSE(searchRoute(map, agent, reserved, kLimits).has_value());
}

// Seeded retries solve the one stage 2 instance a single robot order misses.
TEST(PrioritizedPlannerTest, RetriesSolveTheHardWarehouseInstance) {
    const std::optional<Map> map =
        Map::fromFile(std::string(LOTSE_SOURCE_DIR) + "/maps/warehouse_aisles.map");
    ASSERT_TRUE(map.has_value());
    const std::optional<std::vector<Agent>> fleet = benchmark::makeFleet(*map, 20, 4);
    ASSERT_TRUE(fleet.has_value());

    PrioritizedPlanner single{PrioritizedOptions{AgentOrder::FarthestFirst, 0, 0, 1, 0}};
    EXPECT_FALSE(single.plan(*map, *fleet).has_value());

    PrioritizedPlanner retrying{PrioritizedOptions{AgentOrder::FarthestFirst, 0, 0, 8, 0}};
    const std::optional<Plan> plan = retrying.plan(*map, *fleet);
    ASSERT_TRUE(plan.has_value());
    const judge::RunLog log = sim::followPlanExactly(*plan, plan->makespan());
    EXPECT_TRUE(judge::analyse(log).safe());
}

}  // namespace
}  // namespace lotse::core
