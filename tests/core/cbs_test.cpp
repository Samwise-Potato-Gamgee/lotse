// Unit tests for conflict-based search: safety, optimality on small instances
// and the timeout path.

#include "lotse/core/cbs.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <vector>

#include "lotse/core/space_time_search.hpp"
#include "lotse/sim/grid_simulator.hpp"

#include <gtest/gtest.h>

namespace lotse::core {
namespace {

constexpr const char* kOpen3 =
    "height 3\nwidth 3\nmap\n"
    "...\n"
    "...\n"
    "...\n";

/// Returns the sum of the route lengths of a plan.
/// @param plan the plan to score
[[nodiscard]] int planCost(const Plan& plan) {
    int cost = 0;
    for (const Route& route : plan.routes) {
        cost += route.steps();
    }
    return cost;
}

/// Enumerates every shortest route for one agent by walking the distance field.
/// @param map the warehouse grid
/// @param agent the robot
/// @param distance the BFS distance from the start
/// @param current the cell being extended
/// @param route the route built so far
/// @param routes the collected shortest routes
void collectRoutes(const Map& map, const Agent& agent,
                   const std::vector<std::vector<int>>& distance, GridCell current,
                   std::vector<GridCell>& route, std::vector<std::vector<GridCell>>& routes) {
    if (current == agent.goal) {
        routes.push_back(route);
        return;
    }
    constexpr std::array<GridCell, 4> kMoves{{{0, 1}, {1, 0}, {0, -1}, {-1, 0}}};
    const int here =
        distance[static_cast<std::size_t>(current.y)][static_cast<std::size_t>(current.x)];
    for (const GridCell& move : kMoves) {
        const GridCell next{current.x + move.x, current.y + move.y};
        if (!map.isFree(next)) {
            continue;
        }
        if (distance[static_cast<std::size_t>(next.y)][static_cast<std::size_t>(next.x)] !=
            here + 1) {
            continue;
        }
        route.push_back(next);
        collectRoutes(map, agent, distance, next, route, routes);
        route.pop_back();
    }
}

/// Lists every shortest route for an agent on a small map.
/// @param map the warehouse grid
/// @param agent the robot
[[nodiscard]] std::vector<std::vector<GridCell>> shortestRoutes(const Map& map,
                                                                const Agent& agent) {
    std::vector<std::vector<int>> distance(
        static_cast<std::size_t>(map.height()),
        std::vector<int>(static_cast<std::size_t>(map.width()), -1));
    distance[static_cast<std::size_t>(agent.start.y)][static_cast<std::size_t>(agent.start.x)] = 0;
    std::vector<GridCell> frontier{agent.start};
    constexpr std::array<GridCell, 4> kMoves{{{0, 1}, {1, 0}, {0, -1}, {-1, 0}}};
    while (!frontier.empty()) {
        const GridCell cell = frontier.back();
        frontier.pop_back();
        for (const GridCell& move : kMoves) {
            const GridCell next{cell.x + move.x, cell.y + move.y};
            if (!map.isFree(next) ||
                distance[static_cast<std::size_t>(next.y)][static_cast<std::size_t>(next.x)] >= 0) {
                continue;
            }
            distance[static_cast<std::size_t>(next.y)][static_cast<std::size_t>(next.x)] =
                distance[static_cast<std::size_t>(cell.y)][static_cast<std::size_t>(cell.x)] + 1;
            frontier.push_back(next);
        }
    }

    std::vector<std::vector<GridCell>> routes;
    std::vector<GridCell> route{agent.start};
    if (!map.isFree(agent.goal)) {
        return routes;
    }
    collectRoutes(map, agent, distance, agent.start, route, routes);
    return routes;
}

/// Returns the best sum of route lengths when every agent takes a shortest
/// route and no combination conflicts, or -1 when that is impossible.
/// @param map the warehouse grid
/// @param agents the fleet
[[nodiscard]] int bestShortestCombination(const Map& map, const std::vector<Agent>& agents) {
    std::vector<std::vector<std::vector<GridCell>>> options;
    for (const Agent& agent : agents) {
        options.push_back(shortestRoutes(map, agent));
        if (options.back().empty()) {
            return -1;
        }
    }

    int best = -1;
    std::vector<std::size_t> choice(agents.size(), 0);
    while (true) {
        int horizon = 0;
        for (std::size_t agent = 0; agent < agents.size(); ++agent) {
            horizon = std::max(horizon, static_cast<int>(options[agent][choice[agent]].size()) - 1);
        }
        bool conflict = false;
        for (std::size_t a = 0; a < agents.size() && !conflict; ++a) {
            for (std::size_t b = a + 1; b < agents.size() && !conflict; ++b) {
                const std::vector<GridCell>& first = options[a][choice[a]];
                const std::vector<GridCell>& second = options[b][choice[b]];
                for (int time = 0; time <= horizon && !conflict; ++time) {
                    const GridCell firstCell = first[std::min<std::size_t>(
                        static_cast<std::size_t>(time), first.size() - 1)];
                    const GridCell secondCell = second[std::min<std::size_t>(
                        static_cast<std::size_t>(time), second.size() - 1)];
                    if (firstCell == secondCell) {
                        conflict = true;
                        break;
                    }
                    if (time < horizon) {
                        const GridCell nextFirst = first[std::min<std::size_t>(
                            static_cast<std::size_t>(time) + 1, first.size() - 1)];
                        const GridCell nextSecond = second[std::min<std::size_t>(
                            static_cast<std::size_t>(time) + 1, second.size() - 1)];
                        if (firstCell == nextSecond && secondCell == nextFirst &&
                            firstCell != nextFirst) {
                            conflict = true;
                        }
                    }
                }
            }
        }
        if (!conflict) {
            int cost = 0;
            for (std::size_t agent = 0; agent < agents.size(); ++agent) {
                cost += static_cast<int>(options[agent][choice[agent]].size()) - 1;
            }
            if (best < 0 || cost < best) {
                best = cost;
            }
        }

        std::size_t index = agents.size();
        while (index > 0) {
            --index;
            ++choice[index];
            if (choice[index] < options[index].size()) {
                break;
            }
            choice[index] = 0;
            if (index == 0) {
                return best;
            }
        }
    }
}

// CBS matches the best all-shortest-route cost when one exists.
TEST(CbsPlannerTest, FindsOptimalPlanOnSmallInstance) {
    const Map map = Map::fromText(kOpen3);
    const std::vector<Agent> agents{{0, GridCell{0, 0}, GridCell{2, 2}},
                                    {1, GridCell{2, 0}, GridCell{0, 2}}};
    CbsPlanner planner;
    const std::optional<Plan> plan = planner.plan(map, agents);
    ASSERT_TRUE(plan.has_value());
    EXPECT_EQ(planner.name(), "cbs");
    EXPECT_FALSE(planner.lastCallTimedOut());

    const int best = bestShortestCombination(map, agents);
    ASSERT_GE(best, 0);
    EXPECT_EQ(planCost(plan.value()), best);

    const judge::RunLog log = sim::followPlanExactly(plan.value(), plan.value().makespan());
    EXPECT_TRUE(judge::analyse(log).safe());
}

// A head-on pair must wait for one another and stays safe.
TEST(CbsPlannerTest, ResolvesSwapConflict) {
    const Map map = Map::fromText("height 2\nwidth 3\nmap\n...\n...\n");
    const std::vector<Agent> agents{{0, GridCell{0, 0}, GridCell{2, 1}},
                                    {1, GridCell{2, 0}, GridCell{0, 1}}};
    CbsPlanner planner;
    const std::optional<Plan> plan = planner.plan(map, agents);
    ASSERT_TRUE(plan.has_value());
    const int lowerBound = manhattan_distance(agents[0].start, agents[0].goal) +
                           manhattan_distance(agents[1].start, agents[1].goal);
    EXPECT_GE(planCost(plan.value()), lowerBound);

    const judge::RunLog log = sim::followPlanExactly(plan.value(), plan.value().makespan());
    EXPECT_TRUE(judge::analyse(log).safe());
}

// The cost never beats the sum of the shortest individual routes.
TEST(CbsPlannerTest, NeverBeatsTheIndividualLowerBound) {
    const Map map = Map::fromText(kOpen3);
    const std::vector<Agent> agents{{0, GridCell{0, 0}, GridCell{2, 2}},
                                    {1, GridCell{2, 2}, GridCell{0, 0}},
                                    {2, GridCell{2, 0}, GridCell{0, 2}}};
    CbsPlanner planner(CbsOptions{2000, 0, 0, 100000});
    const std::optional<Plan> plan = planner.plan(map, agents);
    ASSERT_TRUE(plan.has_value());
    const int lowerBound = manhattan_distance(agents[0].start, agents[0].goal) +
                           manhattan_distance(agents[1].start, agents[1].goal) +
                           manhattan_distance(agents[2].start, agents[2].goal);
    EXPECT_GE(planCost(plan.value()), lowerBound);
    const judge::RunLog log = sim::followPlanExactly(plan.value(), plan.value().makespan());
    EXPECT_TRUE(judge::analyse(log).safe());
}

// Two agents with the same goal are infeasible and reported as such.
TEST(CbsPlannerTest, RejectsDuplicateGoals) {
    const Map map = Map::fromText(kOpen3);
    const std::vector<Agent> agents{{0, GridCell{0, 0}, GridCell{2, 2}},
                                    {1, GridCell{2, 0}, GridCell{2, 2}}};
    CbsPlanner planner;
    EXPECT_FALSE(planner.plan(map, agents).has_value());
}

// An empty fleet plans trivially.
TEST(CbsPlannerTest, PlansAnEmptyFleet) {
    const Map map = Map::fromText(kOpen3);
    CbsPlanner planner;
    const std::optional<Plan> plan = planner.plan(map, {});
    ASSERT_TRUE(plan.has_value());
    EXPECT_EQ(plan->agentCount(), 0);
}

}  // namespace
}  // namespace lotse::core
