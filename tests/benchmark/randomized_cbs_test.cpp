// Randomized tests for CBS: it is at least as cheap as prioritized planning and
// its plans are safe, and the continuous runner falls back and counts it.

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "lotse/benchmark/safety_benchmark.hpp"
#include "lotse/core/cbs.hpp"
#include "lotse/core/continuous_runner.hpp"
#include "lotse/core/map.hpp"
#include "lotse/core/order.hpp"
#include "lotse/core/prioritized_planner.hpp"
#include "lotse/core/task_assigner.hpp"
#include "lotse/sim/grid_simulator.hpp"

#include <gtest/gtest.h>

namespace lotse {
namespace {

constexpr const char* kWarehouse =
    "name test_warehouse\n"
    "height 10\nwidth 12\nmap\n"
    "............\n"
    ".#.#.#.#.#..\n"
    ".#.#.#.#.#..\n"
    "............\n"
    ".#.#.#.#.#..\n"
    ".#.#.#.#.#..\n"
    "............\n"
    ".#.#.#.#.#..\n"
    ".#.#.#.#.#..\n"
    "............\n";

/// Returns the sum of the route lengths of a plan.
/// @param plan the plan to score
[[nodiscard]] int planCost(const core::Plan& plan) {
    int cost = 0;
    for (const core::Route& route : plan.routes) {
        cost += route.steps();
    }
    return cost;
}

/// A planner that always fails and always reports a timeout.
class FailAlwaysPlanner final : public core::Planner {
public:
    /// Returns nothing for every call.
    /// @param map the warehouse grid
    /// @param agents the fleet
    [[nodiscard]] std::optional<core::Plan> plan(const core::Map& map,
                                                 const std::vector<core::Agent>& agents) override {
        (void)map;
        (void)agents;
        return std::nullopt;
    }

    /// Returns the planner's name.
    [[nodiscard]] std::string name() const override { return "fail_always"; }

    /// Reports every call as a timeout.
    [[nodiscard]] bool lastCallTimedOut() const noexcept override { return true; }
};

// CBS is never more expensive than prioritized planning and stays safe.
TEST(RandomizedCbsTest, IsAtLeastAsCheapAsPrioritized) {
    const core::Map map = core::Map::fromText(kWarehouse);
    int solvedBoth = 0;
    for (std::uint32_t seed = 0; seed < 12; ++seed) {
        for (const int fleetSize : {2, 4, 6}) {
            const std::optional<std::vector<core::Agent>> fleet =
                benchmark::makeFleet(map, fleetSize, seed);
            ASSERT_TRUE(fleet.has_value());
            core::CbsPlanner cbs(core::CbsOptions{2000, 0, 0, 200000});
            core::PrioritizedPlanner prioritized;
            const std::optional<core::Plan> cbsPlan = cbs.plan(map, *fleet);
            const std::optional<core::Plan> prioritizedPlan = prioritized.plan(map, *fleet);
            if (!cbsPlan.has_value()) {
                continue;
            }
            const judge::RunLog log = sim::followPlanExactly(*cbsPlan, cbsPlan->makespan());
            EXPECT_TRUE(judge::analyse(log).safe()) << "seed " << seed << " fleet " << fleetSize;
            if (prioritizedPlan.has_value()) {
                EXPECT_LE(planCost(*cbsPlan), planCost(*prioritizedPlan))
                    << "seed " << seed << " fleet " << fleetSize;
                ++solvedBoth;
            }
        }
    }
    EXPECT_GT(solvedBoth, 0);
}

// A failing primary planner falls back to prioritized and the counter records it.
TEST(ContinuousCbsTest, CountsFallbacks) {
    const std::optional<core::Map> map =
        core::Map::fromFile(std::string(LOTSE_SOURCE_DIR) + "/maps/warehouse_small.map");
    ASSERT_TRUE(map.has_value());
    std::vector<core::GridCell> starts;
    starts.reserve(4);
    for (int robot = 0; robot < 4; ++robot) {
        starts.push_back(map->parkingCells().at(static_cast<std::size_t>(robot)));
    }
    const core::OrderGeneratorOptions generator{40, 0, 4, 1, 3};
    const std::vector<core::Order> orders = core::generateOrders(*map, generator, 3);

    FailAlwaysPlanner primary;
    core::PrioritizedPlanner fallback(core::PrioritizedOptions{core::AgentOrder::FarthestFirst, 0,
                                                               core::continuousHorizon(*map, 4)});
    core::NearestRobotAssigner assigner;
    core::ContinuousConfig config;
    config.seed = 3;
    config.maxSteps = 200;
    const core::ContinuousResult result =
        core::runContinuous(*map, starts, orders, primary, assigner, config, &fallback);

    EXPECT_GT(result.fallbacks, 0);
    EXPECT_EQ(result.fallbackTimeouts, result.fallbacks);
    EXPECT_TRUE(result.verdict.safe());
    EXPECT_FALSE(result.deadlock);
    EXPECT_GT(result.ordersCompleted, 0);
}

}  // namespace
}  // namespace lotse
