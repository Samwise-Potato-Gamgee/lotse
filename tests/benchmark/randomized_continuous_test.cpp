// Randomized continuous runs on a warehouse map, in miniature so the safety
// check runs with every commit.

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "lotse/core/continuous_runner.hpp"
#include "lotse/core/map.hpp"
#include "lotse/core/order.hpp"
#include "lotse/core/prioritized_planner.hpp"
#include "lotse/core/task_assigner.hpp"

#include <gtest/gtest.h>

namespace lotse {
namespace {

// Runs generated fleets on the small warehouse map and checks the judge.
TEST(RandomizedContinuousTest, StaysSafeOnGeneratedOrders) {
    const std::string path = std::string(LOTSE_SOURCE_DIR) + "/maps/warehouse_small.map";
    const std::optional<core::Map> map = core::Map::fromFile(path);
    ASSERT_TRUE(map.has_value());
    ASSERT_GE(map->parkingCells().size(), 8U);

    const std::vector<int> fleets{2, 5};
    const std::vector<double> delays{0.0, 0.3};
    int runs = 0;
    for (std::uint32_t seed = 0; seed < 2; ++seed) {
        for (const int fleetSize : fleets) {
            std::vector<core::GridCell> starts;
            starts.reserve(static_cast<std::size_t>(fleetSize));
            for (int robot = 0; robot < fleetSize; ++robot) {
                starts.push_back(map->parkingCells().at(static_cast<std::size_t>(robot)));
            }
            const core::OrderGeneratorOptions options{200, 0, 4, 1, 3};
            const std::vector<core::Order> orders = core::generateOrders(*map, options, seed);
            for (const double delay : delays) {
                core::PrioritizedPlanner planner(core::PrioritizedOptions{
                    core::AgentOrder::FarthestFirst, 0, core::continuousHorizon(*map, fleetSize)});
                core::NearestRobotAssigner assigner;
                core::ContinuousConfig config;
                config.seed = seed;
                config.delayProbability = delay;
                config.maxSteps = 200;
                const core::ContinuousResult result =
                    core::runContinuous(*map, starts, orders, planner, assigner, config);
                EXPECT_TRUE(result.verdict.safe()) << "seed " << seed << " fleet " << fleetSize;
                EXPECT_FALSE(result.deadlock) << "seed " << seed << " fleet " << fleetSize;
                EXPECT_GT(result.ordersCompleted, 0) << "seed " << seed << " fleet " << fleetSize;
                ++runs;
            }
        }
    }
    EXPECT_EQ(runs, 8);
}

}  // namespace
}  // namespace lotse
