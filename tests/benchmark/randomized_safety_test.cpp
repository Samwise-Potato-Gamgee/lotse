// Randomized safety tests on generated instances: the same check the headline
// benchmark performs, in miniature so it runs with every commit.

#include <cstdint>
#include <optional>
#include <vector>

#include "lotse/benchmark/safety_benchmark.hpp"
#include "lotse/core/executor.hpp"
#include "lotse/core/prioritized_planner.hpp"

#include <gtest/gtest.h>

namespace lotse {
namespace {

constexpr const char* kWarehouse =
    "name test_warehouse\n"
    "height 8\nwidth 10\nmap\n"
    "..........\n"
    ".#.#.#.#..\n"
    ".#.#.#.#..\n"
    "..........\n"
    ".#.#.#.#..\n"
    ".#.#.#.#..\n"
    "..........\n"
    "..........\n";

// Runs generated fleets with delays and checks the judge sees no violation.
TEST(RandomizedSafetyTest, StaysSafeOnGeneratedInstances) {
    const core::Map map = core::Map::fromText(kWarehouse);
    const std::vector<int> fleets{2, 5, 8};
    const std::vector<double> delays{0.0, 0.1, 0.3};
    core::PrioritizedPlanner planner;

    int runs = 0;
    for (std::uint32_t seed = 0; seed < 3; ++seed) {
        for (const int fleetSize : fleets) {
            const std::optional<std::vector<core::Agent>> fleet =
                benchmark::makeFleet(map, fleetSize, seed);
            ASSERT_TRUE(fleet.has_value());
            const std::vector<core::Agent>& agents = fleet.value();
            for (const double delay : delays) {
                core::ExecutionConfig config;
                config.seed = seed;
                config.delayProbability = delay;
                const core::ExecutionResult result = core::execute(map, agents, planner, config);
                EXPECT_TRUE(result.verdict.safe()) << "seed " << seed << " fleet " << fleetSize;
                EXPECT_FALSE(result.deadlock) << "seed " << seed << " fleet " << fleetSize;
                EXPECT_FALSE(result.stepLimitHit) << "seed " << seed << " fleet " << fleetSize;
                EXPECT_FALSE(result.initialPlanFailed) << "seed " << seed << " fleet " << fleetSize;
                ++runs;
            }
        }
    }
    EXPECT_EQ(runs, 27);
}

}  // namespace
}  // namespace lotse
