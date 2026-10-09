// Unit tests for the MAPF benchmark helpers: scenario parsing and one timed
// planner call.

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "lotse/benchmark/mapf_benchmark.hpp"
#include "lotse/core/cbs.hpp"
#include "lotse/core/map.hpp"
#include "lotse/core/prioritized_planner.hpp"

#include <gtest/gtest.h>

namespace lotse {
namespace {

constexpr const char* kScen =
    "version 1\n"
    "empty-8-8.map\n"
    "1\tempty-8-8.map\t8\t8\t5\t6\t3\t2\t31.3\n"
    "2\tempty-8-8.map\t8\t8\t0\t0\t7\t7\t10.0\n"
    "3\tempty-8-8.map\t8\t8\t1\t2\t6\t5\t8.0\n";

/// Writes a small scenario file and returns its path.
[[nodiscard]] std::string writeScenFile() {
    const std::string path = "/tmp/lotse_mapf_test.scen";
    std::ofstream out{path};
    out << kScen;
    return path;
}

// Reads the first start and goal pairs, skipping header lines.
TEST(MapfScenTest, ReadsStartGoalPairs) {
    const std::string path = writeScenFile();
    const std::vector<core::Agent> agents = benchmark::readScen(path, 2);
    ASSERT_EQ(agents.size(), 2U);
    EXPECT_EQ(agents[0].start, (core::GridCell{5, 6}));
    EXPECT_EQ(agents[0].goal, (core::GridCell{3, 2}));
    EXPECT_EQ(agents[1].start, (core::GridCell{0, 0}));
    EXPECT_EQ(agents[1].goal, (core::GridCell{7, 7}));
    EXPECT_EQ(benchmark::readScen(path, 100).size(), 3U);
    std::remove(path.c_str());
}

// A missing scenario file parses to no agents.
TEST(MapfScenTest, MissingFileIsEmpty) {
    EXPECT_TRUE(benchmark::readScen("/tmp/lotse_missing.scen", 4).empty());
}

// One timed planner call reports solved, cost and time.
TEST(MapfBenchmarkTest, TimesOnePlannerCall) {
    const core::Map map = core::Map::fromText("height 3\nwidth 5\nmap\n.....\n.....\n.....\n");
    const std::vector<core::Agent> agents{{0, core::GridCell{0, 0}, core::GridCell{4, 0}}};
    core::PrioritizedPlanner planner;
    const benchmark::PlannerRun run = benchmark::runPlanner(map, agents, planner);
    EXPECT_TRUE(run.solved);
    EXPECT_FALSE(run.timedOut);
    EXPECT_EQ(run.cost, 4);
    EXPECT_GE(run.seconds, 0.0);

    benchmark::ComparisonRow row;
    row.planner = "prioritized";
    row.mapName = "test";
    row.robotCount = 1;
    row.scenario = 1;
    row.solved = run.solved;
    row.cost = run.cost;
    row.seconds = run.seconds;
    EXPECT_FALSE(benchmark::toComparisonCsvRow(row).empty());
    EXPECT_FALSE(benchmark::formatComparisonSummary({row}).empty());
    EXPECT_EQ(benchmark::comparisonCsvHeader().substr(0, 8), "planner,");
}

// A planner that cannot plan is reported as unsolved.
TEST(MapfBenchmarkTest, ReportsFailure) {
    const core::Map map = core::Map::fromText("height 1\nwidth 3\nmap\n.#.\n");
    const std::vector<core::Agent> agents{{0, core::GridCell{0, 0}, core::GridCell{1, 0}}};
    core::CbsPlanner planner(core::CbsOptions{100, 0, 0, 1000});
    const benchmark::PlannerRun run = benchmark::runPlanner(map, agents, planner);
    EXPECT_FALSE(run.solved);
    EXPECT_FALSE(run.timedOut);
    EXPECT_EQ(run.cost, 0);
}

}  // namespace
}  // namespace lotse
