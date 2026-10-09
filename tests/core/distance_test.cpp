// Unit tests for the static shortest path helper.

#include "lotse/core/distance.hpp"

#include <gtest/gtest.h>

namespace lotse::core {
namespace {

// A short route on an open row.
TEST(ShortestPathStepsTest, CountsOpenGridSteps) {
    const Map map = Map::fromText("height 1\nwidth 5\nmap\n.....\n");
    EXPECT_EQ(shortestPathSteps(map, GridCell{0, 0}, GridCell{4, 0}), 4);
    EXPECT_EQ(shortestPathSteps(map, GridCell{4, 0}, GridCell{0, 0}), 4);
}

// A route around an obstacle is longer than the straight line.
TEST(ShortestPathStepsTest, RoutesAroundObstacles) {
    const Map map = Map::fromText("height 3\nwidth 3\nmap\n.#.\n...\n...\n");
    EXPECT_EQ(shortestPathSteps(map, GridCell{0, 0}, GridCell{2, 0}), 4);
}

// The same cell is zero steps away.
TEST(ShortestPathStepsTest, IsZeroForTheSameCell) {
    const Map map = Map::fromText("height 1\nwidth 1\nmap\n.\n");
    EXPECT_EQ(shortestPathSteps(map, GridCell{0, 0}, GridCell{0, 0}), 0);
}

// An obstacle or an unreachable cell reports minus one.
TEST(ShortestPathStepsTest, ReportsUnreachable) {
    const Map map = Map::fromText("height 1\nwidth 3\nmap\n.#.\n");
    EXPECT_EQ(shortestPathSteps(map, GridCell{0, 0}, GridCell{1, 0}), -1);
    EXPECT_EQ(shortestPathSteps(map, GridCell{0, 0}, GridCell{9, 9}), -1);
}

// A blocked copy of the map hides cells from the search.
TEST(ShortestPathStepsTest, RespectsExtraObstacles) {
    const Map map = Map::fromText("height 3\nwidth 3\nmap\n...\n...\n...\n");
    const Map blocked = map.withObstacles({GridCell{1, 0}, GridCell{1, 1}});
    EXPECT_EQ(shortestPathSteps(blocked, GridCell{0, 0}, GridCell{2, 0}), 6);
    EXPECT_EQ(shortestPathSteps(blocked, GridCell{0, 0}, GridCell{1, 0}), -1);
    EXPECT_EQ(blocked.freeCells().size(), 7U);
}

}  // namespace
}  // namespace lotse::core
