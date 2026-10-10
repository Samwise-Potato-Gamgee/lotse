// Unit tests for the task assigners.

#include "lotse/core/task_assigner.hpp"

#include <optional>
#include <vector>

#include <gtest/gtest.h>

namespace lotse::core {
namespace {

constexpr const char* kRow = "height 1\nwidth 9\nmap\n.........\n";

// Returns the free robot closest to the pickup.
TEST(NearestRobotAssignerTest, PicksTheClosestRobot) {
    const Map map = Map::fromText(kRow);
    const std::vector<GridCell> positions{{0, 0}, {4, 0}, {8, 0}};
    const Order order{0, GridCell{5, 0}, GridCell{8, 0}, 0, 0};
    NearestRobotAssigner assigner;
    const std::optional<int> chosen = assigner.chooseRobot(map, order, positions, {0, 1, 2});
    ASSERT_TRUE(chosen.has_value());
    EXPECT_EQ(chosen.value(), 1);
    EXPECT_EQ(assigner.name(), "nearest_robot");
}

// Breaks a tie by the smaller robot id.
TEST(NearestRobotAssignerTest, BreaksTiesByRobotId) {
    const Map map = Map::fromText(kRow);
    const std::vector<GridCell> positions{{4, 0}, {6, 0}};
    const Order order{0, GridCell{5, 0}, GridCell{8, 0}, 0, 0};
    NearestRobotAssigner assigner;
    const std::optional<int> chosen = assigner.chooseRobot(map, order, positions, {0, 1});
    ASSERT_TRUE(chosen.has_value());
    EXPECT_EQ(chosen.value(), 0);
}

// Breaks an equal-route tie by the robot that has finished fewer orders.
TEST(NearestRobotAssignerTest, PrefersFewerCompletedOnAnEqualRoute) {
    const Map map = Map::fromText(kRow);
    const std::vector<GridCell> positions{{4, 0}, {6, 0}};
    const Order order{0, GridCell{5, 0}, GridCell{8, 0}, 0, 0};
    const std::vector<int> completed{5, 0};
    NearestRobotAssigner assigner;
    const std::optional<int> chosen =
        assigner.chooseRobot(map, order, positions, {0, 1}, completed);
    ASSERT_TRUE(chosen.has_value());
    EXPECT_EQ(chosen.value(), 1);
}

// The shortest route wins even when the other robot has finished fewer orders.
TEST(NearestRobotAssignerTest, ShortestRouteBeatsFewerCompleted) {
    const Map map = Map::fromText(kRow);
    const std::vector<GridCell> positions{{0, 0}, {4, 0}, {8, 0}};
    const Order order{0, GridCell{5, 0}, GridCell{8, 0}, 0, 0};
    const std::vector<int> completed{0, 9, 0};
    NearestRobotAssigner assigner;
    const std::optional<int> chosen =
        assigner.chooseRobot(map, order, positions, {1, 2}, completed);
    ASSERT_TRUE(chosen.has_value());
    EXPECT_EQ(chosen.value(), 1);
}

// Only the free robots are considered.
TEST(NearestRobotAssignerTest, IgnoresBusyRobots) {
    const Map map = Map::fromText(kRow);
    const std::vector<GridCell> positions{{4, 0}, {6, 0}};
    const Order order{0, GridCell{5, 0}, GridCell{8, 0}, 0, 0};
    NearestRobotAssigner assigner;
    const std::optional<int> chosen = assigner.chooseRobot(map, order, positions, {1});
    ASSERT_TRUE(chosen.has_value());
    EXPECT_EQ(chosen.value(), 1);
}

// Reports nothing when no free robot can reach the pickup.
TEST(NearestRobotAssignerTest, ReportsUnreachablePickup) {
    const Map map = Map::fromText("height 1\nwidth 5\nmap\n..#..\n");
    const std::vector<GridCell> positions{{0, 0}, {4, 0}};
    const Order order{0, GridCell{2, 0}, GridCell{4, 0}, 0, 0};
    NearestRobotAssigner assigner;
    EXPECT_FALSE(assigner.chooseRobot(map, order, positions, {0, 1}).has_value());
}

}  // namespace
}  // namespace lotse::core
