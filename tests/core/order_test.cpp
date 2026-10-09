// Unit tests for the order stream generator.

#include "lotse/core/order.hpp"

#include <algorithm>
#include <cstdint>
#include <set>
#include <vector>

#include <gtest/gtest.h>

namespace lotse::core {
namespace {

constexpr const char* kStations =
    "name test\n"
    "height 5\nwidth 9\nmap\n"
    ".........\n"
    ".........\n"
    ".........\n"
    ".........\n"
    ".........\n"
    "stations\n"
    "pickup0 0 0\n"
    "pickup1 1 0\n"
    "dropoff0 7 4\n"
    "dropoff1 8 4\n"
    "parking\n"
    "0 4\n"
    "1 4\n"
    "2 4\n";

// The generator is repeatable for a fixed seed.
TEST(OrderGeneratorTest, IsRepeatable) {
    const Map map = Map::fromText(kStations);
    const OrderGeneratorOptions options{10, 0, 2, 1, 3};
    const std::vector<Order> first = generateOrders(map, options, 42);
    const std::vector<Order> second = generateOrders(map, options, 42);
    ASSERT_EQ(first.size(), 10U);
    for (std::size_t index = 0; index < first.size(); ++index) {
        EXPECT_EQ(first[index].id, second[index].id);
        EXPECT_EQ(first[index].pickup, second[index].pickup);
        EXPECT_EQ(first[index].dropoff, second[index].dropoff);
        EXPECT_EQ(first[index].arrivalStep, second[index].arrivalStep);
        EXPECT_EQ(first[index].priority, second[index].priority);
    }
}

// Pickups and drop-offs come from the matching stations and differ.
TEST(OrderGeneratorTest, UsesMatchingStations) {
    const Map map = Map::fromText(kStations);
    const std::vector<Order> orders = generateOrders(map, OrderGeneratorOptions{25, 0, 1, 0, 4}, 7);
    const std::vector<GridCell> pickups = stationCellsWithPrefix(map, "pickup");
    const std::vector<GridCell> dropoffs = stationCellsWithPrefix(map, "dropoff");
    ASSERT_EQ(pickups.size(), 2U);
    ASSERT_EQ(dropoffs.size(), 2U);
    for (const Order& order : orders) {
        EXPECT_NE(std::find(pickups.begin(), pickups.end(), order.pickup), pickups.end());
        EXPECT_NE(std::find(dropoffs.begin(), dropoffs.end(), order.dropoff), dropoffs.end());
        EXPECT_NE(order.pickup, order.dropoff);
        EXPECT_GE(order.priority, 0);
        EXPECT_LT(order.priority, 4);
    }
}

// Arrival steps never go backwards.
TEST(OrderGeneratorTest, ArrivalsAreOrdered) {
    const Map map = Map::fromText(kStations);
    const std::vector<Order> orders = generateOrders(map, OrderGeneratorOptions{40, 3, 2, 1, 1}, 9);
    for (std::size_t index = 1; index < orders.size(); ++index) {
        EXPECT_GE(orders[index].arrivalStep, orders[index - 1].arrivalStep);
    }
    EXPECT_GE(orders.front().arrivalStep, 3);
}

// The queue order is priority, then arrival, then id.
TEST(OrderPriorityTest, RanksByPriorityArrivalAndId) {
    const Order low{5, GridCell{0, 0}, GridCell{1, 0}, 0, 1};
    const Order highLate{1, GridCell{0, 0}, GridCell{1, 0}, 10, 2};
    const Order highEarly{2, GridCell{0, 0}, GridCell{1, 0}, 1, 2};
    const Order samePriorityLaterId{3, GridCell{0, 0}, GridCell{1, 0}, 1, 2};
    EXPECT_TRUE(orderHasHigherPriority(highEarly, low));
    EXPECT_TRUE(orderHasHigherPriority(highEarly, highLate));
    EXPECT_TRUE(orderHasHigherPriority(highEarly, samePriorityLaterId));
    EXPECT_FALSE(orderHasHigherPriority(low, highEarly));
}

// Ageing lets a low-priority order overtake a fresh high-priority one.
TEST(OrderAgeingTest, LowPriorityCatchesUp) {
    const Order low{0, GridCell{0, 0}, GridCell{1, 0}, 0, 0};
    const Order high{1, GridCell{0, 0}, GridCell{1, 0}, 90, 1};
    EXPECT_LT(effectivePriority(low, 10, 30), effectivePriority(high, 10, 30));
    EXPECT_GT(effectivePriority(low, 90, 30), effectivePriority(high, 90, 30));
}

}  // namespace
}  // namespace lotse::core
