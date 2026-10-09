// Transport orders and a seeded generator that turns a map's stations into a
// repeatable stream of orders.

#pragma once

#include <cstdint>
#include <string_view>
#include <vector>

#include "lotse/core/grid.hpp"
#include "lotse/core/map.hpp"

namespace lotse::core {

/// One transport order: fetch a load at the pickup, bring it to the drop-off.
struct Order {
    int id = 0;
    GridCell pickup{};
    GridCell dropoff{};
    int arrivalStep = 0;
    int priority = 0;
};

/// Tuning for the order generator.
struct OrderGeneratorOptions {
    int orderCount = 0;
    int firstArrivalStep = 0;
    int arrivalInterval = 2;
    int arrivalJitter = 1;
    int priorityLevels = 3;
};

/// Returns the cells of all stations whose name starts with the prefix.
/// @param map the warehouse grid
/// @param prefix the station name prefix
[[nodiscard]] std::vector<GridCell> stationCellsWithPrefix(const Map& map, std::string_view prefix);

/// Returns true when the first order should be served before the second:
/// higher priority first, then earlier arrival, then smaller id.
/// @param a first order
/// @param b second order
[[nodiscard]] bool orderHasHigherPriority(const Order& a, const Order& b) noexcept;

/// Returns an order's ageing-adjusted priority: its priority times the ageing
/// interval plus the steps it has waited. Higher is served first, so a
/// low-priority order eventually overtakes a fresh high-priority one.
/// @param order the order to score
/// @param step the current step
/// @param ageingInterval steps of waiting that add one priority point
[[nodiscard]] long long effectivePriority(const Order& order, int step,
                                          int ageingInterval) noexcept;

/// Generates a repeatable stream of orders. Pickups and drop-offs come from the
/// map's `pickup*` and `dropoff*` stations; a map without those uses free cells.
/// @param map the warehouse grid
/// @param options order count, arrival spacing, jitter and priority levels
/// @param seed the random seed
[[nodiscard]] std::vector<Order> generateOrders(const Map& map,
                                                const OrderGeneratorOptions& options,
                                                std::uint32_t seed);

}  // namespace lotse::core
