// Implements the order stream generator declared in order.hpp.

#include "lotse/core/order.hpp"

#include <algorithm>
#include <cstddef>
#include <random>
#include <stdexcept>

namespace lotse::core {
namespace {

/// Picks a random cell, never the excluded one.
/// @param cells the candidate cells, at least two
/// @param excluded the cell to avoid
/// @param rng the random generator
[[nodiscard]] GridCell pickOther(const std::vector<GridCell>& cells, GridCell excluded,
                                 std::mt19937& rng) {
    std::uniform_int_distribution<std::size_t> choose(0, cells.size() - 1);
    for (int attempt = 0; attempt < 16; ++attempt) {
        const GridCell cell = cells[choose(rng)];
        if (cell != excluded) {
            return cell;
        }
    }
    for (const GridCell& cell : cells) {
        if (cell != excluded) {
            return cell;
        }
    }
    return cells.front();
}

/// Returns the candidate cells for one endpoint, falling back to free cells.
/// @param map the warehouse grid
/// @param preferred the station cells to use when there are at least two
[[nodiscard]] std::vector<GridCell> endpointChoices(const Map& map,
                                                    const std::vector<GridCell>& preferred) {
    if (preferred.size() >= 2U) {
        return preferred;
    }
    return map.freeCells();
}

}  // namespace

std::vector<GridCell> stationCellsWithPrefix(const Map& map, std::string_view prefix) {
    std::vector<GridCell> cells;
    for (const std::pair<std::string, GridCell>& entry : map.stations()) {
        if (entry.first.size() >= prefix.size() &&
            entry.first.compare(0, prefix.size(), prefix) == 0) {
            cells.push_back(entry.second);
        }
    }
    return cells;
}

bool orderHasHigherPriority(const Order& a, const Order& b) noexcept {
    if (a.priority != b.priority) {
        return a.priority > b.priority;
    }
    if (a.arrivalStep != b.arrivalStep) {
        return a.arrivalStep < b.arrivalStep;
    }
    return a.id < b.id;
}

long long effectivePriority(const Order& order, int step, int ageingInterval) noexcept {
    const int interval = ageingInterval > 0 ? ageingInterval : 1;
    const int age = step > order.arrivalStep ? step - order.arrivalStep : 0;
    return (static_cast<long long>(order.priority) * interval) + age;
}

std::vector<Order> generateOrders(const Map& map, const OrderGeneratorOptions& options,
                                  std::uint32_t seed) {
    const std::vector<GridCell> pickups =
        endpointChoices(map, stationCellsWithPrefix(map, "pickup"));
    const std::vector<GridCell> dropoffs =
        endpointChoices(map, stationCellsWithPrefix(map, "dropoff"));
    if (pickups.size() < 2U || dropoffs.size() < 2U) {
        throw std::runtime_error("order generator: the map has too few free cells");
    }

    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> jitter(
        0, options.arrivalJitter < 0 ? 0 : options.arrivalJitter);
    std::uniform_int_distribution<int> priority(
        0, options.priorityLevels > 1 ? options.priorityLevels - 1 : 0);

    std::vector<Order> orders;
    orders.reserve(static_cast<std::size_t>(options.orderCount < 0 ? 0 : options.orderCount));
    int arrival = options.firstArrivalStep;
    for (int index = 0; index < options.orderCount; ++index) {
        const GridCell pickup =
            pickups.at(std::uniform_int_distribution<std::size_t>(0, pickups.size() - 1)(rng));
        const GridCell dropoff = pickOther(dropoffs, pickup, rng);
        if (index > 0) {
            arrival += options.arrivalInterval;
        }
        arrival += jitter(rng);
        orders.push_back(Order{index, pickup, dropoff, arrival, priority(rng)});
    }
    return orders;
}

}  // namespace lotse::core
