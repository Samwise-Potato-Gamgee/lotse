// Implements the space-time A* route search declared in space_time_search.hpp.

#include "lotse/core/space_time_search.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <vector>

namespace lotse::core {
namespace {

/// A search node: a cell reached at a plan step with a known cost.
struct Node {
    int estimate = 0;
    int cost = 0;
    int time = 0;
    GridCell cell{};
};

/// Orders nodes by estimated total cost, cheapest first.
struct CheapestFirst {
    /// Compares two nodes for the priority queue.
    /// @param lhs first node
    /// @param rhs second node
    [[nodiscard]] bool operator()(const Node& lhs, const Node& rhs) const noexcept {
        return lhs.estimate > rhs.estimate;
    }
};

/// The four 4-connected moves; waiting is handled separately.
constexpr std::array<GridCell, 4> kMoves{{{0, 1}, {1, 0}, {0, -1}, {-1, 0}}};

/// Packs a cell and a step into one key for the search maps.
/// @param cell the cell
/// @param time the plan step
/// @param cellCount the number of cells in the map
/// @param width the map width
[[nodiscard]] std::int64_t stateKey(GridCell cell, int time, int cellCount, int width) noexcept {
    const auto index = (static_cast<std::int64_t>(cell.y) * static_cast<std::int64_t>(width)) +
                       static_cast<std::int64_t>(cell.x);
    return (static_cast<std::int64_t>(time) * static_cast<std::int64_t>(cellCount)) + index;
}

/// Returns true when a cell stays free from a plan step to the horizon.
/// @param cell the cell to test
/// @param fromTime the first plan step to test
/// @param horizon the last plan step to test
/// @param reserved cells already claimed by other robots
[[nodiscard]] bool staysFree(GridCell cell, int fromTime, int horizon,
                             const Reservations& reserved) {
    for (int time = fromTime; time <= horizon; ++time) {
        if (reserved.cellReserved(cell, time)) {
            return false;
        }
    }
    return true;
}

/// Rebuilds the route from the search parent links.
/// @param goalKey the key of the goal state
/// @param parents maps each state to the state it came from
/// @param cellCount the number of cells in the map
/// @param width the map width
/// @param agentId the agent the route belongs to
[[nodiscard]] Route rebuildRoute(std::int64_t goalKey,
                                 const std::unordered_map<std::int64_t, std::int64_t>& parents,
                                 int cellCount, int width, int agentId) {
    std::vector<GridCell> cells;
    std::int64_t key = goalKey;
    while (true) {
        const auto index = static_cast<int>(key % cellCount);
        cells.push_back(GridCell{index % width, index / width});
        const auto parent = parents.find(key);
        if (parent == parents.end()) {
            break;
        }
        key = parent->second;
    }
    std::reverse(cells.begin(), cells.end());
    Route route;
    route.agentId = agentId;
    route.cells = std::move(cells);
    return route;
}

}  // namespace

std::optional<Route> searchRoute(const Map& map, const Agent& agent, const Reservations& reserved,
                                 const SearchLimits& limits) {
    if (!map.isFree(agent.start) || !map.isFree(agent.goal)) {
        return std::nullopt;
    }

    const int cellCount = map.width() * map.height();
    const int horizon = limits.horizon < 0 ? 0 : limits.horizon;
    const std::int64_t startKey = stateKey(agent.start, 0, cellCount, map.width());

    std::priority_queue<Node, std::vector<Node>, CheapestFirst> open;
    std::unordered_map<std::int64_t, int> bestCost;
    std::unordered_map<std::int64_t, std::int64_t> parents;

    bestCost[startKey] = 0;
    open.push(Node{manhattan_distance(agent.start, agent.goal), 0, 0, agent.start});

    int expansions = 0;
    while (!open.empty()) {
        const Node node = open.top();
        open.pop();

        const std::int64_t key = stateKey(node.cell, node.time, cellCount, map.width());
        if (node.cost > bestCost[key]) {
            continue;
        }
        if (node.cell == agent.goal && staysFree(agent.goal, node.time, horizon, reserved)) {
            return rebuildRoute(key, parents, cellCount, map.width(), agent.id);
        }
        if (++expansions > limits.maxExpansions) {
            break;
        }
        if (node.time >= horizon) {
            continue;
        }

        const int nextTime = node.time + 1;
        if (!reserved.cellReserved(node.cell, nextTime)) {
            const std::int64_t waitKey = stateKey(node.cell, nextTime, cellCount, map.width());
            const int waitCost = node.cost + 1;
            if (!bestCost.contains(waitKey) || waitCost < bestCost[waitKey]) {
                bestCost[waitKey] = waitCost;
                parents[waitKey] = key;
                open.push(Node{waitCost + manhattan_distance(node.cell, agent.goal), waitCost,
                               nextTime, node.cell});
            }
        }

        for (const GridCell& move : kMoves) {
            const GridCell next{node.cell.x + move.x, node.cell.y + move.y};
            if (!map.isFree(next)) {
                continue;
            }
            if (reserved.cellReserved(next, nextTime)) {
                continue;
            }
            if (reserved.moveReserved(node.cell, next, node.time) ||
                reserved.moveReserved(next, node.cell, node.time)) {
                continue;
            }
            const std::int64_t nextKey = stateKey(next, nextTime, cellCount, map.width());
            const int nextCost = node.cost + 1;
            if (!bestCost.contains(nextKey) || nextCost < bestCost[nextKey]) {
                bestCost[nextKey] = nextCost;
                parents[nextKey] = key;
                open.push(Node{nextCost + manhattan_distance(next, agent.goal), nextCost, nextTime,
                               next});
            }
        }
    }
    return std::nullopt;
}

}  // namespace lotse::core
