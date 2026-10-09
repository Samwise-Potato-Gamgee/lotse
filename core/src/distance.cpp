// Implements the static shortest path helper declared in distance.hpp.

#include "lotse/core/distance.hpp"

#include <array>
#include <cstddef>
#include <queue>
#include <unordered_map>

namespace lotse::core {
namespace {

/// The four 4-connected moves.
constexpr std::array<GridCell, 4> kMoves{{{0, 1}, {1, 0}, {0, -1}, {-1, 0}}};

}  // namespace

int shortestPathSteps(const Map& map, GridCell from, GridCell to) {
    if (!map.isFree(from) || !map.isFree(to)) {
        return -1;
    }
    if (from == to) {
        return 0;
    }

    std::queue<GridCell> frontier;
    std::unordered_map<GridCell, int, GridCellHash> distance;
    distance[from] = 0;
    frontier.push(from);

    while (!frontier.empty()) {
        const GridCell cell = frontier.front();
        frontier.pop();
        const int steps = distance.at(cell);
        for (const GridCell& move : kMoves) {
            const GridCell next{cell.x + move.x, cell.y + move.y};
            if (!map.isFree(next) || distance.contains(next)) {
                continue;
            }
            if (next == to) {
                return steps + 1;
            }
            distance[next] = steps + 1;
            frontier.push(next);
        }
    }
    return -1;
}

}  // namespace lotse::core
