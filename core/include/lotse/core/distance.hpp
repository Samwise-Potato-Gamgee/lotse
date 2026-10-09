// Shortest path length on the static grid, ignoring other robots. Used to
// estimate how far a robot is from an order's pickup when assigning orders.

#pragma once

#include "lotse/core/grid.hpp"
#include "lotse/core/map.hpp"

namespace lotse::core {

/// Returns the number of 4-connected moves on the shortest route between two
/// free cells, or -1 when no route exists.
/// @param map the warehouse grid
/// @param from the start cell, must be free
/// @param to the target cell, must be free
[[nodiscard]] int shortestPathSteps(const Map& map, GridCell from, GridCell to);

}  // namespace lotse::core
