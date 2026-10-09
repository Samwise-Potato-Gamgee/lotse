// The pass-order safety rule shared by the single-shot executor and the
// continuous runner: from a plan it derives which robot may enter a cell.

#pragma once

#include <unordered_map>
#include <vector>

#include "lotse/core/grid.hpp"
#include "lotse/core/plan.hpp"

namespace lotse::core {

/// A run of consecutive plan steps one robot spends in one cell.
struct CellInterval {
    int agent = 0;
    int first = 0;
    int last = 0;
};

/// The planned visits of every robot at every cell.
using CellSchedule = std::unordered_map<GridCell, std::vector<CellInterval>, GridCellHash>;

/// Builds, for every cell, the planned visit intervals of every route.
/// @param routes every robot's current route
[[nodiscard]] CellSchedule buildCellSchedule(const std::vector<Route>& routes);

/// Decides whether the pass-order rule lets one robot enter a cell this step.
/// The cell must be free after the step, no robot may enter the cell the mover
/// leaves, and every robot scheduled to pass the cell first must have left it.
/// @param mover index of the moving robot
/// @param target the cell the mover enters
/// @param arrivalIndex the mover's route index after the step
/// @param currentIndex every robot's route index now
/// @param nextIndex every robot's route index after the step
/// @param advancing whether each robot advances this step
/// @param routes every robot's current route
/// @param schedule the planned visits per cell
[[nodiscard]] bool passOrderAllows(int mover, GridCell target, int arrivalIndex,
                                   const std::vector<int>& currentIndex,
                                   const std::vector<int>& nextIndex,
                                   const std::vector<bool>& advancing,
                                   const std::vector<Route>& routes, const CellSchedule& schedule);

}  // namespace lotse::core
