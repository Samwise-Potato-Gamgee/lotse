// Plan types: the planning problem for one robot, its route, and the whole plan.

#pragma once

#include <cstddef>
#include <vector>

#include "lotse/core/grid.hpp"

namespace lotse::core {

/// One robot's planning problem: where it starts and where it must go.
struct Agent {
    int id = 0;
    GridCell start{};
    GridCell goal{};
};

/// One robot's route: cells[t] is where the robot is at plan step t.
struct Route {
    int agentId = 0;
    std::vector<GridCell> cells;

    /// Returns the number of moves from the start to the goal.
    [[nodiscard]] int steps() const noexcept { return static_cast<int>(cells.size()) - 1; }

    /// Returns the cell at a plan step, clamped to the last cell.
    /// @param time plan step, zero based
    [[nodiscard]] GridCell cellAt(int time) const noexcept {
        const int last = steps();
        return cells[static_cast<std::size_t>(time > last ? last : time)];
    }

    /// Returns the first plan step at which the robot is at a cell, or -1.
    /// @param cell the cell to look for
    [[nodiscard]] int firstTimeAt(GridCell cell) const noexcept {
        for (int time = 0; time <= steps(); ++time) {
            if (cellAt(time) == cell) {
                return time;
            }
        }
        return -1;
    }

    /// Returns the last plan step at which the robot is at a cell, or -1.
    /// @param cell the cell to look for
    [[nodiscard]] int lastTimeAt(GridCell cell) const noexcept {
        for (int time = steps(); time >= 0; --time) {
            if (cellAt(time) == cell) {
                return time;
            }
        }
        return -1;
    }
};

/// A conflict-free set of routes for the whole fleet, one route per agent in
/// the same order as the agents were given to the planner.
struct Plan {
    std::vector<Route> routes;

    /// Returns the number of agents in the plan.
    [[nodiscard]] int agentCount() const noexcept { return static_cast<int>(routes.size()); }

    /// Returns the number of steps until every robot has finished.
    [[nodiscard]] int makespan() const noexcept {
        int result = 0;
        for (const Route& route : routes) {
            result = route.steps() > result ? route.steps() : result;
        }
        return result;
    }
};

}  // namespace lotse::core
