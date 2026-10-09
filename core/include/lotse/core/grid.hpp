#pragma once

#include <string>

namespace lotse::core {

/// A cell on the warehouse grid. `x` counts columns from the west wall and
/// `y` counts rows from the south wall. All coordinates are integers; the
/// centimetre-accurate world position is a separate concern handled by the
/// simulator bridge.
struct GridCell {
    int x = 0;
    int y = 0;
};

[[nodiscard]] bool operator==(GridCell lhs, GridCell rhs) noexcept;
[[nodiscard]] bool operator!=(GridCell lhs, GridCell rhs) noexcept;

/// Manhattan distance between two cells: the number of 4-connected steps
/// needed on an empty grid. The grid planners use this as their heuristic.
[[nodiscard]] int manhattan_distance(GridCell from, GridCell to) noexcept;

/// Human readable form, for example "(3, -4)". Used in logs and test output.
[[nodiscard]] std::string to_string(GridCell cell);

}  // namespace lotse::core
