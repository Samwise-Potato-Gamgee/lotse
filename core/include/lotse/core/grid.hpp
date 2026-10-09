// Warehouse grid types: a cell type, its hash and the Manhattan heuristic used
// by the grid planners.

#pragma once

#include <cstddef>
#include <string>

namespace lotse::core {

/// A cell on the warehouse grid: x is the column, y is the row.
struct GridCell {
    int x = 0;
    int y = 0;
};

/// Compares two cells by both coordinates.
/// @param lhs left cell
/// @param rhs right cell
[[nodiscard]] bool operator==(GridCell lhs, GridCell rhs) noexcept;

/// Compares two cells, true when they differ.
/// @param lhs left cell
/// @param rhs right cell
[[nodiscard]] bool operator!=(GridCell lhs, GridCell rhs) noexcept;

/// Manhattan distance: the number of 4-connected steps on an empty grid.
/// @param from start cell
/// @param to target cell
[[nodiscard]] int manhattan_distance(GridCell from, GridCell to) noexcept;

/// Formats a cell as "(x, y)".
/// @param cell the cell to format
[[nodiscard]] std::string to_string(GridCell cell);

/// Hashes a cell so it can key an unordered container.
struct GridCellHash {
    /// Hashes one cell.
    /// @param cell the cell to hash
    [[nodiscard]] std::size_t operator()(GridCell cell) const noexcept;
};

}  // namespace lotse::core
