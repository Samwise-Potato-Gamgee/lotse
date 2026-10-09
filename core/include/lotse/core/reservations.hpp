// Cells and directed moves already claimed by other robots, indexed by plan step.

#pragma once

#include <cstddef>
#include <unordered_set>

#include "lotse/core/grid.hpp"

namespace lotse::core {

/// A cell at one plan step.
struct CellTime {
    GridCell cell{};
    int time = 0;
};

/// A directed move over one plan step, recorded by the step it starts at.
struct MoveTime {
    GridCell from{};
    GridCell to{};
    int time = 0;
};

/// Compares two cell-times.
/// @param lhs first cell-time
/// @param rhs second cell-time
[[nodiscard]] bool operator==(const CellTime& lhs, const CellTime& rhs) noexcept;

/// Compares two move-times.
/// @param lhs first move-time
/// @param rhs second move-time
[[nodiscard]] bool operator==(const MoveTime& lhs, const MoveTime& rhs) noexcept;

/// Hashes a cell-time.
struct CellTimeHash {
    /// Hashes one cell-time.
    /// @param key the key to hash
    [[nodiscard]] std::size_t operator()(const CellTime& key) const noexcept;
};

/// Hashes a move-time.
struct MoveTimeHash {
    /// Hashes one move-time.
    /// @param key the key to hash
    [[nodiscard]] std::size_t operator()(const MoveTime& key) const noexcept;
};

/// Records which cells and moves are taken at which plan steps.
class Reservations {
public:
    /// Claims a cell at a plan step.
    /// @param cell the claimed cell
    /// @param time the plan step
    void reserveCell(GridCell cell, int time);

    /// Claims a directed move over one plan step.
    /// @param from the cell left
    /// @param to the cell entered
    /// @param time the plan step the move starts at
    void reserveMove(GridCell from, GridCell to, int time);

    /// Returns true when the cell is claimed at that step.
    /// @param cell the cell to test
    /// @param time the plan step
    [[nodiscard]] bool cellReserved(GridCell cell, int time) const;

    /// Returns true when the directed move is claimed at that step.
    /// @param from the cell left
    /// @param to the cell entered
    /// @param time the plan step the move starts at
    [[nodiscard]] bool moveReserved(GridCell from, GridCell to, int time) const;

    /// Removes every claim.
    void clear();

private:
    std::unordered_set<CellTime, CellTimeHash> cells_;
    std::unordered_set<MoveTime, MoveTimeHash> moves_;
};

}  // namespace lotse::core
