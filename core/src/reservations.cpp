// Implements the reservations declared in reservations.hpp.

#include "lotse/core/reservations.hpp"

namespace lotse::core {
namespace {

/// Mixes two hash values into one.
/// @param seed the running hash
/// @param value the value to mix in
[[nodiscard]] std::size_t mix(std::size_t seed, std::size_t value) noexcept {
    return seed ^ (value + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U));
}

}  // namespace

bool operator==(const CellTime& lhs, const CellTime& rhs) noexcept {
    return lhs.cell == rhs.cell && lhs.time == rhs.time;
}

bool operator==(const MoveTime& lhs, const MoveTime& rhs) noexcept {
    return lhs.from == rhs.from && lhs.to == rhs.to && lhs.time == rhs.time;
}

std::size_t CellTimeHash::operator()(const CellTime& key) const noexcept {
    return mix(GridCellHash{}(key.cell), static_cast<std::size_t>(key.time));
}

std::size_t MoveTimeHash::operator()(const MoveTime& key) const noexcept {
    const std::size_t endpoints = mix(GridCellHash{}(key.from), GridCellHash{}(key.to));
    return mix(endpoints, static_cast<std::size_t>(key.time));
}

void Reservations::reserveCell(GridCell cell, int time) {
    cells_.insert(CellTime{cell, time});
}

void Reservations::reserveMove(GridCell from, GridCell to, int time) {
    moves_.insert(MoveTime{from, to, time});
}

bool Reservations::cellReserved(GridCell cell, int time) const {
    return cells_.contains(CellTime{cell, time});
}

bool Reservations::moveReserved(GridCell from, GridCell to, int time) const {
    return moves_.contains(MoveTime{from, to, time});
}

void Reservations::clear() {
    cells_.clear();
    moves_.clear();
}

}  // namespace lotse::core
