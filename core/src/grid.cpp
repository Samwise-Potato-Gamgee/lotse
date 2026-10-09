// Implements the grid cell helpers declared in grid.hpp.

#include "lotse/core/grid.hpp"

namespace lotse::core {
namespace {

/// Returns the absolute difference of two integers.
/// @param a first value
/// @param b second value
[[nodiscard]] int absolute_difference(int a, int b) noexcept {
    return a < b ? b - a : a - b;
}

}  // namespace

bool operator==(GridCell lhs, GridCell rhs) noexcept {
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

bool operator!=(GridCell lhs, GridCell rhs) noexcept {
    return !(lhs == rhs);
}

int manhattan_distance(GridCell from, GridCell to) noexcept {
    return absolute_difference(from.x, to.x) + absolute_difference(from.y, to.y);
}

std::string to_string(GridCell cell) {
    return "(" + std::to_string(cell.x) + ", " + std::to_string(cell.y) + ")";
}

std::size_t GridCellHash::operator()(GridCell cell) const noexcept {
    const auto x = static_cast<std::size_t>(cell.x);
    const auto y = static_cast<std::size_t>(cell.y);
    return (x * 73856093U) ^ (y * 19349663U);
}

}  // namespace lotse::core
