// Single-robot route search in space and time, avoiding cells and moves that
// other robots have already claimed.

#pragma once

#include <optional>

#include "lotse/core/map.hpp"
#include "lotse/core/plan.hpp"
#include "lotse/core/reservations.hpp"

namespace lotse::core {

/// Bounds for one route search.
struct SearchLimits {
    int horizon = 0;
    int maxExpansions = 2000000;
};

/// Finds a shortest route for one robot that never shares a cell or a move with
/// the given reservations. Waiting in place is allowed and costs one step.
/// @param map the warehouse grid
/// @param agent the robot's start and goal
/// @param reserved cells and moves already claimed by other robots
/// @param limits time horizon and expansion cap
/// @return the route when one exists within the limits
[[nodiscard]] std::optional<Route> searchRoute(const Map& map, const Agent& agent,
                                               const Reservations& reserved,
                                               const SearchLimits& limits);

}  // namespace lotse::core
