// The fleet planner interface. Prioritized planning implements it now, conflict
// based search will implement it in a later stage.

#pragma once

#include <optional>
#include <string>
#include <vector>

#include "lotse/core/map.hpp"
#include "lotse/core/plan.hpp"

namespace lotse::core {

/// Plans collision-free routes for a whole fleet at once.
class Planner {
public:
    Planner() = default;
    Planner(const Planner&) = delete;
    Planner& operator=(const Planner&) = delete;
    Planner(Planner&&) = delete;
    Planner& operator=(Planner&&) = delete;
    virtual ~Planner() = default;

    /// Plans routes for every agent together.
    /// @param map the warehouse grid
    /// @param agents the fleet, one start and one goal per robot
    /// @return a conflict-free plan with one route per agent, or nothing when
    ///         no plan exists
    [[nodiscard]] virtual std::optional<Plan> plan(const Map& map,
                                                   const std::vector<Agent>& agents) = 0;

    /// Returns the planner's name for reports and logs.
    [[nodiscard]] virtual std::string name() const = 0;
};

}  // namespace lotse::core
