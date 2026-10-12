// The fleet planner interface. Prioritized planning and conflict-based search
// both implement it.

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

    /// Returns true when the last plan call stopped because it hit its time
    /// limit rather than because no solution exists. The default is false.
    [[nodiscard]] virtual bool lastCallTimedOut() const noexcept { return false; }

    /// Returns the robot order the last plan call used, or an empty list when
    /// the planner does not report one. The default is empty.
    [[nodiscard]] virtual std::vector<int> lastPlanningOrder() const { return {}; }

    /// Returns the attempt number the last plan call succeeded on, or -1 when
    /// the planner does not report one. The default is -1.
    [[nodiscard]] virtual int lastPlanningAttempt() const noexcept { return -1; }
};

}  // namespace lotse::core
