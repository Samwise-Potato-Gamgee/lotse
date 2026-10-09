// The task assigner interface: decides which free robot should serve an order.
// The first implementation picks the free robot closest to the pickup.

#pragma once

#include <optional>
#include <string>
#include <vector>

#include "lotse/core/grid.hpp"
#include "lotse/core/map.hpp"
#include "lotse/core/order.hpp"

namespace lotse::core {

/// Decides which free robot serves an order.
class TaskAssigner {
public:
    TaskAssigner() = default;
    TaskAssigner(const TaskAssigner&) = delete;
    TaskAssigner& operator=(const TaskAssigner&) = delete;
    TaskAssigner(TaskAssigner&&) = delete;
    TaskAssigner& operator=(TaskAssigner&&) = delete;
    virtual ~TaskAssigner() = default;

    /// Chooses a robot for one order.
    /// @param map the warehouse grid
    /// @param order the order to assign
    /// @param robotPositions the current cell of every robot, indexed by robot id
    /// @param freeRobotIds the robots that have no order right now
    /// @return the chosen robot id, or nothing when the pickup is unreachable
    [[nodiscard]] virtual std::optional<int> chooseRobot(
        const Map& map, const Order& order, const std::vector<GridCell>& robotPositions,
        const std::vector<int>& freeRobotIds) = 0;

    /// Returns the assigner's name for reports and logs.
    [[nodiscard]] virtual std::string name() const = 0;
};

/// Assigns an order to the free robot with the shortest static route to the
/// pickup, breaking ties by the smaller robot id.
class NearestRobotAssigner final : public TaskAssigner {
public:
    /// Chooses the closest free robot to the order's pickup.
    /// @param map the warehouse grid
    /// @param order the order to assign
    /// @param robotPositions the current cell of every robot
    /// @param freeRobotIds the robots that have no order
    [[nodiscard]] std::optional<int> chooseRobot(const Map& map, const Order& order,
                                                 const std::vector<GridCell>& robotPositions,
                                                 const std::vector<int>& freeRobotIds) override;

    /// Returns the assigner's name.
    [[nodiscard]] std::string name() const override;
};

}  // namespace lotse::core
