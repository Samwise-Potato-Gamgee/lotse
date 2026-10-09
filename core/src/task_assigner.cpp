// Implements the task assigners declared in task_assigner.hpp.

#include "lotse/core/task_assigner.hpp"

#include <cstddef>
#include <limits>

#include "lotse/core/distance.hpp"

namespace lotse::core {

std::optional<int> NearestRobotAssigner::chooseRobot(const Map& map, const Order& order,
                                                     const std::vector<GridCell>& robotPositions,
                                                     const std::vector<int>& freeRobotIds) {
    int bestRobot = -1;
    int bestDistance = std::numeric_limits<int>::max();
    for (const int robot : freeRobotIds) {
        const auto index = static_cast<std::size_t>(robot);
        if (index >= robotPositions.size()) {
            continue;
        }
        const int distance = shortestPathSteps(map, robotPositions[index], order.pickup);
        if (distance >= 0 && (bestRobot < 0 || distance < bestDistance ||
                              (distance == bestDistance && robot < bestRobot))) {
            bestRobot = robot;
            bestDistance = distance;
        }
    }
    if (bestRobot < 0) {
        return std::nullopt;
    }
    return bestRobot;
}

std::string NearestRobotAssigner::name() const {
    return "nearest_robot";
}

}  // namespace lotse::core
