// Implements the delay-aware executor declared in executor.hpp.

#include "lotse/core/executor.hpp"

#include <algorithm>
#include <cstddef>
#include <optional>
#include <random>
#include <utility>
#include <vector>

#include "lotse/core/pass_order.hpp"

namespace lotse::core {
namespace {

/// A robot during execution: how far along its current route it has got.
struct Robot {
    int routeIndex = 0;
};

/// Returns the cell a robot is standing on.
/// @param robot the robot
/// @param route the robot's current route
[[nodiscard]] GridCell positionOf(const Robot& robot, const Route& route) {
    return route.cellAt(robot.routeIndex);
}

/// Returns true when a robot has reached the last cell of its route.
/// @param robot the robot
/// @param route the robot's current route
[[nodiscard]] bool atGoal(const Robot& robot, const Route& route) {
    return robot.routeIndex >= route.steps();
}

/// Returns true when every robot has reached the last cell of its route.
/// @param robots the fleet
/// @param routes the fleet's current routes
[[nodiscard]] bool allAtGoal(const std::vector<Robot>& robots, const std::vector<Route>& routes) {
    for (std::size_t index = 0; index < robots.size(); ++index) {
        if (!atGoal(robots[index], routes[index])) {
            return false;
        }
    }
    return true;
}

/// Appends the current positions as the next trajectory frame.
/// @param log the trajectory to extend
/// @param robots the fleet
/// @param routes the fleet's current routes
void appendFrame(judge::RunLog& log, const std::vector<Robot>& robots,
                 const std::vector<Route>& routes) {
    judge::Frame frame;
    frame.reserve(robots.size());
    for (std::size_t index = 0; index < robots.size(); ++index) {
        frame.push_back(positionOf(robots[index], routes[index]));
    }
    log.frames.push_back(std::move(frame));
}

}  // namespace

ExecutionResult execute(const Map& map, const std::vector<Agent>& agents, Planner& planner,
                        const ExecutionConfig& config) {
    ExecutionResult result;
    const auto fleetSize = static_cast<int>(agents.size());
    result.log.agentCount = fleetSize;

    if (fleetSize == 0) {
        result.finished = true;
        result.verdict = judge::analyse(result.log);
        return result;
    }

    std::vector<Robot> robots(static_cast<std::size_t>(fleetSize));
    std::vector<Route> routes;

    const std::optional<Plan> initial = planner.plan(map, agents);
    if (!initial.has_value()) {
        result.initialPlanFailed = true;
        result.verdict = judge::analyse(result.log);
        return result;
    }
    routes = initial->routes;
    CellSchedule schedule = buildCellSchedule(routes);

    std::mt19937 rng(config.seed);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    int planStartStep = 0;
    int lastReplanStep = -config.replanCooldown - 1;
    int stagnantSteps = 0;

    appendFrame(result.log, robots, routes);

    int step = 0;
    while (step < config.maxSteps) {
        if (allAtGoal(robots, routes)) {
            result.finished = true;
            break;
        }
        ++step;

        std::vector<bool> willAdvance(static_cast<std::size_t>(fleetSize), false);
        std::vector<int> nextIndex(static_cast<std::size_t>(fleetSize), 0);
        std::vector<int> currentIndex(static_cast<std::size_t>(fleetSize), 0);
        for (int agent = 0; agent < fleetSize; ++agent) {
            const auto index = static_cast<std::size_t>(agent);
            Robot& robot = robots[index];
            const double draw = uniform(rng);
            currentIndex[index] = robot.routeIndex;
            nextIndex[index] = robot.routeIndex;
            if (atGoal(robot, routes[index])) {
                continue;
            }
            const int candidate = robot.routeIndex + 1;
            if (routes[index].cellAt(candidate) == positionOf(robot, routes[index])) {
                nextIndex[index] = candidate;
                willAdvance[index] = true;
                continue;
            }
            if (draw < config.delayProbability) {
                continue;
            }
            nextIndex[index] = candidate;
            willAdvance[index] = true;
        }

        if (config.passOrderEnabled) {
            bool changed = true;
            while (changed) {
                changed = false;
                for (int agent = 0; agent < fleetSize; ++agent) {
                    const auto index = static_cast<std::size_t>(agent);
                    if (!willAdvance[index] || nextIndex[index] == currentIndex[index]) {
                        continue;
                    }
                    const GridCell target = routes[index].cellAt(nextIndex[index]);
                    if (!passOrderAllows(agent, target, nextIndex[index], currentIndex, nextIndex,
                                         willAdvance, routes, schedule)) {
                        nextIndex[index] = currentIndex[index];
                        willAdvance[index] = false;
                        changed = true;
                    }
                }
            }
        }

        int moved = 0;
        for (int agent = 0; agent < fleetSize; ++agent) {
            const auto index = static_cast<std::size_t>(agent);
            if (!willAdvance[index]) {
                continue;
            }
            const GridCell before = positionOf(robots[index], routes[index]);
            robots[index].routeIndex = nextIndex[index];
            if (positionOf(robots[index], routes[index]) != before) {
                ++moved;
            }
        }

        appendFrame(result.log, robots, routes);
        stagnantSteps = moved == 0 ? stagnantSteps + 1 : 0;

        if (stagnantSteps >= config.deadlockPatience) {
            result.deadlock = true;
            break;
        }

        bool behind = false;
        const int expectedIndex = step - planStartStep;
        for (std::size_t index = 0; index < robots.size(); ++index) {
            if (!atGoal(robots[index], routes[index]) &&
                expectedIndex - robots[index].routeIndex > config.replanBehindSlack) {
                behind = true;
                break;
            }
        }
        const bool stuck = stagnantSteps >= config.stuckPatience;
        const bool cooldownOver = step - lastReplanStep >= config.replanCooldown;
        if (!(behind || stuck) || !cooldownOver) {
            continue;
        }

        ++result.replans;
        lastReplanStep = step;
        std::vector<Agent> current;
        current.reserve(static_cast<std::size_t>(fleetSize));
        for (int agent = 0; agent < fleetSize; ++agent) {
            const auto index = static_cast<std::size_t>(agent);
            current.push_back(Agent{agents[index].id, positionOf(robots[index], routes[index]),
                                    agents[index].goal});
        }
        const std::optional<Plan> replanned = planner.plan(map, current);
        if (!replanned.has_value()) {
            ++result.replanFailures;
            continue;
        }
        routes = replanned->routes;
        for (Robot& robot : robots) {
            robot.routeIndex = 0;
        }
        schedule = buildCellSchedule(routes);
        planStartStep = step;
        stagnantSteps = 0;
    }

    if (!result.finished && !result.deadlock) {
        result.stepLimitHit = true;
    }
    result.stepsUsed = step;
    result.verdict = judge::analyse(result.log);
    return result;
}

}  // namespace lotse::core
