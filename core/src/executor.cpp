// Implements the delay-aware executor declared in executor.hpp.

#include "lotse/core/executor.hpp"

#include <algorithm>
#include <cstddef>
#include <optional>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lotse::core {
namespace {

/// A robot during execution: its problem, its current route and its progress.
struct Robot {
    Agent problem{};
    Route route{};
    int routeIndex = 0;
};

/// A run of consecutive plan steps one robot spends in one cell.
struct CellInterval {
    int agent = 0;
    int first = 0;
    int last = 0;
};

using CellSchedule = std::unordered_map<GridCell, std::vector<CellInterval>, GridCellHash>;

/// Builds, for every cell, the planned visit intervals of every robot.
/// @param robots the fleet with their routes
[[nodiscard]] CellSchedule buildSchedule(const std::vector<Robot>& robots) {
    CellSchedule schedule;
    for (std::size_t agent = 0; agent < robots.size(); ++agent) {
        const Route& route = robots[agent].route;
        int time = 0;
        while (time <= route.steps()) {
            const int first = time;
            const GridCell cell = route.cellAt(time);
            while (time + 1 <= route.steps() && route.cellAt(time + 1) == cell) {
                ++time;
            }
            schedule[cell].push_back(CellInterval{static_cast<int>(agent), first, time});
            ++time;
        }
    }
    return schedule;
}

/// Returns the cell a robot is standing on.
/// @param robot the robot
[[nodiscard]] GridCell positionOf(const Robot& robot) {
    return robot.route.cellAt(robot.routeIndex);
}

/// Returns the cell a robot would stand on after a route index.
/// @param robot the robot
/// @param index the route index to look at
[[nodiscard]] GridCell cellAtIndex(const Robot& robot, int index) {
    return robot.route.cellAt(index);
}

/// Returns true when a robot has reached the last cell of its route.
/// @param robot the robot
[[nodiscard]] bool atGoal(const Robot& robot) {
    return robot.routeIndex >= robot.route.steps();
}

/// Returns true when every robot has reached the last cell of its route.
/// @param robots the fleet
[[nodiscard]] bool allAtGoal(const std::vector<Robot>& robots) {
    return std::ranges::all_of(robots, [](const Robot& robot) { return atGoal(robot); });
}

/// Decides whether the pass-order rule lets one robot enter a cell this step.
/// The cell must be free after the step, no robot may enter the cell the mover
/// leaves, and every robot scheduled to pass the cell first must have left it.
/// @param mover index of the moving robot
/// @param target the cell the mover enters
/// @param arrivalIndex the plan step at which the mover arrives
/// @param currentIndex every robot's route index now
/// @param nextIndex every robot's route index after the step
/// @param advancing whether each robot advances this step
/// @param robots the fleet
/// @param schedule the planned visits per cell
[[nodiscard]] bool passOrderAllows(int mover, GridCell target, int arrivalIndex,
                                   const std::vector<int>& currentIndex,
                                   const std::vector<int>& nextIndex,
                                   const std::vector<bool>& advancing,
                                   const std::vector<Robot>& robots, const CellSchedule& schedule) {
    const GridCell origin = robots[static_cast<std::size_t>(mover)].route.cellAt(
        currentIndex[static_cast<std::size_t>(mover)]);

    for (std::size_t agent = 0; agent < robots.size(); ++agent) {
        if (static_cast<int>(agent) == mover) {
            continue;
        }
        const GridCell endCell = robots[agent].route.cellAt(nextIndex[agent]);
        if (endCell == target) {
            return false;
        }
        const GridCell startCell = robots[agent].route.cellAt(currentIndex[agent]);
        if (advancing[agent] && startCell == target && endCell == origin) {
            return false;
        }
    }

    const auto found = schedule.find(target);
    if (found == schedule.end()) {
        return true;
    }
    return std::ranges::all_of(
        found->second, [mover, arrivalIndex, &nextIndex](const CellInterval& interval) {
            return interval.agent == mover || interval.first >= arrivalIndex ||
                   nextIndex[static_cast<std::size_t>(interval.agent)] > interval.last;
        });
}

/// Appends the current positions as the next trajectory frame.
/// @param log the trajectory to extend
/// @param robots the fleet
void appendFrame(judge::RunLog& log, const std::vector<Robot>& robots) {
    judge::Frame frame;
    frame.reserve(robots.size());
    for (const Robot& robot : robots) {
        frame.push_back(positionOf(robot));
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
    for (int agent = 0; agent < fleetSize; ++agent) {
        robots[static_cast<std::size_t>(agent)].problem = agents[static_cast<std::size_t>(agent)];
    }

    const std::optional<Plan> initial = planner.plan(map, agents);
    if (!initial.has_value()) {
        result.initialPlanFailed = true;
        result.verdict = judge::analyse(result.log);
        return result;
    }
    for (int agent = 0; agent < fleetSize; ++agent) {
        const auto index = static_cast<std::size_t>(agent);
        robots[index].route = initial->routes[index];
        robots[index].routeIndex = 0;
    }
    CellSchedule schedule = buildSchedule(robots);

    std::mt19937 rng(config.seed);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    int planStartStep = 0;
    int lastReplanStep = -config.replanCooldown - 1;
    int stagnantSteps = 0;

    appendFrame(result.log, robots);

    int step = 0;
    while (step < config.maxSteps) {
        if (allAtGoal(robots)) {
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
            if (atGoal(robot)) {
                continue;
            }
            const int candidate = robot.routeIndex + 1;
            if (robot.route.cellAt(candidate) == positionOf(robot)) {
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
                    const GridCell target = cellAtIndex(robots[index], nextIndex[index]);
                    if (!passOrderAllows(agent, target, nextIndex[index], currentIndex, nextIndex,
                                         willAdvance, robots, schedule)) {
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
            const GridCell before = positionOf(robots[index]);
            robots[index].routeIndex = nextIndex[index];
            if (positionOf(robots[index]) != before) {
                ++moved;
            }
        }

        appendFrame(result.log, robots);
        stagnantSteps = moved == 0 ? stagnantSteps + 1 : 0;

        if (stagnantSteps >= config.deadlockPatience) {
            result.deadlock = true;
            break;
        }

        bool behind = false;
        const int expectedIndex = step - planStartStep;
        for (const Robot& robot : robots) {
            if (!atGoal(robot) && expectedIndex - robot.routeIndex > config.replanBehindSlack) {
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
            current.push_back(Agent{agents[static_cast<std::size_t>(agent)].id,
                                    positionOf(robots[static_cast<std::size_t>(agent)]),
                                    agents[static_cast<std::size_t>(agent)].goal});
        }
        const std::optional<Plan> replanned = planner.plan(map, current);
        if (!replanned.has_value()) {
            ++result.replanFailures;
            continue;
        }
        for (int agent = 0; agent < fleetSize; ++agent) {
            const auto index = static_cast<std::size_t>(agent);
            robots[index].route = replanned->routes[index];
            robots[index].routeIndex = 0;
        }
        schedule = buildSchedule(robots);
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
