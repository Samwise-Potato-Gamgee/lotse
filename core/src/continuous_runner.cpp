// Implements continuous operation declared in continuous_runner.hpp.

#include "lotse/core/continuous_runner.hpp"

#include <algorithm>
#include <cstddef>
#include <numeric>
#include <optional>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

#include "lotse/core/pass_order.hpp"

namespace lotse::core {
namespace {

/// What a robot is doing right now.
enum class RobotState : std::uint8_t { Parked, ToParking, ToPickup, ToDropoff };

/// A robot during a continuous run.
struct Robot {
    int routeIndex = 0;
    RobotState state = RobotState::ToParking;
    bool hasOrder = false;
    Order order{};
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

/// Returns the cell a robot is currently trying to reach.
/// @param robot the robot
/// @param parking the robot's parking cell
[[nodiscard]] GridCell goalOf(const Robot& robot, GridCell parking) {
    if (!robot.hasOrder) {
        return parking;
    }
    return robot.state == RobotState::ToDropoff ? robot.order.dropoff : robot.order.pickup;
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

ContinuousResult runContinuous(const Map& map, const std::vector<GridCell>& robotStarts,
                               const std::vector<Order>& orders, Planner& planner,
                               TaskAssigner& assigner, const ContinuousConfig& config,
                               Planner* fallback) {
    ContinuousResult result;
    const auto fleetSize = static_cast<int>(robotStarts.size());
    result.log.agentCount = fleetSize;

    const std::vector<GridCell>& parking = map.parkingCells();
    if (parking.size() < robotStarts.size()) {
        throw std::runtime_error("continuous runner: fewer parking cells than robots");
    }

    std::vector<Order> stream = orders;
    std::sort(stream.begin(), stream.end(), [](const Order& a, const Order& b) {
        if (a.arrivalStep != b.arrivalStep) {
            return a.arrivalStep < b.arrivalStep;
        }
        return a.id < b.id;
    });

    std::vector<Robot> robots(static_cast<std::size_t>(fleetSize));
    std::vector<Route> routes(static_cast<std::size_t>(fleetSize));
    for (int robot = 0; robot < fleetSize; ++robot) {
        const auto index = static_cast<std::size_t>(robot);
        routes[index] = Route{robot, {robotStarts[index]}};
        robots[index].state =
            robotStarts[index] == parking[index] ? RobotState::Parked : RobotState::ToParking;
    }

    std::mt19937 rng(config.seed);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    std::vector<Order> pending;
    std::size_t nextOrder = 0;

    int step = 0;
    int stagnantSteps = 0;
    int stepsSincePlan = config.windowSteps;
    int lastPlanStep = -config.replanCooldown - 1;
    bool needsReplan = true;
    std::vector<int> waits;
    CellSchedule schedule = buildCellSchedule(routes);

    auto releaseOrders = [&stream, &pending, &nextOrder](int uptoStep) {
        while (nextOrder < stream.size() && stream[nextOrder].arrivalStep <= uptoStep) {
            pending.push_back(stream[nextOrder]);
            ++nextOrder;
        }
    };

    auto recordWait = [&result, &waits](int wait, const Order& order) {
        waits.push_back(wait);
        if (wait > result.longestWaitSteps) {
            result.longestWaitSteps = wait;
            result.longestWaitPriority = order.priority;
            result.longestWaitOrderId = order.id;
        }
    };

    auto cellAvailable = [&](GridCell cell, int self) {
        for (int other = 0; other < fleetSize; ++other) {
            if (other == self) {
                continue;
            }
            const auto index = static_cast<std::size_t>(other);
            if (positionOf(robots[index], routes[index]) == cell) {
                return false;
            }
            if (robots[index].hasOrder && goalOf(robots[index], parking[index]) == cell) {
                return false;
            }
        }
        return true;
    };

    auto assignOrders = [&]() {
        std::vector<int> freeRobots;
        std::vector<GridCell> positions(static_cast<std::size_t>(fleetSize));
        for (int robot = 0; robot < fleetSize; ++robot) {
            const auto index = static_cast<std::size_t>(robot);
            positions[index] = positionOf(robots[index], routes[index]);
            if (!robots[index].hasOrder) {
                freeRobots.push_back(robot);
            }
        }
        if (pending.empty() || freeRobots.empty()) {
            return;
        }

        std::vector<std::size_t> order(pending.size());
        std::iota(order.begin(), order.end(), std::size_t{0});
        const int ageing = config.priorityAgeingInterval;
        std::sort(order.begin(), order.end(), [&](std::size_t lhs, std::size_t rhs) {
            const long long left = effectivePriority(pending[lhs], step, ageing);
            const long long right = effectivePriority(pending[rhs], step, ageing);
            if (left != right) {
                return left > right;
            }
            return pending[lhs].id < pending[rhs].id;
        });

        std::vector<bool> remove(pending.size(), false);
        for (const std::size_t candidate : order) {
            if (freeRobots.empty()) {
                break;
            }
            const Order offer = pending[candidate];
            const std::optional<int> chosen =
                assigner.chooseRobot(map, offer, positions, freeRobots);
            if (!chosen.has_value()) {
                ++result.droppedOrders;
                recordWait(step - offer.arrivalStep, offer);
                remove[candidate] = true;
                continue;
            }
            const int robot = chosen.value();
            if (!cellAvailable(offer.pickup, robot) || !cellAvailable(offer.dropoff, robot)) {
                continue;
            }
            robots[static_cast<std::size_t>(robot)].hasOrder = true;
            robots[static_cast<std::size_t>(robot)].order = offer;
            robots[static_cast<std::size_t>(robot)].state = RobotState::ToPickup;
            needsReplan = true;
            remove[candidate] = true;
            std::erase(freeRobots, robot);
        }

        std::vector<Order> kept;
        kept.reserve(pending.size());
        for (std::size_t index = 0; index < pending.size(); ++index) {
            if (!remove[index]) {
                kept.push_back(pending[index]);
            }
        }
        pending = std::move(kept);
    };

    auto planAhead = [&]() {
        if (!needsReplan && stepsSincePlan < config.windowSteps) {
            return;
        }
        if (step - lastPlanStep < config.replanCooldown) {
            return;
        }

        std::vector<GridCell> blocked;
        std::vector<int> agentRobots;
        std::vector<Agent> agents;
        for (int robot = 0; robot < fleetSize; ++robot) {
            const auto index = static_cast<std::size_t>(robot);
            const GridCell position = positionOf(robots[index], routes[index]);
            if (robots[index].state == RobotState::Parked) {
                blocked.push_back(position);
                continue;
            }
            agentRobots.push_back(robot);
            agents.push_back(Agent{robot, position, goalOf(robots[index], parking[index])});
        }

        lastPlanStep = step;
        stepsSincePlan = 0;
        if (agents.empty()) {
            needsReplan = false;
            return;
        }

        const Map planningMap = map.withObstacles(blocked);
        ++result.planningCalls;
        std::optional<Plan> planned = planner.plan(planningMap, agents);
        if (!planned.has_value() && fallback != nullptr) {
            ++result.fallbacks;
            if (planner.lastCallTimedOut()) {
                ++result.fallbackTimeouts;
            }
            planned = fallback->plan(planningMap, agents);
        }
        if (!planned.has_value()) {
            ++result.planningFailures;
            needsReplan = true;
            return;
        }
        for (std::size_t k = 0; k < agentRobots.size(); ++k) {
            const auto robot = static_cast<std::size_t>(agentRobots[k]);
            routes[robot] = planned->routes[k];
            robots[robot].routeIndex = 0;
        }
        schedule = buildCellSchedule(routes);
        needsReplan = false;
    };

    auto advance = [&]() {
        std::vector<bool> willAdvance(static_cast<std::size_t>(fleetSize), false);
        std::vector<int> nextIndex(static_cast<std::size_t>(fleetSize), 0);
        std::vector<int> currentIndex(static_cast<std::size_t>(fleetSize), 0);
        for (int robot = 0; robot < fleetSize; ++robot) {
            const auto index = static_cast<std::size_t>(robot);
            Robot& unit = robots[index];
            const double draw = uniform(rng);
            currentIndex[index] = unit.routeIndex;
            nextIndex[index] = unit.routeIndex;
            if (atGoal(unit, routes[index])) {
                continue;
            }
            const int candidate = unit.routeIndex + 1;
            if (routes[index].cellAt(candidate) == positionOf(unit, routes[index])) {
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
                for (int robot = 0; robot < fleetSize; ++robot) {
                    const auto index = static_cast<std::size_t>(robot);
                    if (!willAdvance[index] || nextIndex[index] == currentIndex[index]) {
                        continue;
                    }
                    const GridCell target = routes[index].cellAt(nextIndex[index]);
                    if (!passOrderAllows(robot, target, nextIndex[index], currentIndex, nextIndex,
                                         willAdvance, routes, schedule)) {
                        nextIndex[index] = currentIndex[index];
                        willAdvance[index] = false;
                        changed = true;
                    }
                }
            }
        }

        int moved = 0;
        for (int robot = 0; robot < fleetSize; ++robot) {
            const auto index = static_cast<std::size_t>(robot);
            if (!willAdvance[index]) {
                continue;
            }
            const GridCell before = positionOf(robots[index], routes[index]);
            robots[index].routeIndex = nextIndex[index];
            if (positionOf(robots[index], routes[index]) != before) {
                ++moved;
            }
        }
        ++stepsSincePlan;
        return moved;
    };

    auto handleArrivals = [&]() {
        for (int robot = 0; robot < fleetSize; ++robot) {
            const auto index = static_cast<std::size_t>(robot);
            Robot& unit = robots[index];
            if (!atGoal(unit, routes[index])) {
                continue;
            }
            if (unit.hasOrder && unit.state == RobotState::ToPickup) {
                if (cellAvailable(unit.order.dropoff, robot)) {
                    unit.state = RobotState::ToDropoff;
                    needsReplan = true;
                }
            } else if (unit.hasOrder && unit.state == RobotState::ToDropoff) {
                const int wait = step - unit.order.arrivalStep;
                recordWait(wait, unit.order);
                ++result.ordersCompleted;
                unit.hasOrder = false;
                unit.state = RobotState::ToParking;
                needsReplan = true;
            } else if (!unit.hasOrder && unit.state == RobotState::ToParking) {
                unit.state = RobotState::Parked;
            }
        }
    };

    auto workDelivered = [&]() {
        if (nextOrder < stream.size() || !pending.empty()) {
            return false;
        }
        return std::ranges::none_of(robots, [](const Robot& robot) { return robot.hasOrder; });
    };

    auto allWorkDone = [&]() {
        if (!workDelivered()) {
            return false;
        }
        return std::ranges::all_of(
            robots, [](const Robot& robot) { return robot.state == RobotState::Parked; });
    };

    appendFrame(result.log, robots, routes);

    while (step < config.maxSteps) {
        releaseOrders(step);
        assignOrders();
        planAhead();

        ++step;
        const int moved = advance();
        handleArrivals();
        appendFrame(result.log, robots, routes);

        const bool outstanding =
            !pending.empty() || std::ranges::any_of(robots, [](const Robot& robot) {
                return robot.state != RobotState::Parked;
            });
        if (outstanding && moved == 0) {
            ++stagnantSteps;
        } else {
            stagnantSteps = 0;
        }
        if (stagnantSteps >= config.deadlockPatience) {
            result.deadlock = true;
            break;
        }
        if (stagnantSteps >= config.stuckPatience) {
            needsReplan = true;
        }

        if (allWorkDone()) {
            result.allOrdersCompleted = true;
            break;
        }
        if (workDelivered()) {
            result.allOrdersCompleted = true;
        }
    }

    result.stepsRun = step;
    result.ordersReleased = static_cast<int>(nextOrder);
    result.pendingAtEnd = static_cast<int>(pending.size());
    for (const Order& order : pending) {
        recordWait(step - order.arrivalStep, order);
    }
    pending.clear();

    if (!waits.empty()) {
        std::sort(waits.begin(), waits.end());
        long long sum = 0;
        for (const int wait : waits) {
            sum += wait;
        }
        result.meanWaitSteps = static_cast<double>(sum) / static_cast<double>(waits.size());
        result.waitTotalSteps = sum;
        result.waitCount = static_cast<int>(waits.size());
        result.medianWaitSteps = waits[waits.size() / 2U];
        result.percentile95WaitSteps =
            waits[static_cast<std::size_t>(0.95 * static_cast<double>(waits.size() - 1U))];
    }
    result.verdict = judge::analyse(result.log);
    return result;
}

}  // namespace lotse::core
