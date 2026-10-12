// Implements prioritized planning declared in prioritized_planner.hpp.

#include "lotse/core/prioritized_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <optional>
#include <random>
#include <utility>
#include <vector>

#include "lotse/core/space_time_search.hpp"

namespace lotse::core {
namespace {

/// Adds one finished route to the reservations, including the goal held until
/// the horizon so no later robot routes through a finished robot. When
/// oneStepGap is set the cell a robot leaves is also held for one extra step, so
/// a real-time follower cannot enter it in the step the robot moves out.
/// @param reserved the reservation set to extend
/// @param route the route to reserve
/// @param horizon the last plan step to reserve
/// @param oneStepGap true to hold each vacated cell for one more plan step
void reserveRoute(Reservations& reserved, const Route& route, int horizon, bool oneStepGap) {
    for (int time = 0; time <= route.steps(); ++time) {
        reserved.reserveCell(route.cellAt(time), time);
    }
    for (int time = 0; time < route.steps(); ++time) {
        const GridCell from = route.cellAt(time);
        const GridCell to = route.cellAt(time + 1);
        reserved.reserveMove(from, to, time);
        reserved.reserveMove(to, from, time);
        if (oneStepGap) {
            reserved.reserveCell(from, time + 1);
        }
    }
    for (int time = route.steps(); time <= horizon; ++time) {
        reserved.reserveCell(route.cellAt(route.steps()), time);
    }
}

/// Sorts an order by remaining distance, farthest or nearest first.
/// @param order the robot order to sort
/// @param agents the fleet
/// @param farthest true for farthest first, false for nearest first
void sortByDistance(std::vector<std::size_t>& order, const std::vector<Agent>& agents,
                    bool farthest) {
    std::sort(order.begin(), order.end(), [&agents, farthest](std::size_t lhs, std::size_t rhs) {
        const int left = manhattan_distance(agents[lhs].start, agents[lhs].goal);
        const int right = manhattan_distance(agents[rhs].start, agents[rhs].goal);
        if (left != right) {
            return farthest ? left > right : left < right;
        }
        return agents[lhs].id < agents[rhs].id;
    });
}

/// Builds the robot order for one planning attempt. Attempt 0 uses the caller's
/// order; later attempts cycle the fixed orders and then seeded shuffles.
/// @param attempt zero based attempt number
/// @param agents the fleet
/// @param base the order chosen in the options
[[nodiscard]] std::vector<std::size_t> attemptOrder(int attempt, const std::vector<Agent>& agents,
                                                    AgentOrder base) {
    std::vector<std::size_t> order(agents.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    if (attempt == 0) {
        if (base == AgentOrder::FarthestFirst) {
            sortByDistance(order, agents, true);
        } else if (base == AgentOrder::NearestFirst) {
            sortByDistance(order, agents, false);
        }
        return order;
    }
    if (attempt == 1) {
        return order;
    }
    if (attempt == 2) {
        sortByDistance(order, agents, false);
        return order;
    }
    if (attempt == 3) {
        sortByDistance(order, agents, true);
        return order;
    }
    std::mt19937 rng(0x5eedU + static_cast<std::uint32_t>(attempt));
    std::shuffle(order.begin(), order.end(), rng);
    return order;
}

/// Plans one attempt: routes robots in the given order, each avoiding the ones
/// already fixed. Reports a timeout separately from a routing failure.
/// @param map the warehouse grid
/// @param agents the fleet
/// @param order the robot order for this attempt
/// @param horizon the last plan step
/// @param deadline the wall-clock deadline
/// @param timedOut set to true when the deadline is reached
/// @param oneStepGap true to hold each vacated cell for one more plan step
[[nodiscard]] std::optional<Plan> planInOrder(const Map& map, const std::vector<Agent>& agents,
                                              const std::vector<std::size_t>& order, int horizon,
                                              const std::chrono::steady_clock::time_point& deadline,
                                              bool& timedOut, bool oneStepGap) {
    Reservations reserved;
    Plan plan;
    plan.routes.resize(agents.size());
    for (const std::size_t index : order) {
        if (std::chrono::steady_clock::now() >= deadline) {
            timedOut = true;
            return std::nullopt;
        }
        const SearchLimits limits{horizon, 2000000};
        const std::optional<Route> route = searchRoute(map, agents[index], reserved, limits);
        if (!route.has_value()) {
            return std::nullopt;
        }
        reserveRoute(reserved, *route, horizon, oneStepGap);
        plan.routes[index] = *route;
    }
    return plan;
}

}  // namespace

int continuousHorizon(const Map& map, int fleetSize) {
    const int legs = 2 * (map.width() + map.height());
    const int slack = (4 * fleetSize) + 16;
    return legs + slack;
}

PrioritizedPlanner::PrioritizedPlanner(PrioritizedOptions options) : options_(options) {}

std::optional<Plan> PrioritizedPlanner::plan(const Map& map, const std::vector<Agent>& agents) {
    lastTimedOut_ = false;
    lastOrder_.clear();
    lastAttempt_ = -1;
    const int horizon = horizonFor(map, agents);
    const auto deadline =
        options_.timeLimitMs > 0
            ? std::chrono::steady_clock::now() + std::chrono::milliseconds(options_.timeLimitMs)
            : std::chrono::steady_clock::time_point::max();
    const int attempts = options_.attempts > 1 ? options_.attempts : 1;

    for (int attempt = 0; attempt < attempts; ++attempt) {
        if (std::chrono::steady_clock::now() >= deadline) {
            lastTimedOut_ = true;
            return std::nullopt;
        }
        const std::vector<std::size_t> order = attempt == 0 && options_.stableOrder
                                                   ? stableOrder(agents)
                                                   : attemptOrder(attempt, agents, options_.order);
        bool timedOut = false;
        std::optional<Plan> planned =
            planInOrder(map, agents, order, horizon, deadline, timedOut, options_.oneStepGap);
        if (planned.has_value()) {
            lastOrder_.clear();
            lastOrder_.reserve(order.size());
            for (const std::size_t index : order) {
                lastOrder_.push_back(agents[index].id);
            }
            lastAttempt_ = attempt;
            if (options_.stableOrder) {
                stable_.clear();
                stable_.reserve(order.size());
                for (const std::size_t index : order) {
                    stable_.emplace_back(agents[index].id, agents[index].goal);
                }
            }
            return planned;
        }
        if (timedOut) {
            lastTimedOut_ = true;
            return std::nullopt;
        }
    }
    return std::nullopt;
}

std::string PrioritizedPlanner::name() const {
    return "prioritized";
}

bool PrioritizedPlanner::lastCallTimedOut() const noexcept {
    return lastTimedOut_;
}

std::vector<int> PrioritizedPlanner::lastPlanningOrder() const {
    return lastOrder_;
}

int PrioritizedPlanner::lastPlanningAttempt() const noexcept {
    return lastAttempt_;
}

std::vector<std::size_t> PrioritizedPlanner::stableOrder(const std::vector<Agent>& agents) const {
    std::vector<std::size_t> order;
    std::vector<bool> placed(agents.size(), false);
    for (const auto& [id, goal] : stable_) {
        for (std::size_t index = 0; index < agents.size(); ++index) {
            if (!placed[index] && agents[index].id == id && agents[index].goal == goal) {
                order.push_back(index);
                placed[index] = true;
                break;
            }
        }
    }
    std::vector<std::size_t> rest;
    for (std::size_t index = 0; index < agents.size(); ++index) {
        if (!placed[index]) {
            rest.push_back(index);
        }
    }
    sortByDistance(rest, agents, true);
    order.insert(order.end(), rest.begin(), rest.end());
    return order;
}

int PrioritizedPlanner::horizonFor(const Map& map, const std::vector<Agent>& agents) const {
    long totalDistance = 0;
    for (const Agent& agent : agents) {
        totalDistance += manhattan_distance(agent.start, agent.goal);
    }
    const long slack = (4L * static_cast<long>(agents.size())) + static_cast<long>(map.width()) +
                       static_cast<long>(map.height()) + 16L;
    const long horizon = (4L * totalDistance) + slack + static_cast<long>(options_.horizonSlack);
    if (options_.maxHorizon > 0 && horizon > static_cast<long>(options_.maxHorizon)) {
        return options_.maxHorizon;
    }
    return static_cast<int>(horizon < 1L ? 1L : horizon);
}

}  // namespace lotse::core
