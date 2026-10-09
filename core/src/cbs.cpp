// Implements conflict-based search declared in cbs.hpp.

#include "lotse/core/cbs.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <queue>
#include <unordered_set>
#include <utility>
#include <vector>

#include "lotse/core/reservations.hpp"
#include "lotse/core/space_time_search.hpp"

namespace lotse::core {
namespace {

/// One constraint on one agent: a forbidden cell or a forbidden move at a step.
struct Constraint {
    bool edge = false;
    int time = 0;
    GridCell cell{};
    GridCell from{};
    GridCell to{};
};

/// Two routes that clash, and where.
struct Conflict {
    int first = 0;
    int second = 0;
    GridCell cell{};
    GridCell firstFrom{};
    GridCell firstTo{};
    GridCell secondFrom{};
    GridCell secondTo{};
    int time = 0;
    bool edge = false;
};

/// One node of the search: one route per agent plus that agent's constraints.
struct CbsNode {
    std::vector<Route> routes;
    std::vector<std::vector<Constraint>> constraints;
    int cost = 0;
};

using NodePtr = std::shared_ptr<CbsNode>;

/// Orders nodes by cost, cheapest first.
struct CheapestNodeFirst {
    /// Compares two nodes for the priority queue.
    /// @param lhs first node
    /// @param rhs second node
    [[nodiscard]] bool operator()(const NodePtr& lhs, const NodePtr& rhs) const noexcept {
        return lhs->cost > rhs->cost;
    }
};

/// Returns the largest number of steps of any route.
/// @param routes the routes to inspect
[[nodiscard]] int longestRoute(const std::vector<Route>& routes) {
    int longest = 0;
    for (const Route& route : routes) {
        longest = std::max(longest, route.steps());
    }
    return longest;
}

/// Finds the first vertex or edge conflict between any two routes.
/// @param routes every agent's route
/// @return the conflict, or nothing when the routes are conflict-free
[[nodiscard]] std::optional<Conflict> findFirstConflict(const std::vector<Route>& routes) {
    const int horizon = longestRoute(routes);
    for (int time = 0; time <= horizon; ++time) {
        for (std::size_t a = 0; a < routes.size(); ++a) {
            for (std::size_t b = a + 1; b < routes.size(); ++b) {
                if (routes[a].cellAt(time) == routes[b].cellAt(time)) {
                    Conflict conflict;
                    conflict.first = static_cast<int>(a);
                    conflict.second = static_cast<int>(b);
                    conflict.cell = routes[a].cellAt(time);
                    conflict.time = time;
                    return conflict;
                }
            }
        }
        if (time == horizon) {
            break;
        }
        for (std::size_t a = 0; a < routes.size(); ++a) {
            for (std::size_t b = a + 1; b < routes.size(); ++b) {
                const GridCell firstFrom = routes[a].cellAt(time);
                const GridCell firstTo = routes[a].cellAt(time + 1);
                const GridCell secondFrom = routes[b].cellAt(time);
                const GridCell secondTo = routes[b].cellAt(time + 1);
                if (firstFrom == secondTo && secondFrom == firstTo && firstFrom != firstTo) {
                    Conflict conflict;
                    conflict.first = static_cast<int>(a);
                    conflict.second = static_cast<int>(b);
                    conflict.firstFrom = firstFrom;
                    conflict.firstTo = firstTo;
                    conflict.secondFrom = secondFrom;
                    conflict.secondTo = secondTo;
                    conflict.time = time;
                    conflict.edge = true;
                    return conflict;
                }
            }
        }
    }
    return std::nullopt;
}

/// Searches one agent's route under that agent's constraints.
/// @param map the warehouse grid
/// @param agent the robot's start and goal
/// @param constraints the cells and moves forbidden for this agent
/// @param horizon the last plan step
/// @param maxExpansions the search expansion cap
[[nodiscard]] std::optional<Route> searchWithConstraints(const Map& map, const Agent& agent,
                                                         const std::vector<Constraint>& constraints,
                                                         int horizon, int maxExpansions) {
    Reservations reserved;
    for (const Constraint& constraint : constraints) {
        if (constraint.edge) {
            reserved.reserveMove(constraint.from, constraint.to, constraint.time);
        } else {
            reserved.reserveCell(constraint.cell, constraint.time);
        }
    }
    return searchRoute(map, agent, reserved, SearchLimits{horizon, maxExpansions});
}

/// Returns the total route length of a node's plan.
/// @param routes the routes to add up
[[nodiscard]] int totalCost(const std::vector<Route>& routes) {
    int cost = 0;
    for (const Route& route : routes) {
        cost += route.steps();
    }
    return cost;
}

/// Returns true when two agents share the same goal cell.
/// @param agents the fleet
[[nodiscard]] bool hasDuplicateGoals(const std::vector<Agent>& agents) {
    std::unordered_set<GridCell, GridCellHash> goals;
    for (const Agent& agent : agents) {
        if (!goals.insert(agent.goal).second) {
            return true;
        }
    }
    return false;
}

}  // namespace

CbsPlanner::CbsPlanner(CbsOptions options) : options_(options) {}

int CbsPlanner::horizonFor(const Map& map, const std::vector<Agent>& agents) const {
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

std::string CbsPlanner::name() const {
    return "cbs";
}

bool CbsPlanner::lastCallTimedOut() const noexcept {
    return lastTimedOut_;
}

std::optional<Plan> CbsPlanner::plan(const Map& map, const std::vector<Agent>& agents) {
    lastTimedOut_ = false;
    if (agents.empty()) {
        return Plan{};
    }

    const int horizon = horizonFor(map, agents);
    const int expansions = std::max(1, options_.maxNodes);
    if (hasDuplicateGoals(agents)) {
        return std::nullopt;
    }

    auto root = std::make_shared<CbsNode>();
    root->routes.resize(agents.size());
    root->constraints.resize(agents.size());
    for (std::size_t agent = 0; agent < agents.size(); ++agent) {
        const std::optional<Route> route =
            searchRoute(map, agents[agent], Reservations{}, SearchLimits{horizon, expansions});
        if (!route.has_value()) {
            return std::nullopt;
        }
        root->routes[agent] = *route;
    }
    root->cost = totalCost(root->routes);

    std::priority_queue<NodePtr, std::vector<NodePtr>, CheapestNodeFirst> open;
    open.push(root);

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(std::max(1, options_.timeLimitMs));
    int expanded = 0;
    while (!open.empty()) {
        if (std::chrono::steady_clock::now() >= deadline || expanded >= options_.maxNodes) {
            lastTimedOut_ = true;
            return std::nullopt;
        }

        const NodePtr node = open.top();
        open.pop();
        ++expanded;

        const std::optional<Conflict> conflict = findFirstConflict(node->routes);
        if (!conflict.has_value()) {
            Plan plan;
            plan.routes = node->routes;
            return plan;
        }

        const Conflict& found = conflict.value();
        for (const int agent : {found.first, found.second}) {
            auto child = std::make_shared<CbsNode>(*node);
            if (found.edge) {
                const bool isFirst = agent == found.first;
                child->constraints[static_cast<std::size_t>(agent)].push_back(Constraint{
                    true, found.time, GridCell{}, isFirst ? found.firstFrom : found.secondFrom,
                    isFirst ? found.firstTo : found.secondTo});
            } else {
                child->constraints[static_cast<std::size_t>(agent)].push_back(
                    Constraint{false, found.time, found.cell, GridCell{}, GridCell{}});
            }
            const std::optional<Route> route = searchWithConstraints(
                map, agents[static_cast<std::size_t>(agent)],
                child->constraints[static_cast<std::size_t>(agent)], horizon, expansions);
            if (!route.has_value()) {
                continue;
            }
            child->cost -= child->routes[static_cast<std::size_t>(agent)].steps();
            child->routes[static_cast<std::size_t>(agent)] = *route;
            child->cost += route->steps();
            open.push(child);
        }
    }
    return std::nullopt;
}

}  // namespace lotse::core
