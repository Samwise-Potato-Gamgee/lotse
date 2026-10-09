// Implements prioritized planning declared in prioritized_planner.hpp.

#include "lotse/core/prioritized_planner.hpp"

#include <algorithm>
#include <numeric>
#include <utility>

#include "lotse/core/space_time_search.hpp"

namespace lotse::core {
namespace {

/// Adds one finished route to the reservations, including the goal held until
/// the horizon so no later robot routes through a finished robot.
/// @param reserved the reservation set to extend
/// @param route the route to reserve
/// @param horizon the last plan step to reserve
void reserveRoute(Reservations& reserved, const Route& route, int horizon) {
    for (int time = 0; time <= route.steps(); ++time) {
        reserved.reserveCell(route.cellAt(time), time);
    }
    for (int time = 0; time < route.steps(); ++time) {
        const GridCell from = route.cellAt(time);
        const GridCell to = route.cellAt(time + 1);
        reserved.reserveMove(from, to, time);
        reserved.reserveMove(to, from, time);
    }
    for (int time = route.steps(); time <= horizon; ++time) {
        reserved.reserveCell(route.cellAt(route.steps()), time);
    }
}

}  // namespace

PrioritizedPlanner::PrioritizedPlanner(PrioritizedOptions options) : options_(options) {}

std::optional<Plan> PrioritizedPlanner::plan(const Map& map, const std::vector<Agent>& agents) {
    const int horizon = horizonFor(map, agents);

    std::vector<std::size_t> order(agents.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    if (options_.order == AgentOrder::FarthestFirst) {
        std::sort(order.begin(), order.end(), [&agents](std::size_t lhs, std::size_t rhs) {
            const int left = manhattan_distance(agents[lhs].start, agents[lhs].goal);
            const int right = manhattan_distance(agents[rhs].start, agents[rhs].goal);
            if (left != right) {
                return left > right;
            }
            return agents[lhs].id < agents[rhs].id;
        });
    } else if (options_.order == AgentOrder::NearestFirst) {
        std::sort(order.begin(), order.end(), [&agents](std::size_t lhs, std::size_t rhs) {
            const int left = manhattan_distance(agents[lhs].start, agents[lhs].goal);
            const int right = manhattan_distance(agents[rhs].start, agents[rhs].goal);
            if (left != right) {
                return left < right;
            }
            return agents[lhs].id < agents[rhs].id;
        });
    }

    Reservations reserved;
    Plan plan;
    plan.routes.resize(agents.size());

    for (const std::size_t index : order) {
        const SearchLimits limits{horizon, 2000000};
        const std::optional<Route> route = searchRoute(map, agents[index], reserved, limits);
        if (!route.has_value()) {
            return std::nullopt;
        }
        reserveRoute(reserved, *route, horizon);
        plan.routes[index] = *route;
    }
    return plan;
}

std::string PrioritizedPlanner::name() const {
    return "prioritized";
}

int PrioritizedPlanner::horizonFor(const Map& map, const std::vector<Agent>& agents) const {
    long totalDistance = 0;
    for (const Agent& agent : agents) {
        totalDistance += manhattan_distance(agent.start, agent.goal);
    }
    const long slack = (4L * static_cast<long>(agents.size())) + static_cast<long>(map.width()) +
                       static_cast<long>(map.height()) + 16L;
    const long horizon = (4L * totalDistance) + slack + static_cast<long>(options_.horizonSlack);
    return static_cast<int>(horizon < 1L ? 1L : horizon);
}

}  // namespace lotse::core
