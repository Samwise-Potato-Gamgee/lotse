// Implements the pass-order rule declared in pass_order.hpp.

#include "lotse/core/pass_order.hpp"

#include <algorithm>
#include <cstddef>

namespace lotse::core {

CellSchedule buildCellSchedule(const std::vector<Route>& routes) {
    CellSchedule schedule;
    for (std::size_t agent = 0; agent < routes.size(); ++agent) {
        const Route& route = routes[agent];
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

bool passOrderAllows(int mover, GridCell target, int arrivalIndex,
                     const std::vector<int>& currentIndex, const std::vector<int>& nextIndex,
                     const std::vector<bool>& advancing, const std::vector<Route>& routes,
                     const CellSchedule& schedule) {
    const GridCell origin = routes[static_cast<std::size_t>(mover)].cellAt(
        currentIndex[static_cast<std::size_t>(mover)]);

    for (std::size_t agent = 0; agent < routes.size(); ++agent) {
        if (static_cast<int>(agent) == mover) {
            continue;
        }
        const GridCell endCell = routes[agent].cellAt(nextIndex[agent]);
        if (endCell == target) {
            return false;
        }
        const GridCell startCell = routes[agent].cellAt(currentIndex[agent]);
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

}  // namespace lotse::core
