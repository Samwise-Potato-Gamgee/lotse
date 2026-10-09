// Implements the exact grid simulator declared in grid_simulator.hpp.

#include "lotse/sim/grid_simulator.hpp"

#include <cstddef>
#include <utility>

namespace lotse::sim {

judge::RunLog followPlanExactly(const core::Plan& plan, int steps) {
    judge::RunLog log;
    log.agentCount = plan.agentCount();
    const int lastStep = steps < 0 ? 0 : steps;

    for (int time = 0; time <= lastStep; ++time) {
        judge::Frame frame(static_cast<std::size_t>(log.agentCount));
        for (int agent = 0; agent < log.agentCount; ++agent) {
            const core::Route& route = plan.routes[static_cast<std::size_t>(agent)];
            frame[static_cast<std::size_t>(agent)] = route.cellAt(time);
        }
        log.frames.push_back(std::move(frame));
    }
    return log;
}

}  // namespace lotse::sim
