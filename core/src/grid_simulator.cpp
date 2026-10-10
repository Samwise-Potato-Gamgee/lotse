// Implements the grid simulator declared in grid_simulator.hpp.

#include "lotse/sim/grid_simulator.hpp"

#include <cstddef>
#include <stdexcept>
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

GridRobotSide::GridRobotSide(std::vector<core::GridCell> starts, GridRobotSideConfig config)
    : starts_(std::move(starts)), config_(config), rng_(config.seed) {
    if (config_.cellSeconds <= 0.0) {
        throw std::runtime_error("grid robot side: cellSeconds must be positive");
    }
    robots_.reserve(starts_.size());
    for (const core::GridCell start : starts_) {
        RobotState state;
        state.cell = start;
        state.target = start;
        robots_.push_back(std::move(state));
    }
}

int GridRobotSide::robotCount() const {
    return static_cast<int>(robots_.size());
}

std::vector<core::GridCell> GridRobotSide::starts() const {
    return starts_;
}

void GridRobotSide::setAllowedWaypoints(int robot, const std::vector<core::GridCell>& waypoints) {
    robots_.at(static_cast<std::size_t>(robot)).waypoints = waypoints;
}

std::vector<core::RobotStatus> GridRobotSide::statuses() const {
    std::vector<core::RobotStatus> result;
    result.reserve(robots_.size());
    for (std::size_t index = 0; index < robots_.size(); ++index) {
        const RobotState& robot = robots_[index];
        result.push_back(core::RobotStatus{static_cast<int>(index), robot.cell, robot.target,
                                           robot.progress, robot.stopped});
    }
    return result;
}

void GridRobotSide::advance(double seconds) {
    if (seconds <= 0.0) {
        return;
    }
    std::uniform_real_distribution<double> chance(0.0, 1.0);
    for (RobotState& robot : robots_) {
        if (robot.stopped) {
            robot.stopRemaining -= seconds;
            if (robot.stopRemaining <= 0.0) {
                robot.stopped = false;
                robot.stopRemaining = 0.0;
            }
            continue;
        }
        if (robot.target == robot.cell) {
            while (!robot.waypoints.empty() && robot.waypoints.front() == robot.cell) {
                robot.waypoints.erase(robot.waypoints.begin());
            }
            if (robot.waypoints.empty()) {
                continue;
            }
            const core::GridCell next = robot.waypoints.front();
            robot.waypoints.erase(robot.waypoints.begin());
            if (core::manhattan_distance(robot.cell, next) != 1) {
                continue;
            }
            robot.target = next;
            robot.progress = 0.0;
            robot.speedFactor = 1.0;
            if (config_.slowdownProbability > 0.0 && chance(rng_) < config_.slowdownProbability) {
                std::uniform_real_distribution<double> factor(config_.minSpeedFactor,
                                                              config_.slowestSpeedFactor);
                robot.speedFactor = factor(rng_);
            }
            if (config_.stopProbability > 0.0 && chance(rng_) < config_.stopProbability) {
                std::uniform_real_distribution<double> wait(0.0, config_.maxStopSeconds);
                robot.stopped = true;
                robot.stopRemaining = wait(rng_);
                continue;
            }
        }
        robot.progress += seconds / (config_.cellSeconds * robot.speedFactor);
        if (robot.progress >= 1.0) {
            robot.cell = robot.target;
            robot.target = robot.cell;
            robot.progress = 0.0;
        }
    }
}

}  // namespace lotse::sim
