// The grid simulator plant. followPlanExactly replays a plan for the early
// benchmarks; GridRobotSide is the real-time robot side the manager drives,
// moving each robot toward the last allowed waypoint and injecting seeded
// slowdowns and stops.

#pragma once

#include <cstdint>
#include <random>
#include <vector>

#include "lotse/core/grid.hpp"
#include "lotse/core/plan.hpp"
#include "lotse/core/robot_side.hpp"
#include "lotse/judge/judge.hpp"

namespace lotse::sim {

/// Transcribes a plan into a trajectory, one plan step per frame.
/// @param plan the routes to follow
/// @param steps how many steps to simulate
/// @return the trajectory, frame 0 being the start state
[[nodiscard]] judge::RunLog followPlanExactly(const core::Plan& plan, int steps);

/// Tuning for the real-time grid simulator.
struct GridRobotSideConfig {
    double cellSeconds = 1.0;
    std::uint32_t seed = 1;
    double slowdownProbability = 0.0;
    double minSpeedFactor = 0.4;
    double slowestSpeedFactor = 1.0;
    double stopProbability = 0.0;
    double maxStopSeconds = 2.0;
};

/// A real-time grid plant. Each robot walks to the next allowed waypoint at a
/// speed in cells per second; delays and stops come from a fixed seed.
class GridRobotSide final : public core::RobotSide {
public:
    /// Creates the fleet on the given start cells.
    /// @param starts the start cell of every robot, indexed by robot id
    /// @param config cell time and seeded delay tuning
    GridRobotSide(std::vector<core::GridCell> starts, GridRobotSideConfig config = {});

    /// Returns the number of robots.
    [[nodiscard]] int robotCount() const override;

    /// Returns the start cell of every robot.
    [[nodiscard]] std::vector<core::GridCell> starts() const override;

    /// Replaces the cells a robot may enter.
    /// @param robot the robot id
    /// @param waypoints the allowed cells, in order
    void setAllowedWaypoints(int robot, const std::vector<core::GridCell>& waypoints) override;

    /// Returns the latest status of every robot.
    [[nodiscard]] std::vector<core::RobotStatus> statuses() const override;

    /// Integrates one time slice.
    /// @param seconds how much simulated time passes
    void advance(double seconds) override;

private:
    struct RobotState {
        core::GridCell cell{};
        core::GridCell target{};
        double progress = 0.0;
        double speedFactor = 1.0;
        double stopRemaining = 0.0;
        bool stopped = false;
        std::vector<core::GridCell> waypoints;
    };

    std::vector<RobotState> robots_;
    std::vector<core::GridCell> starts_;
    GridRobotSideConfig config_;
    std::mt19937 rng_;
};

}  // namespace lotse::sim
