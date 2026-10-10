// The robot side interface: one boundary between the fleet manager and whatever
// drives the robots. The manager sends the cells a robot may enter and reads
// back each robot's position, progress and status. The grid simulator implements
// it first; the ROS 2 wrapper implements it in stage 6.

#pragma once

#include <vector>

#include "lotse/core/grid.hpp"

namespace lotse::core {

/// What a robot last reported to the manager.
struct RobotStatus {
    int id = 0;
    GridCell cell{};
    GridCell target{};
    double progress = 0.0;
    bool stopped = false;
};

/// The manager's view of the fleet hardware.
class RobotSide {
public:
    RobotSide() = default;
    RobotSide(const RobotSide&) = delete;
    RobotSide& operator=(const RobotSide&) = delete;
    RobotSide(RobotSide&&) = delete;
    RobotSide& operator=(RobotSide&&) = delete;
    virtual ~RobotSide() = default;

    /// Returns the number of robots under the manager.
    [[nodiscard]] virtual int robotCount() const = 0;

    /// Returns the cell every robot starts on, indexed by robot id.
    [[nodiscard]] virtual std::vector<GridCell> starts() const = 0;

    /// Replaces the ordered cells a robot may enter. A robot never moves to a
    /// cell that is not in this list.
    /// @param robot the robot id
    /// @param waypoints the allowed cells, in order
    virtual void setAllowedWaypoints(int robot, const std::vector<GridCell>& waypoints) = 0;

    /// Reads the latest status of every robot, indexed by robot id.
    [[nodiscard]] virtual std::vector<RobotStatus> statuses() const = 0;

    /// Lets the plant advance by a time slice. The grid simulator integrates its
    /// motion here; a real robot side returns immediately because the robots run
    /// in their own processes.
    /// @param seconds how much simulated time passes
    virtual void advance(double seconds) = 0;
};

}  // namespace lotse::core
