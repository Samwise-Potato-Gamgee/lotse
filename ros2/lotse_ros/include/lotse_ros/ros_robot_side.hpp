// The ROS 2 implementation of the core robot side. It publishes the cells a
// robot may enter as a nav_msgs Path on <ns>/waypoints and reads each driver's
// position and progress from a compact status string on <ns>/driver_status.

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "lotse/core/robot_side.hpp"

namespace lotse::ros {

/// Drives a fleet of ROS 2 robots through the core RobotSide interface.
class RosRobotSide final : public core::RobotSide {
public:
    /// Creates one publisher and one status listener per robot.
    /// @param node the node that owns the ROS entities
    /// @param namespaces the per-robot namespaces, e.g. "robot_0"
    /// @param startCells the start cell of every robot, indexed by robot id
    /// @param cellSize metres per grid cell
    RosRobotSide(rclcpp::Node& node, std::vector<std::string> namespaces,
                 std::vector<core::GridCell> startCells, double cellSize);

    /// Returns the number of robots.
    [[nodiscard]] int robotCount() const override;

    /// Returns the start cell of every robot.
    [[nodiscard]] std::vector<core::GridCell> starts() const override;

    /// Publishes the ordered cells a robot may enter as a Path.
    /// @param robot the robot id
    /// @param waypoints the allowed cells, in order
    void setAllowedWaypoints(int robot, const std::vector<core::GridCell>& waypoints) override;

    /// Returns the latest status of every robot.
    [[nodiscard]] std::vector<core::RobotStatus> statuses() const override;

    /// Returns immediately; the robots run in their own nodes and processes.
    /// @param seconds how much simulated time passes, unused
    void advance(double seconds) override;

private:
    /// Parses one driver status string into the stored status.
    /// @param robot the robot id
    /// @param message the status text: cell and target cells, progress, stopped
    void onStatus(int robot, const std_msgs::msg::String& message);

    rclcpp::Node& node_;
    std::vector<std::string> namespaces_;
    std::vector<core::GridCell> starts_;
    double cell_size_ = 1.0;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> publishers_;
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_;
    mutable std::mutex mutex_;
    std::vector<core::RobotStatus> statuses_;
};

}  // namespace lotse::ros
