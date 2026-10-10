// Implements the ROS 2 robot side declared in ros_robot_side.hpp.

#include "lotse_ros/ros_robot_side.hpp"

#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "lotse/core/geometry.hpp"

namespace lotse::ros {

RosRobotSide::RosRobotSide(rclcpp::Node& node, std::vector<std::string> namespaces,
                           std::vector<core::GridCell> startCells, double cellSize)
    : node_(node),
      namespaces_(std::move(namespaces)),
      starts_(std::move(startCells)),
      cell_size_(cellSize) {
    if (namespaces_.size() != starts_.size()) {
        throw std::runtime_error("ros robot side: namespaces and start cells differ in size");
    }
    statuses_.resize(starts_.size());
    for (std::size_t index = 0; index < starts_.size(); ++index) {
        statuses_[index] =
            core::RobotStatus{static_cast<int>(index), starts_[index], starts_[index], 0.0, false};
        publishers_.push_back(node_.create_publisher<nav_msgs::msg::Path>(
            namespaces_[index] + "/waypoints", rclcpp::QoS(1).transient_local()));
        subscriptions_.push_back(node_.create_subscription<std_msgs::msg::String>(
            namespaces_[index] + "/driver_status", 10,
            [this, index](const std_msgs::msg::String& message) {
                onStatus(static_cast<int>(index), message);
            }));
    }
}

int RosRobotSide::robotCount() const {
    return static_cast<int>(starts_.size());
}

std::vector<core::GridCell> RosRobotSide::starts() const {
    return starts_;
}

void RosRobotSide::setAllowedWaypoints(int robot, const std::vector<core::GridCell>& waypoints) {
    if (robot < 0 || static_cast<std::size_t>(robot) >= publishers_.size()) {
        return;
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_.get_clock()->now();
    for (const core::GridCell cell : waypoints) {
        const core::Pose2D centre = core::cellCentre(cell, cell_size_);
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = centre.x;
        pose.pose.position.y = centre.y;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }
    publishers_.at(static_cast<std::size_t>(robot))->publish(path);
}

std::vector<core::RobotStatus> RosRobotSide::statuses() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return statuses_;
}

void RosRobotSide::advance(double seconds) {
    (void)seconds;
}

void RosRobotSide::onStatus(int robot, const std_msgs::msg::String& message) {
    std::istringstream stream(message.data);
    int cell_x = 0;
    int cell_y = 0;
    int target_x = 0;
    int target_y = 0;
    double progress = 0.0;
    int stopped = 0;
    if (!(stream >> cell_x >> cell_y >> target_x >> target_y >> progress >> stopped)) {
        return;
    }
    const std::lock_guard<std::mutex> lock(mutex_);
    statuses_[static_cast<std::size_t>(robot)] =
        core::RobotStatus{robot, core::GridCell{cell_x, cell_y}, core::GridCell{target_x, target_y},
                          progress, stopped != 0};
}

}  // namespace lotse::ros
