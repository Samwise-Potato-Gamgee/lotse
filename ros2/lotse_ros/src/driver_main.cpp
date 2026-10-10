// One driver node per robot: it follows the allowed waypoints from
// <ns>/waypoints with velocity commands on <ns>/cmd_vel, stops when the LiDAR
// on <ns>/scan sees something closer than stop_distance, and reports its cell,
// target and progress on <ns>/driver_status.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "lotse/core/geometry.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>

namespace {

/// Follows waypoints for one differential-drive robot.
class DriverNode final : public rclcpp::Node {
public:
    DriverNode() : rclcpp::Node("driver") {
        cell_size_ = declare_parameter<double>("cell_size", 1.0);
        stop_distance_ = declare_parameter<double>("stop_distance", 0.5);
        max_speed_ = declare_parameter<double>("max_speed", 0.7);
        turn_gain_ = declare_parameter<double>("turn_gain", 2.0);
        turn_limit_ = declare_parameter<double>("turn_limit", 1.6);
        goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.06);
        const double rate = declare_parameter<double>("control_rate_hz", 20.0);

        command_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        status_publisher_ = create_publisher<std_msgs::msg::String>("driver_status", 10);
        waypoint_subscription_ = create_subscription<nav_msgs::msg::Path>(
            "waypoints", rclcpp::QoS(1).transient_local(),
            [this](const nav_msgs::msg::Path::SharedPtr message) { onPath(*message); });
        odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr message) { onOdom(*message); });
        scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr message) { onScan(*message); });

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / std::max(rate, 1.0)));
        control_timer_ = create_wall_timer(period, [this] { control(); });
    }

private:
    /// Stores the new allowed waypoints and the leg they describe.
    /// @param path the Path published by the manager
    void onPath(const nav_msgs::msg::Path& path) {
        if (path.poses.size() < 2) {
            has_leg_ = false;
            return;
        }
        const auto& first = path.poses.front().pose.position;
        const auto& second = path.poses.back().pose.position;
        from_cell_ = lotse::core::cellAt(lotse::core::Pose2D{first.x, first.y, 0.0}, cell_size_);
        to_cell_ = lotse::core::cellAt(lotse::core::Pose2D{second.x, second.y, 0.0}, cell_size_);
        target_ = lotse::core::Pose2D{second.x, second.y, 0.0};
        has_leg_ = true;
        stopped_ = false;
    }

    /// Stores the latest odometry pose.
    /// @param odometry the odometry message
    void onOdom(const nav_msgs::msg::Odometry& odometry) {
        const auto& position = odometry.pose.pose.position;
        const auto& orientation = odometry.pose.pose.orientation;
        pose_.x = position.x;
        pose_.y = position.y;
        pose_.theta = std::atan2(2.0 * (orientation.w * orientation.z),
                                 1.0 - 2.0 * orientation.z * orientation.z);
        has_pose_ = true;
    }

    /// Stores the smallest LiDAR range.
    /// @param scan the laser scan message
    void onScan(const sensor_msgs::msg::LaserScan& scan) {
        float nearest = scan.range_max;
        for (const float range : scan.ranges) {
            if (std::isfinite(range) && range > scan.range_min && range < nearest) {
                nearest = range;
            }
        }
        front_range_ = nearest;
    }

    /// Publishes a velocity command.
    /// @param linear forward speed
    /// @param angular turn rate
    void publishCommand(double linear, double angular) {
        geometry_msgs::msg::Twist command;
        command.linear.x = linear;
        command.angular.z = angular;
        command_publisher_->publish(command);
    }

    /// Publishes the cell, target, progress and stopped flag.
    /// @param cell the cell the robot is leaving
    /// @param target the cell the robot is entering
    /// @param progress how far along the leg the robot is, 0..1
    void publishStatus(lotse::core::GridCell cell, lotse::core::GridCell target, double progress) {
        std::ostringstream stream;
        stream << cell.x << ' ' << cell.y << ' ' << target.x << ' ' << target.y << ' ' << progress
               << ' ' << (stopped_ ? 1 : 0);
        std_msgs::msg::String message;
        message.data = stream.str();
        status_publisher_->publish(message);
    }

    /// Runs one control step: stop for an obstacle, else follow the leg.
    void control() {
        if (!has_pose_) {
            return;
        }
        if (front_range_ < stop_distance_) {
            stopped_ = true;
            publishCommand(0.0, 0.0);
            if (has_leg_) {
                publishStatus(from_cell_, to_cell_,
                              lotse::core::legProgress(pose_, from_cell_, to_cell_, cell_size_));
            } else {
                const lotse::core::GridCell cell = lotse::core::cellAt(pose_, cell_size_);
                publishStatus(cell, cell, 0.0);
            }
            return;
        }
        stopped_ = false;
        if (!has_leg_) {
            const lotse::core::GridCell cell = lotse::core::cellAt(pose_, cell_size_);
            publishCommand(0.0, 0.0);
            publishStatus(cell, cell, 0.0);
            return;
        }

        const double dx = target_.x - pose_.x;
        const double dy = target_.y - pose_.y;
        const double distance = std::hypot(dx, dy);
        if (distance < goal_tolerance_) {
            has_leg_ = false;
            publishCommand(0.0, 0.0);
            publishStatus(to_cell_, to_cell_, 0.0);
            return;
        }
        const double error = lotse::core::headingError(std::atan2(dy, dx), pose_.theta);
        double linear = max_speed_ * std::cos(error);
        if (linear < 0.0 || std::abs(error) > 1.2) {
            linear = 0.0;
        }
        const double angular = std::clamp(turn_gain_ * error, -turn_limit_, turn_limit_);
        publishCommand(linear, angular);
        publishStatus(from_cell_, to_cell_,
                      lotse::core::legProgress(pose_, from_cell_, to_cell_, cell_size_));
    }

    double cell_size_ = 1.0;
    double stop_distance_ = 0.5;
    double max_speed_ = 0.7;
    double turn_gain_ = 2.0;
    double turn_limit_ = 1.6;
    double goal_tolerance_ = 0.06;

    bool has_leg_ = false;
    bool has_pose_ = false;
    bool stopped_ = false;
    lotse::core::GridCell from_cell_{};
    lotse::core::GridCell to_cell_{};
    lotse::core::Pose2D target_{};
    lotse::core::Pose2D pose_{};
    double front_range_ = 100.0;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriverNode>());
    rclcpp::shutdown();
    return 0;
}
