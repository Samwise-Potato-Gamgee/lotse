// The stand-in differential-drive simulator: it moves N robots from their
// velocity commands on <ns>/cmd_vel and publishes true positions on
// <ns>/odom and <ns>/ground_truth plus a three-beam LiDAR on <ns>/scan. Seeded
// slowdowns and stops inject delays, time_scale runs it faster than real time,
// and one robot can be given a short-lived obstacle in front.

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lotse/core/geometry.hpp"
#include "lotse/core/map.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>

namespace {

/// Moves a fleet of differential-drive robots from their velocity commands.
class StandinSim final : public rclcpp::Node {
public:
    StandinSim() : rclcpp::Node("standin_sim") {
        const std::string map_path = declare_parameter<std::string>("map_path", "");
        const int fleet_size = static_cast<int>(declare_parameter<std::int64_t>("fleet_size", 4));
        cell_size_ = declare_parameter<double>("cell_size", 1.0);
        rate_hz_ = declare_parameter<double>("rate_hz", 50.0);
        time_scale_ = declare_parameter<double>("time_scale", 1.0);
        const int seed = static_cast<int>(declare_parameter<std::int64_t>("seed", 1));
        slowdown_probability_ = declare_parameter<double>("slowdown_probability", 0.0);
        stop_probability_ = declare_parameter<double>("stop_probability", 0.0);
        slowdown_min_factor_ = declare_parameter<double>("slowdown_min_factor", 0.3);
        obstacle_robot_ = static_cast<int>(declare_parameter<std::int64_t>("obstacle_robot", -1));
        obstacle_start_ = declare_parameter<double>("obstacle_start", -1.0);
        obstacle_end_ = declare_parameter<double>("obstacle_end", -1.0);
        obstacle_distance_ = declare_parameter<double>("obstacle_distance", 0.35);

        const std::optional<lotse::core::Map> map = lotse::core::Map::fromFile(map_path);
        if (!map.has_value()) {
            throw std::runtime_error("standin_sim: cannot read map " + map_path);
        }
        if (static_cast<int>(map->parkingCells().size()) < fleet_size) {
            throw std::runtime_error("standin_sim: not enough parking cells for the fleet");
        }

        random_.seed(static_cast<std::uint32_t>(seed));
        for (int robot = 0; robot < fleet_size; ++robot) {
            const std::string name = "robot_" + std::to_string(robot);
            Robot state;
            state.pose = lotse::core::cellCentre(
                map->parkingCells().at(static_cast<std::size_t>(robot)), cell_size_);
            state.delay_remaining = 0.0;
            state.delay_factor = 1.0;
            robots_.push_back(state);

            command_subscriptions_.push_back(create_subscription<geometry_msgs::msg::Twist>(
                name + "/cmd_vel", 10,
                [this, robot](const geometry_msgs::msg::Twist::SharedPtr msg) {
                    robots_.at(static_cast<std::size_t>(robot)).linear = msg->linear.x;
                    robots_.at(static_cast<std::size_t>(robot)).angular = msg->angular.z;
                }));
            odom_publishers_.push_back(
                create_publisher<nav_msgs::msg::Odometry>(name + "/odom", 10));
            truth_publishers_.push_back(
                create_publisher<nav_msgs::msg::Odometry>(name + "/ground_truth", 10));
            scan_publishers_.push_back(
                create_publisher<sensor_msgs::msg::LaserScan>(name + "/scan", 10));
        }

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / std::max(rate_hz_, 1.0)));
        step_timer_ = create_wall_timer(period, [this] { step(); });
    }

private:
    struct Robot {
        lotse::core::Pose2D pose{};
        double linear = 0.0;
        double angular = 0.0;
        double delay_remaining = 0.0;
        double delay_factor = 1.0;
    };

    /// Advances every robot by one time slice and publishes its state.
    void step() {
        const double dt = time_scale_ / std::max(rate_hz_, 1.0);
        elapsed_ += dt;
        for (std::size_t index = 0; index < robots_.size(); ++index) {
            Robot& robot = robots_[index];
            updateDelay(robot, dt);
            robot.pose =
                lotse::core::integrateDiffDrive(robot.pose, robot.linear * robot.delay_factor,
                                                robot.angular * robot.delay_factor, dt);
            publishState(static_cast<int>(index), robot);
        }
    }

    /// Draws a new seeded delay when the previous one ends.
    /// @param robot the robot state
    /// @param dt the time slice
    void updateDelay(Robot& robot, double dt) {
        robot.delay_remaining -= dt;
        if (robot.delay_remaining > 0.0) {
            return;
        }
        std::uniform_real_distribution<double> chance(0.0, 1.0);
        std::uniform_real_distribution<double> length(0.4, 1.8);
        robot.delay_factor = 1.0;
        robot.delay_remaining = length(random_);
        if (stop_probability_ > 0.0 && chance(random_) < stop_probability_) {
            robot.delay_factor = 0.0;
        } else if (slowdown_probability_ > 0.0 && chance(random_) < slowdown_probability_) {
            std::uniform_real_distribution<double> factor(slowdown_min_factor_, 0.9);
            robot.delay_factor = factor(random_);
        }
    }

    /// Publishes odometry, ground truth and one LiDAR frame.
    /// @param robot the robot id
    /// @param state the robot state
    void publishState(int robot, const Robot& state) {
        const auto slot = static_cast<std::size_t>(robot);
        nav_msgs::msg::Odometry message;
        message.header.frame_id = "map";
        message.header.stamp = now();
        message.child_frame_id = "base_link";
        message.pose.pose.position.x = state.pose.x;
        message.pose.pose.position.y = state.pose.y;
        message.pose.pose.orientation.z = std::sin(state.pose.theta / 2.0);
        message.pose.pose.orientation.w = std::cos(state.pose.theta / 2.0);
        message.twist.twist.linear.x = state.linear * state.delay_factor;
        message.twist.twist.angular.z = state.angular * state.delay_factor;
        odom_publishers_.at(slot)->publish(message);
        truth_publishers_.at(slot)->publish(message);

        sensor_msgs::msg::LaserScan scan;
        scan.header.frame_id = "base_link";
        scan.header.stamp = message.header.stamp;
        scan.angle_min = -0.4F;
        scan.angle_max = 0.4F;
        scan.angle_increment = 0.4F;
        scan.range_min = 0.05F;
        scan.range_max = 10.0F;
        scan.ranges = {scan.range_max, scan.range_max, scan.range_max};
        if (robot == obstacle_robot_ && elapsed_ >= obstacle_start_ && elapsed_ < obstacle_end_) {
            scan.ranges[1] = static_cast<float>(obstacle_distance_);
        }
        scan_publishers_.at(slot)->publish(scan);
    }

    double cell_size_ = 1.0;
    double time_scale_ = 1.0;
    double rate_hz_ = 50.0;
    double slowdown_probability_ = 0.0;
    double stop_probability_ = 0.0;
    double slowdown_min_factor_ = 0.3;
    int obstacle_robot_ = -1;
    double obstacle_start_ = -1.0;
    double obstacle_end_ = -1.0;
    double obstacle_distance_ = 0.35;
    double elapsed_ = 0.0;

    std::mt19937 random_;
    std::vector<Robot> robots_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> command_subscriptions_;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odom_publishers_;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> truth_publishers_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> scan_publishers_;
    rclcpp::TimerBase::SharedPtr step_timer_;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StandinSim>());
    rclcpp::shutdown();
    return 0;
}
