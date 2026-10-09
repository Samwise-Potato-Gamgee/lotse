// Lotse stage 0 link check.
//
// Subscribes to the simulator's odometry, commands a forward velocity for a
// few seconds, and checks from odometry alone that the robot actually moved.
// The process exits with status 0 only when the measured travel is at least
// min_distance_m, which is what the shell wrapper uses as the pass/fail signal.
//
// This is deliberately small: it is the seed of the real Lotse robot driver
// node (stage 6), which will add waypoint following and a LiDAR stop.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

namespace lotse {

namespace {
using namespace std::chrono_literals;

constexpr auto kCmdPublishPeriod = 50ms;
}  // namespace

/// Drives one robot over ROS 2 and reports how far it travelled.
class LinkCheckNode : public rclcpp::Node {
public:
    LinkCheckNode() : rclcpp::Node("lotse_link_check") {
        forward_speed_ = declare_parameter("forward_speed", 0.3);
        drive_seconds_ = declare_parameter("drive_seconds", 3.0);
        settle_seconds_ = declare_parameter("settle_seconds", 1.0);
        wait_timeout_seconds_ = declare_parameter("wait_timeout_seconds", 120.0);
        min_distance_m_ = declare_parameter("min_distance_m", 0.25);
        odom_topic_ = declare_parameter("odom_topic", std::string("odom"));
        cmd_vel_topic_ = declare_parameter("cmd_vel_topic", std::string("cmd_vel"));
        selftest_ = declare_parameter("selftest", false);

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, [this](const nav_msgs::msg::Odometry& msg) {
                last_odom_ = msg;
                ++odom_count_;
            });
    }

    /// Runs the check and returns a process exit status.
    int run_check() {
        if (selftest_) {
            RCLCPP_INFO(get_logger(), "selftest ok: node up, publishing on '%s', listening on '%s'",
                        cmd_vel_topic_.c_str(), odom_topic_.c_str());
            return EXIT_SUCCESS;
        }

        RCLCPP_INFO(get_logger(), "waiting for odometry on '%s' (up to %.1f s)",
                    odom_topic_.c_str(), wait_timeout_seconds_);
        if (!wait_for_odom()) {
            RCLCPP_ERROR(get_logger(), "no odometry received on '%s'", odom_topic_.c_str());
            return EXIT_FAILURE;
        }

        const auto start = last_odom_.pose.pose.position;
        RCLCPP_INFO(get_logger(), "start position: (%.3f, %.3f, %.3f)", start.x, start.y, start.z);

        drive_for(drive_seconds_, forward_speed_);
        drive_for(settle_seconds_, 0.0);

        const auto end = last_odom_.pose.pose.position;
        const double dx = end.x - start.x;
        const double dy = end.y - start.y;
        const double distance = std::sqrt((dx * dx) + (dy * dy));

        RCLCPP_INFO(get_logger(), "end position: (%.3f, %.3f, %.3f)", end.x, end.y, end.z);
        RCLCPP_INFO(get_logger(), "RESULT odom_msgs=%zu distance_m=%.3f min_distance_m=%.3f",
                    odom_count_, distance, min_distance_m_);

        if (distance < min_distance_m_) {
            RCLCPP_ERROR(get_logger(), "robot moved only %.3f m, less than the required %.3f m",
                         distance, min_distance_m_);
            return EXIT_FAILURE;
        }
        RCLCPP_INFO(get_logger(), "link check passed: robot moved %.3f m", distance);
        return EXIT_SUCCESS;
    }

private:
    bool wait_for_odom() {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(wait_timeout_seconds_));
        while (rclcpp::ok() && odom_count_ == 0U && std::chrono::steady_clock::now() < deadline) {
            rclcpp::spin_some(shared_from_this());
            std::this_thread::sleep_for(20ms);
        }
        return odom_count_ > 0U;
    }

    /// Publishes a constant forward velocity for `seconds`, then stops.
    void drive_for(double seconds, double forward_speed) {
        geometry_msgs::msg::Twist command;
        command.linear.x = forward_speed;

        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(seconds));
        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
            cmd_vel_pub_->publish(command);
            rclcpp::spin_some(shared_from_this());
            std::this_thread::sleep_for(kCmdPublishPeriod);
        }
        // Always send an explicit stop so the robot does not coast on.
        command.linear.x = 0.0;
        cmd_vel_pub_->publish(command);
    }

    double forward_speed_ = 0.3;
    double drive_seconds_ = 3.0;
    double settle_seconds_ = 1.0;
    double wait_timeout_seconds_ = 120.0;
    double min_distance_m_ = 0.25;
    std::string odom_topic_ = "odom";
    std::string cmd_vel_topic_ = "cmd_vel";
    bool selftest_ = false;

    nav_msgs::msg::Odometry last_odom_;
    std::size_t odom_count_ = 0;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}  // namespace lotse

int main(int argc, char** argv) {
    try {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<lotse::LinkCheckNode>();
        const int status = node->run_check();
        rclcpp::shutdown();
        return status;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "link check failed with an exception: %s\n", error.what());
    } catch (...) {
        std::fprintf(stderr, "link check failed with an unknown exception\n");
    }
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return EXIT_FAILURE;
}
