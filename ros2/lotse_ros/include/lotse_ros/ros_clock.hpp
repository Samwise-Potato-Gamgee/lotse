// A core::Clock backed by a ROS 2 node clock. With use_sim_time set the node
// clock follows the simulator's /clock, so the fleet manager measures elapsed
// time, timeouts and orders per hour in simulated seconds; without it the same
// clock reads the system clock. The core stays free of any ROS dependency.

#pragma once

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "lotse/core/clock.hpp"

namespace lotse::ros {

/// Adapts an rclcpp clock to the core's injectable clock interface.
class RosClock final : public core::Clock {
public:
    /// Wraps a ROS 2 clock.
    /// @param clock the node clock to read, following /clock under use_sim_time
    explicit RosClock(rclcpp::Clock::SharedPtr clock) : clock_(std::move(clock)) {}

    /// Returns the ROS clock reading as a steady-clock time point.
    [[nodiscard]] std::chrono::steady_clock::time_point now() const noexcept override {
        return std::chrono::steady_clock::time_point(
            std::chrono::nanoseconds(clock_->now().nanoseconds()));
    }

private:
    rclcpp::Clock::SharedPtr clock_;
};

}  // namespace lotse::ros
