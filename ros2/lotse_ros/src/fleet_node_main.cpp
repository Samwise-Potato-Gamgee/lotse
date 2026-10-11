// The fleet node: it wires the core fleet manager to the ROS 2 robot side with
// the stand-in simulator running in other nodes. It releases an order list, ticks
// the manager, and writes a small result file so the integration script can read
// the counters. It shuts the whole launch down when the orders are done.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "lotse/core/clock.hpp"
#include "lotse/core/fleet_manager.hpp"
#include "lotse/core/map.hpp"
#include "lotse/core/order.hpp"
#include "lotse/core/prioritized_planner.hpp"
#include "lotse/core/task_assigner.hpp"

#include "lotse_ros/ros_clock.hpp"
#include "lotse_ros/ros_robot_side.hpp"

namespace {

/// Runs the real-time manager against the ROS 2 robots.
class FleetNode final : public rclcpp::Node {
public:
    FleetNode() : rclcpp::Node("fleet_node") {
        const std::string map_path = declare_parameter<std::string>("map_path", "");
        fleet_size_ = static_cast<int>(declare_parameter<std::int64_t>("fleet_size", 4));
        cell_size_ = declare_parameter<double>("cell_size", 1.0);
        tick_hz_ = declare_parameter<double>("tick_hz", 20.0);
        order_count_ = static_cast<int>(declare_parameter<std::int64_t>("order_count", 20));
        const int arrival_interval =
            static_cast<int>(declare_parameter<std::int64_t>("arrival_interval", 10));
        const int seed = static_cast<int>(declare_parameter<std::int64_t>("seed", 1));
        const int plan_time_limit_ms =
            static_cast<int>(declare_parameter<std::int64_t>("plan_time_limit_ms", 500));
        const double replan_window_seconds =
            declare_parameter<double>("replan_window_seconds", 6.0);
        const double priority_ageing_seconds =
            declare_parameter<double>("priority_ageing_seconds", 3.75);
        const double deadlock_patience_seconds =
            declare_parameter<double>("deadlock_patience_seconds", 450.0);
        result_path_ = declare_parameter<std::string>("result_path", "/tmp/lotse-fleet.json");
        timeout_seconds_ = declare_parameter<double>("timeout_seconds", 240.0);

        const std::optional<lotse::core::Map> loaded = lotse::core::Map::fromFile(map_path);
        if (!loaded.has_value()) {
            throw std::runtime_error("fleet_node: cannot read map " + map_path);
        }
        map_ = loaded.value();
        if (static_cast<int>(map_.parkingCells().size()) < fleet_size_) {
            throw std::runtime_error("fleet_node: not enough parking cells for the fleet");
        }
        std::vector<lotse::core::GridCell> starts;
        std::vector<std::string> namespaces;
        for (int robot = 0; robot < fleet_size_; ++robot) {
            starts.push_back(map_.parkingCells().at(static_cast<std::size_t>(robot)));
            namespaces.push_back("robot_" + std::to_string(robot));
        }

        side_ = std::make_unique<lotse::ros::RosRobotSide>(*this, namespaces, starts, cell_size_);
        planner_ =
            std::make_unique<lotse::core::PrioritizedPlanner>(lotse::core::PrioritizedOptions{
                lotse::core::AgentOrder::FarthestFirst, 0,
                lotse::core::continuousHorizon(map_, fleet_size_), 8, plan_time_limit_ms, true});
        lotse::core::FleetManagerConfig config;
        config.windowSeconds = replan_window_seconds;
        config.planTimeLimitMs = plan_time_limit_ms;
        config.priorityAgeingSeconds = priority_ageing_seconds;
        config.deadlockPatienceSeconds = deadlock_patience_seconds;
        config.backgroundPlanning = true;
        config.passOrderEnabled = true;
        manager_ = std::make_unique<lotse::core::FleetManager>(map_, *side_, *planner_, assigner_,
                                                               clock_, config);

        const lotse::core::OrderGeneratorOptions options{order_count_, 0, arrival_interval, 2, 3};
        orders_ = lotse::core::generateOrders(map_, options, static_cast<std::uint32_t>(seed));
        start_time_ = clock_.now();
        wall_start_ = std::chrono::steady_clock::now();

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / (tick_hz_ > 1.0 ? tick_hz_ : 1.0)));
        timer_ = create_timer(period, [this] { tick(); });
    }

private:
    /// Releases due orders, ticks the manager and checks the stop conditions.
    void tick() {
        ++step_;
        while (next_order_ < orders_.size() && orders_[next_order_].arrivalStep <= step_) {
            const lotse::core::Order& order = orders_[next_order_];
            manager_->submitOrder(order.pickup, order.dropoff, order.priority);
            ++next_order_;
        }
        manager_->tick(1.0 / (tick_hz_ > 1.0 ? tick_hz_ : 1.0));
        for (const lotse::core::RobotStatus& status : manager_->robotStatuses()) {
            if (status.stopped) {
                ++stopped_frames_;
                break;
            }
        }
        const lotse::core::ThroughputStats stats = manager_->throughput();
        if (stats.ordersCompleted >= order_count_) {
            finish();
            return;
        }
        const double elapsed = simulatedSeconds();
        if (elapsed > timeout_seconds_) {
            finish();
            return;
        }
        if (step_ % 40 == 0) {
            writeResult();
        }
    }

    /// Returns the simulated seconds since the first order was submitted.
    [[nodiscard]] double simulatedSeconds() const {
        return std::chrono::duration<double>(clock_.now() - start_time_).count();
    }

    /// Writes the result file and stops the launch.
    void finish() {
        writeResult();
        rclcpp::shutdown();
    }

    /// Writes the counters as one JSON object.
    void writeResult() {
        const double simSeconds = simulatedSeconds();
        const double wallSeconds =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start_).count();
        const lotse::core::ThroughputStats stats = manager_->throughput();
        const lotse::judge::Report report = manager_->safetyReport();
        std::vector<bool> robotUsed(static_cast<std::size_t>(fleet_size_), false);
        for (const lotse::core::ManagedOrder& order : manager_->orders()) {
            if (order.state == lotse::core::OrderState::Completed && order.robot >= 0) {
                robotUsed[static_cast<std::size_t>(order.robot)] = true;
            }
        }
        int robotsUsed = 0;
        for (const bool used : robotUsed) {
            if (used) {
                ++robotsUsed;
            }
        }
        int ordersPending = 0;
        int ordersAssigned = 0;
        int ordersInProgress = 0;
        for (const lotse::core::ManagedOrder& order : manager_->orders()) {
            if (order.state == lotse::core::OrderState::Pending) {
                ++ordersPending;
            } else if (order.state == lotse::core::OrderState::Assigned) {
                ++ordersAssigned;
            } else if (order.state == lotse::core::OrderState::InProgress) {
                ++ordersInProgress;
            }
        }
        std::ofstream file(result_path_);
        if (!file) {
            return;
        }
        file << "{\"orders_released\":" << next_order_
             << ",\"orders_completed\":" << stats.ordersCompleted
             << ",\"orders_failed\":" << stats.ordersFailed
             << ",\"deadlocked\":" << (manager_->deadlocked() ? "true" : "false")
             << ",\"collisions\":" << report.collisions << ",\"swaps\":" << report.swaps
             << ",\"steps\":" << stats.steps << ",\"stopped_frames\":" << stopped_frames_
             << ",\"sim_seconds\":" << simSeconds << ",\"wall_seconds\":" << wallSeconds
             << ",\"planning_calls\":" << stats.planningCalls
             << ",\"planning_failures\":" << stats.planningFailures
             << ",\"plan_adoptions\":" << stats.planAdoptions
             << ",\"plan_rejections\":" << stats.planRejections << ",\"robots_used\":" << robotsUsed
             << ",\"orders_pending\":" << ordersPending << ",\"orders_assigned\":" << ordersAssigned
             << ",\"orders_in_progress\":" << ordersInProgress << "}";
    }

    double tick_hz_ = 20.0;
    int fleet_size_ = 4;
    double cell_size_ = 1.0;
    int order_count_ = 20;
    std::string result_path_;
    double timeout_seconds_ = 240.0;

    lotse::core::Map map_;
    std::unique_ptr<lotse::ros::RosRobotSide> side_;
    lotse::core::NearestRobotAssigner assigner_;
    std::unique_ptr<lotse::core::PrioritizedPlanner> planner_;
    lotse::ros::RosClock clock_{get_clock()};
    std::unique_ptr<lotse::core::FleetManager> manager_;
    std::vector<lotse::core::Order> orders_;
    std::size_t next_order_ = 0;
    int step_ = 0;
    long long stopped_frames_ = 0;
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point wall_start_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FleetNode>());
    rclcpp::shutdown();
    return 0;
}
