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

#include "lotse_ros/ros_robot_side.hpp"

namespace {

/// Runs the real-time manager against the ROS 2 robots.
class FleetNode final : public rclcpp::Node {
public:
    FleetNode() : rclcpp::Node("fleet_node") {
        const std::string map_path = declare_parameter<std::string>("map_path", "");
        fleet_size_ = static_cast<int>(declare_parameter<std::int64_t>("fleet_size", 4));
        cell_size_ = declare_parameter<double>("cell_size", 1.0);
        const double tick_hz = declare_parameter<double>("tick_hz", 20.0);
        order_count_ = static_cast<int>(declare_parameter<std::int64_t>("order_count", 20));
        const int arrival_interval =
            static_cast<int>(declare_parameter<std::int64_t>("arrival_interval", 10));
        const int seed = static_cast<int>(declare_parameter<std::int64_t>("seed", 1));
        const int plan_time_limit_ms =
            static_cast<int>(declare_parameter<std::int64_t>("plan_time_limit_ms", 500));
        const int window_steps =
            static_cast<int>(declare_parameter<std::int64_t>("window_steps", 40));
        const int deadlock_patience =
            static_cast<int>(declare_parameter<std::int64_t>("deadlock_patience", 3000));
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
                lotse::core::AgentOrder::FarthestFirst, 0, 0, 8, plan_time_limit_ms});
        lotse::core::FleetManagerConfig config;
        config.windowSteps = window_steps;
        config.planTimeLimitMs = plan_time_limit_ms;
        config.priorityAgeingInterval = 25;
        config.deadlockPatience = deadlock_patience;
        config.backgroundPlanning = true;
        config.passOrderEnabled = true;
        manager_ = std::make_unique<lotse::core::FleetManager>(map_, *side_, *planner_, assigner_,
                                                               clock_, config);

        const lotse::core::OrderGeneratorOptions options{order_count_, 0, arrival_interval, 2, 3};
        orders_ = lotse::core::generateOrders(map_, options, static_cast<std::uint32_t>(seed));
        start_time_ = std::chrono::steady_clock::now();

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / (tick_hz > 1.0 ? tick_hz : 1.0)));
        timer_ = create_wall_timer(period, [this] { tick(); });
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
        manager_->tick(1.0);
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
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
        if (elapsed > timeout_seconds_) {
            finish();
            return;
        }
        if (step_ % 40 == 0) {
            writeResult(elapsed);
        }
    }

    /// Writes the result file and stops the launch.
    void finish() {
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
        writeResult(elapsed);
        rclcpp::shutdown();
    }

    /// Writes the counters as one JSON object.
    /// @param elapsed wall seconds since startup
    void writeResult(double elapsed) {
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
             << ",\"wall_seconds\":" << elapsed << ",\"planning_calls\":" << stats.planningCalls
             << ",\"planning_failures\":" << stats.planningFailures
             << ",\"plan_adoptions\":" << stats.planAdoptions
             << ",\"plan_rejections\":" << stats.planRejections << ",\"robots_used\":" << robotsUsed
             << "}";
    }

    int fleet_size_ = 4;
    double cell_size_ = 1.0;
    int order_count_ = 20;
    std::string result_path_;
    double timeout_seconds_ = 240.0;

    lotse::core::Map map_;
    std::unique_ptr<lotse::ros::RosRobotSide> side_;
    lotse::core::NearestRobotAssigner assigner_;
    std::unique_ptr<lotse::core::PrioritizedPlanner> planner_;
    lotse::core::SteadyClock clock_;
    std::unique_ptr<lotse::core::FleetManager> manager_;
    std::vector<lotse::core::Order> orders_;
    std::size_t next_order_ = 0;
    int step_ = 0;
    long long stopped_frames_ = 0;
    std::chrono::steady_clock::time_point start_time_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FleetNode>());
    rclcpp::shutdown();
    return 0;
}
