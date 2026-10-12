// The Isaac Sim fleet node: it runs the core fleet manager against the ROS 2
// robots with the simulator clock, persists the order history in SQLite, and
// serves the shared service dashboard and JSON API. It writes a small result
// file for the demo script and shuts the launch down when the list is done.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lotse/core/fleet_manager.hpp"
#include "lotse/core/map.hpp"
#include "lotse/core/order.hpp"
#include "lotse/core/order_store.hpp"
#include "lotse/core/prioritized_planner.hpp"
#include "lotse/core/task_assigner.hpp"
#include "lotse/service/fleet_api.hpp"
#include "lotse/service/fleet_json.hpp"
#include "lotse/service/http_api.hpp"
#include "lotse/service/sqlite_order_store.hpp"

#include "lotse_ros/ros_clock.hpp"
#include "lotse_ros/ros_robot_side.hpp"

namespace {

using lotse::core::GridCell;

/// Returns the nearest-rank percentile of a non-empty sample.
/// @param samples the values to rank
/// @param percentile the percentile in [0, 1]
[[nodiscard]] double percentile(const std::vector<double>& samples, double percentile) {
    std::vector<double> sorted = samples;
    std::sort(sorted.begin(), sorted.end());
    const auto count = static_cast<double>(sorted.size());
    const auto rank = static_cast<std::size_t>(std::ceil(percentile * count));
    const std::size_t index = rank == 0 ? 0 : std::min(rank - 1, sorted.size() - 1);
    return sorted[index];
}

using lotse::service::CancelOutcome;
using lotse::service::OrderRejected;

/// One true robot position in metres, taken from the simulator ground truth.
struct TruePose {
    double x = 0.0;
    double y = 0.0;
};

/// Adapts the node's live fleet manager to the shared service API and saves
/// every order transition in the SQLite store.
class RosFleetSource final : public lotse::service::FleetApiSource {
public:
    /// Stores the references the JSON builders and the store need.
    /// @param map the warehouse grid
    /// @param manager the live fleet manager
    /// @param store the SQLite order store
    /// @param fleetSize the configured number of robots
    RosFleetSource(const lotse::core::Map& map, lotse::core::FleetManager& manager,
                   lotse::core::OrderStore& store, int fleetSize)
        : map_(map), manager_(manager), store_(store), fleetSize_(fleetSize) {}

    /// Returns the shared JSON map document.
    std::string mapJson() const override { return lotse::service::mapJson(map_); }

    /// Returns the shared JSON fleet document.
    std::string fleetJson() const override { return lotse::service::fleetJson(manager_); }

    /// Returns the shared JSON order list.
    std::string ordersJson() const override { return lotse::service::ordersJson(manager_); }

    /// Returns one order as the shared JSON shape.
    /// @param id the order id
    std::optional<std::string> orderJson(int id) const override {
        return lotse::service::orderJson(manager_, id);
    }

    /// Returns the shared JSON throughput document.
    std::string throughputJson() const override { return lotse::service::throughputJson(manager_); }

    /// Returns the shared JSON health document.
    std::string healthJson() const override {
        return lotse::service::healthJson(manager_, fleetSize_, "prioritized");
    }

    /// Validates a named order, stores it and returns its id.
    /// @param pickupName the named pickup station
    /// @param dropoffName the named drop-off station
    /// @param priority higher is served sooner
    int submitOrder(const std::string& pickupName, const std::string& dropoffName,
                    int priority) override {
        const std::optional<GridCell> pickup = map_.station(pickupName);
        if (!pickup.has_value()) {
            throw OrderRejected("unknown station '" + pickupName + "'");
        }
        const std::optional<GridCell> dropoff = map_.station(dropoffName);
        if (!dropoff.has_value()) {
            throw OrderRejected("unknown station '" + dropoffName + "'");
        }
        if (!pickupName.starts_with("pickup")) {
            throw OrderRejected("'" + pickupName + "' is not a pickup station");
        }
        if (!dropoffName.starts_with("dropoff")) {
            throw OrderRejected("'" + dropoffName + "' is not a drop-off station");
        }
        if (pickup.value() == dropoff.value()) {
            throw OrderRejected("pickup and drop-off are the same cell");
        }
        return submitOrderCells(pickup.value(), dropoff.value(), priority);
    }

    /// Stores an order the node generated directly from map cells.
    /// @param pickup the pickup cell
    /// @param dropoff the drop-off cell
    /// @param priority higher is served sooner
    int submitOrderCells(GridCell pickup, GridCell dropoff, int priority) {
        const int id = manager_.submitOrder(pickup, dropoff, priority);
        persistOrder(id);
        return id;
    }

    /// Cancels a stored order that has not been picked up yet.
    /// @param orderId the order to cancel
    CancelOutcome cancelOrder(int orderId) override {
        const std::optional<lotse::core::ManagedOrder> order = manager_.order(orderId);
        if (!order.has_value()) {
            return CancelOutcome::NotFound;
        }
        if (order->state == lotse::core::OrderState::InProgress) {
            return CancelOutcome::InProgress;
        }
        if (order->state != lotse::core::OrderState::Pending &&
            order->state != lotse::core::OrderState::Assigned) {
            return CancelOutcome::Finished;
        }
        if (!manager_.cancelOrder(orderId)) {
            return CancelOutcome::InProgress;
        }
        persistEvents();
        return CancelOutcome::Cancelled;
    }

    /// Keeps the dashboard push callback the server installs.
    /// @param sink the callback, empty to stop updates
    void setUpdateSink(std::function<void(std::string)> sink) override { sink_ = std::move(sink); }

    /// Saves every order transition produced since the previous call.
    void persistEvents() {
        for (const lotse::core::OrderEvent& event : manager_.drainEvents()) {
            store_.appendEvent(lotse::core::StoredEvent{
                event.orderId, lotse::core::toString(event.state), event.atMs});
            persistOrder(event.orderId);
        }
    }

private:
    /// Saves the current state of one order when it still exists.
    /// @param orderId the order to save
    void persistOrder(int orderId) {
        const std::optional<lotse::core::ManagedOrder> order = manager_.order(orderId);
        if (order.has_value()) {
            store_.saveOrder(lotse::service::storedOrder(order.value()));
        }
    }

    const lotse::core::Map& map_;
    lotse::core::FleetManager& manager_;
    lotse::core::OrderStore& store_;
    int fleetSize_ = 4;
    std::function<void(std::string)> sink_;
};

/// Runs the real-time manager against ROS 2 robots and serves the service API.
class FleetApiNode final : public rclcpp::Node {
public:
    /// Builds the map, manager, SQLite store and dashboard from the parameters.
    FleetApiNode() : rclcpp::Node("fleet_api") {
        const std::string map_path = declare_parameter<std::string>("map_path", "");
        fleet_size_ = static_cast<int>(declare_parameter<std::int64_t>("fleet_size", 4));
        cell_size_ = declare_parameter<double>("cell_size", 1.0);
        tick_hz_ = declare_parameter<double>("tick_hz", 20.0);
        order_count_ = static_cast<int>(declare_parameter<std::int64_t>("order_count", 12));
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
        const double progress_window_seconds =
            declare_parameter<double>("progress_window_seconds", 120.0);
        result_path_ = declare_parameter<std::string>("result_path", "/tmp/lotse-isaac-fleet.json");
        const std::string database_path =
            declare_parameter<std::string>("database_path", "/tmp/lotse-isaac-orders.sqlite");
        timeout_seconds_ = declare_parameter<double>("timeout_seconds", 1800.0);
        const int dashboard_port =
            static_cast<int>(declare_parameter<std::int64_t>("dashboard_port", 8080));
        const std::string bind_address =
            declare_parameter<std::string>("bind_address", "127.0.0.1");
        const std::string dashboard_dir =
            declare_parameter<std::string>("dashboard_dir", "service/dashboard");

        const std::optional<lotse::core::Map> loaded = lotse::core::Map::fromFile(map_path);
        if (!loaded.has_value()) {
            throw std::runtime_error("fleet_api: cannot read map " + map_path);
        }
        map_ = loaded.value();
        if (static_cast<int>(map_.parkingCells().size()) < fleet_size_) {
            throw std::runtime_error("fleet_api: not enough parking cells for the fleet");
        }
        std::vector<lotse::core::GridCell> starts;
        std::vector<std::string> namespaces;
        for (int robot = 0; robot < fleet_size_; ++robot) {
            starts.push_back(map_.parkingCells().at(static_cast<std::size_t>(robot)));
            namespaces.push_back("robot_" + std::to_string(robot));
        }

        side_ = std::make_unique<lotse::ros::RosRobotSide>(*this, namespaces, starts, cell_size_);
        starts_ = starts;
        true_poses_.assign(static_cast<std::size_t>(fleet_size_), TruePose{});
        for (int robot = 0; robot < fleet_size_; ++robot) {
            const std::string topic = namespaces[static_cast<std::size_t>(robot)] + "/ground_truth";
            pose_subscriptions_.push_back(create_subscription<nav_msgs::msg::Odometry>(
                topic, 10, [this, robot](const nav_msgs::msg::Odometry::SharedPtr message) {
                    if (robot < static_cast<int>(true_poses_.size())) {
                        true_poses_[static_cast<std::size_t>(robot)] = {
                            message->pose.pose.position.x, message->pose.pose.position.y};
                    }
                }));
        }
        planner_ = std::make_unique<lotse::core::PrioritizedPlanner>(
            lotse::core::PrioritizedOptions{lotse::core::AgentOrder::FarthestFirst, 0,
                                            lotse::core::continuousHorizon(map_, fleet_size_), 8,
                                            plan_time_limit_ms, true, true});
        lotse::core::FleetManagerConfig config;
        config.windowSeconds = replan_window_seconds;
        config.planTimeLimitMs = plan_time_limit_ms;
        config.priorityAgeingSeconds = priority_ageing_seconds;
        config.deadlockPatienceSeconds = deadlock_patience_seconds;
        config.progressWindowSeconds = progress_window_seconds;
        config.backgroundPlanning = false;
        config.passOrderEnabled = true;
        config.diagnostics = true;
        manager_ = std::make_unique<lotse::core::FleetManager>(map_, *side_, *planner_, assigner_,
                                                               clock_, config);

        store_ = std::make_unique<lotse::service::SqliteOrderStore>(database_path);
        store_->open();
        source_ = std::make_unique<RosFleetSource>(map_, *manager_, *store_, fleet_size_);

        const lotse::core::OrderGeneratorOptions options{order_count_, 0, arrival_interval, 2, 3};
        orders_ = lotse::core::generateOrders(map_, options, static_cast<std::uint32_t>(seed));
        start_time_ = clock_.now();
        wall_start_ = std::chrono::steady_clock::now();
        const std::size_t slash = result_path_.find_last_of('/');
        run_log_path_ =
            (slash == std::string::npos ? std::string("run_log.jsonl")
                                        : result_path_.substr(0, slash + 1) + "run_log.jsonl");
        run_log_.open(run_log_path_);
        prevPassOrder_.assign(static_cast<std::size_t>(fleet_size_), 0.0);
        prevCellOccupied_.assign(static_cast<std::size_t>(fleet_size_), 0.0);

        if (dashboard_port >= 0) {
            api_ = std::make_unique<lotse::service::ApiServer>(*source_, bind_address,
                                                               dashboard_port, dashboard_dir);
            server_thread_ = std::thread([this] {
                if (!api_->listen()) {
                    RCLCPP_ERROR(get_logger(), "dashboard failed to listen");
                }
            });
        }

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / (tick_hz_ > 1.0 ? tick_hz_ : 1.0)));
        timer_ = create_timer(period, [this] { tick(); });
    }

    FleetApiNode(const FleetApiNode&) = delete;
    FleetApiNode& operator=(const FleetApiNode&) = delete;
    FleetApiNode(FleetApiNode&&) = delete;
    FleetApiNode& operator=(FleetApiNode&&) = delete;

    /// Stops the dashboard server and joins its thread.
    ~FleetApiNode() override {
        if (api_) {
            api_->stop();
        }
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
    }

private:
    /// Releases due orders, ticks the manager and checks the stop conditions.
    void tick() {
        ++step_;
        while (next_order_ < orders_.size() && orders_[next_order_].arrivalStep <= step_) {
            const lotse::core::Order& order = orders_[next_order_];
            source_->submitOrderCells(order.pickup, order.dropoff, order.priority);
            ++next_order_;
        }
        manager_->tick(1.0 / (tick_hz_ > 1.0 ? tick_hz_ : 1.0));
        source_->persistEvents();
        for (const lotse::core::RobotStatus& status : manager_->robotStatuses()) {
            if (status.stopped) {
                ++stopped_frames_;
                break;
            }
        }
        writeRunLog();
        if (api_) {
            api_->publishFleet();
        }
        const lotse::core::ThroughputStats stats = manager_->throughput();
        if (stats.ordersCompleted >= order_count_) {
            finish();
            return;
        }
        if (simulatedSeconds() > timeout_seconds_) {
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
        writeRunLog();
        rclcpp::shutdown();
    }

    /// Returns the name of the station or parking cell at a grid cell.
    /// @param cell the cell to name
    [[nodiscard]] std::string stationName(GridCell cell) const {
        for (const std::pair<std::string, GridCell>& entry : map_.stations()) {
            if (entry.second == cell) {
                return entry.first;
            }
        }
        const std::vector<GridCell>& parking = map_.parkingCells();
        for (std::size_t index = 0; index < parking.size(); ++index) {
            if (parking[index] == cell) {
                return "parking" + std::to_string(index);
            }
        }
        return "cell";
    }

    /// Writes one run-log line for the video overlay: true positions, tasks and
    /// the reason each robot is waiting, all stamped with simulated time.
    void writeRunLog() {
        if (!run_log_.is_open()) {
            return;
        }
        const std::vector<lotse::core::RobotStatus> statuses = manager_->robotStatuses();
        const std::vector<int> orderIds = manager_->robotOrderIds();
        const lotse::core::ManagerDiagnostics diag = manager_->diagnostics();
        const lotse::core::ThroughputStats stats = manager_->throughput();
        const double absoluteSeconds =
            std::chrono::duration<double>(clock_.now().time_since_epoch()).count();
        std::ostringstream line;
        line << R"({"t":)" << std::fixed << std::setprecision(3) << simulatedSeconds()
             << R"(,"t_abs":)" << absoluteSeconds << R"(,"done":)" << stats.ordersCompleted
             << R"(,"released":)" << next_order_ << R"(,"robots":[)";
        for (int robot = 0; robot < fleet_size_; ++robot) {
            const auto slot = static_cast<std::size_t>(robot);
            if (robot > 0) {
                line << ',';
            }
            std::string phase = "parked";
            std::string station = stationName(starts_[slot]);
            int orderId = slot < orderIds.size() ? orderIds[slot] : -1;
            if (orderId >= 0) {
                const std::optional<lotse::core::ManagedOrder> managed = manager_->order(orderId);
                if (managed.has_value()) {
                    const bool carrying = managed->state == lotse::core::OrderState::InProgress;
                    phase = carrying ? "dropoff" : "pickup";
                    station =
                        stationName(carrying ? managed->order.dropoff : managed->order.pickup);
                }
            } else if (slot < statuses.size() && statuses[slot].cell != starts_[slot]) {
                phase = "parking";
            }
            std::string wait = "none";
            if (slot < diag.passOrderWaitSeconds.size() &&
                diag.passOrderWaitSeconds[slot] > prevPassOrder_[slot]) {
                wait = "pass_order";
            } else if (slot < diag.cellOccupiedWaitSeconds.size() &&
                       diag.cellOccupiedWaitSeconds[slot] > prevCellOccupied_[slot]) {
                wait = "cell_occupied";
            }
            const double x = slot < true_poses_.size() ? true_poses_[slot].x : 0.0;
            const double y = slot < true_poses_.size() ? true_poses_[slot].y : 0.0;
            line << R"({"id":)" << robot << R"(,"x":)" << x << R"(,"y":)" << y << R"(,"order":)"
                 << orderId << R"(,"phase":")" << phase << R"(,"station":")" << station
                 << R"(,"wait":")" << wait << R"("})";
        }
        line << "]}\n";
        run_log_ << line.str();
        run_log_.flush();
        for (std::size_t index = 0; index < prevPassOrder_.size(); ++index) {
            if (index < diag.passOrderWaitSeconds.size()) {
                prevPassOrder_[index] = diag.passOrderWaitSeconds[index];
                prevCellOccupied_[index] = diag.cellOccupiedWaitSeconds[index];
            }
        }
    }

    /// Writes the counters as one JSON object.
    void writeResult() {
        const double simSeconds = simulatedSeconds();
        const double wallSeconds =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start_).count();
        const lotse::core::ThroughputStats stats = manager_->throughput();
        const lotse::judge::Report report = manager_->safetyReport();
        const lotse::core::ManagerDiagnostics diagnostics = manager_->diagnostics();
        const double planningP95Ms =
            diagnostics.planningCallMs.empty() ? 0.0 : percentile(diagnostics.planningCallMs, 0.95);
        const double planningMaxMs =
            diagnostics.planningCallMs.empty() ? 0.0 : percentile(diagnostics.planningCallMs, 1.0);
        int robotsUsed = 0;
        std::vector<bool> robotUsed(static_cast<std::size_t>(fleet_size_), false);
        for (const lotse::core::ManagedOrder& order : manager_->orders()) {
            if (order.state == lotse::core::OrderState::Completed && order.robot >= 0) {
                robotUsed[static_cast<std::size_t>(order.robot)] = true;
            }
        }
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
             << ",\"planning_p95_ms\":" << planningP95Ms << ",\"planning_max_ms\":" << planningMaxMs
             << ",\"no_progress\":" << (stats.noProgress ? "true" : "false")
             << ",\"progress_alarm\":" << (stats.progressAlarm ? "true" : "false")
             << ",\"seconds_without_progress\":" << stats.secondsWithoutProgress
             << ",\"plan_adoptions\":" << stats.planAdoptions
             << ",\"plan_rejections\":" << stats.planRejections << ",\"robots_used\":" << robotsUsed
             << ",\"orders_pending\":" << ordersPending << ",\"orders_assigned\":" << ordersAssigned
             << ",\"orders_in_progress\":" << ordersInProgress << "}";
    }

    int fleet_size_ = 4;
    double cell_size_ = 1.0;
    int order_count_ = 12;
    double tick_hz_ = 20.0;
    std::string result_path_;
    double timeout_seconds_ = 1800.0;

    lotse::core::Map map_;
    std::unique_ptr<lotse::ros::RosRobotSide> side_;
    lotse::core::NearestRobotAssigner assigner_;
    std::unique_ptr<lotse::core::PrioritizedPlanner> planner_;
    lotse::ros::RosClock clock_{get_clock()};
    std::unique_ptr<lotse::core::FleetManager> manager_;
    std::unique_ptr<lotse::service::SqliteOrderStore> store_;
    std::unique_ptr<RosFleetSource> source_;
    std::unique_ptr<lotse::service::ApiServer> api_;
    std::thread server_thread_;
    std::vector<lotse::core::Order> orders_;
    std::size_t next_order_ = 0;
    int step_ = 0;
    long long stopped_frames_ = 0;
    std::string run_log_path_;
    std::ofstream run_log_;
    std::vector<lotse::core::GridCell> starts_;
    std::vector<TruePose> true_poses_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> pose_subscriptions_;
    std::vector<double> prevPassOrder_;
    std::vector<double> prevCellOccupied_;
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point wall_start_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

/// Starts the node and spins it until the order list is done.
/// @param argc the argument count
/// @param argv the argument values
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FleetApiNode>());
    rclcpp::shutdown();
    return 0;
}
