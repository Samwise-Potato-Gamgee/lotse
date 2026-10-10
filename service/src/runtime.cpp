// Implements the service runtime declared in runtime.hpp.

#include "lotse/service/runtime.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "lotse/core/cbs.hpp"
#include "lotse/core/fleet_manager.hpp"
#include "lotse/core/order_store.hpp"
#include "lotse/core/prioritized_planner.hpp"
#include "lotse/core/robot_side.hpp"
#include "lotse/core/task_assigner.hpp"
#include "lotse/sim/grid_simulator.hpp"

#include <nlohmann/json.hpp>

namespace lotse::service {
namespace {

using nlohmann::json;

/// Loads the map, throwing when the file is missing.
/// @param path the map file to read
[[nodiscard]] core::Map loadMapOrThrow(const std::string& path) {
    const std::optional<core::Map> map = core::Map::fromFile(path);
    if (!map.has_value()) {
        throw std::runtime_error("service: cannot read map " + path);
    }
    return map.value();
}

/// Picks the parking cells as start cells for the fleet.
/// @param map the warehouse grid
/// @param fleetSize how many robots to place
[[nodiscard]] std::vector<core::GridCell> makeStarts(const core::Map& map, int fleetSize) {
    if (fleetSize <= 0) {
        throw std::runtime_error("service: fleet_size must be positive");
    }
    if (static_cast<int>(map.parkingCells().size()) < fleetSize) {
        throw std::runtime_error("service: map has fewer parking cells than the fleet size");
    }
    return {map.parkingCells().begin(), map.parkingCells().begin() + fleetSize};
}

/// Builds the simulator tuning from the service settings.
/// @param config the service settings
[[nodiscard]] sim::GridRobotSideConfig simulatorConfig(const ServiceConfig& config) {
    sim::GridRobotSideConfig sim_config;
    sim_config.cellSeconds = config.cellSeconds;
    sim_config.seed = config.simSeed;
    sim_config.slowdownProbability = config.slowdownProbability;
    sim_config.stopProbability = config.stopProbability;
    return sim_config;
}

/// Formats one order as JSON.
/// @param order the order to format
[[nodiscard]] json orderToJson(const core::ManagedOrder& order) {
    json document;
    document["id"] = order.order.id;
    document["pickup"] = {{"x", order.order.pickup.x}, {"y", order.order.pickup.y}};
    document["dropoff"] = {{"x", order.order.dropoff.x}, {"y", order.order.dropoff.y}};
    document["priority"] = order.order.priority;
    document["state"] = core::toString(order.state);
    document["robot"] = order.robot;
    document["created_ms"] = order.createdMs;
    document["updated_ms"] = order.updatedMs;
    return document;
}

/// Converts a managed order into a stored order.
/// @param order the order to convert
[[nodiscard]] core::StoredOrder toStored(const core::ManagedOrder& order) {
    core::StoredOrder stored;
    stored.id = order.order.id;
    stored.pickup = order.order.pickup;
    stored.dropoff = order.order.dropoff;
    stored.priority = order.order.priority;
    stored.state = core::toString(order.state);
    stored.createdMs = order.createdMs;
    stored.updatedMs = order.updatedMs;
    return stored;
}

}  // namespace

OrderRejected::OrderRejected(const std::string& message) : std::runtime_error(message) {}

struct Runtime::Impl {
    ServiceConfig config;
    core::Map map;
    core::Clock& clock;
    core::OrderStore& store;
    sim::GridRobotSide side;
    core::NearestRobotAssigner assigner;
    std::unique_ptr<core::Planner> planner;
    std::unique_ptr<core::Planner> fallback;
    std::unique_ptr<core::FleetManager> manager;

    std::mutex sinkMutex;
    std::function<void(std::string)> sink;

    std::thread ticker;
    std::atomic<bool> stop{false};
    std::mutex wakeMutex;
    std::condition_variable wake;

    Impl(ServiceConfig serviceConfig, core::OrderStore& orderStore, core::Clock& timeSource)
        : config(std::move(serviceConfig)),
          map(loadMapOrThrow(config.mapPath)),
          clock(timeSource),
          store(orderStore),
          side(makeStarts(map, config.fleetSize), simulatorConfig(config)) {
        const int horizon = core::continuousHorizon(map, config.fleetSize);
        if (config.planner == "cbs") {
            planner = std::make_unique<core::CbsPlanner>(
                core::CbsOptions{config.planTimeLimitMs, 0, horizon});
            fallback = std::make_unique<core::PrioritizedPlanner>(core::PrioritizedOptions{
                core::AgentOrder::FarthestFirst, 0, horizon, 8, config.planTimeLimitMs});
        } else if (config.planner == "prioritized") {
            planner = std::make_unique<core::PrioritizedPlanner>(core::PrioritizedOptions{
                core::AgentOrder::FarthestFirst, 0, horizon, 8, config.planTimeLimitMs});
        } else {
            throw std::runtime_error("service: unknown planner " + config.planner);
        }
        core::FleetManagerConfig managerConfig;
        managerConfig.windowSteps = config.replanWindowSteps;
        managerConfig.planTimeLimitMs = config.planTimeLimitMs;
        managerConfig.priorityAgeingInterval = config.priorityAgeingInterval;
        managerConfig.deadlockPatience = config.deadlockPatience;
        managerConfig.backgroundPlanning = config.backgroundPlanning;
        managerConfig.passOrderEnabled = config.passOrder;
        manager = std::make_unique<core::FleetManager>(map, side, *planner, assigner, clock,
                                                       managerConfig, fallback.get());
        manager->restoreOrders(store.loadOrders());
    }

    /// Stores one order and its latest state.
    /// @param orderId the order to store
    void persistOrder(int orderId) {
        const std::optional<core::ManagedOrder> order = manager->order(orderId);
        if (order.has_value()) {
            store.saveOrder(toStored(order.value()));
        }
    }

    /// Runs the ticker loop until stop() is called.
    void run() {
        if (config.tickRateHz <= 0.0) {
            return;
        }
        const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / config.tickRateHz));
        auto next = std::chrono::steady_clock::now();
        while (!stop.load()) {
            tick(config.simSecondsPerTick);
            next += period;
            std::unique_lock<std::mutex> lock(wakeMutex);
            wake.wait_until(lock, next, [this] { return stop.load(); });
        }
    }

    /// Runs one step and publishes the results.
    /// @param seconds how much simulated time the step covers
    void tick(double seconds) {
        manager->tick(seconds);
        const std::vector<core::OrderEvent> events = manager->drainEvents();
        for (const core::OrderEvent& event : events) {
            store.appendEvent(
                core::StoredEvent{event.orderId, core::toString(event.state), event.atMs});
            persistOrder(event.orderId);
        }
        std::function<void(std::string)> notify;
        {
            const std::lock_guard<std::mutex> lock(sinkMutex);
            notify = sink;
        }
        if (notify) {
            notify(fleetJson());
            for (const core::OrderEvent& event : events) {
                json message;
                message["type"] = "order";
                message["order_id"] = event.orderId;
                message["state"] = core::toString(event.state);
                message["at_ms"] = event.atMs;
                notify(message.dump());
            }
        }
    }

    /// Builds the live fleet document.
    [[nodiscard]] std::string fleetJson() const {
        const std::vector<core::RobotStatus> statuses = manager->robotStatuses();
        const std::vector<int> orderIds = manager->robotOrderIds();
        const core::ThroughputStats stats = manager->throughput();
        const judge::Report report = manager->safetyReport();
        json robots = json::array();
        for (std::size_t index = 0; index < statuses.size(); ++index) {
            const core::RobotStatus& status = statuses[index];
            json robot;
            robot["id"] = status.id;
            robot["x"] = status.cell.x;
            robot["y"] = status.cell.y;
            robot["target_x"] = status.target.x;
            robot["target_y"] = status.target.y;
            robot["progress"] = status.progress;
            robot["stopped"] = status.stopped;
            robot["order"] = orderIds[index];
            robot["busy"] = orderIds[index] >= 0;
            robots.push_back(std::move(robot));
        }
        json document;
        document["type"] = "fleet";
        document["step"] = stats.steps;
        document["deadlocked"] = manager->deadlocked();
        document["collisions"] = report.collisions;
        document["swaps"] = report.swaps;
        document["orders_completed"] = stats.ordersCompleted;
        document["orders_open"] = stats.ordersOpen;
        document["robots"] = std::move(robots);
        return document.dump();
    }
};

Runtime::Runtime(ServiceConfig config, core::OrderStore& store, core::Clock& clock)
    : impl_(std::make_unique<Impl>(std::move(config), store, clock)) {}

Runtime::~Runtime() {
    stop();
}

void Runtime::start() {
    if (impl_->ticker.joinable()) {
        return;
    }
    impl_->stop = false;
    impl_->ticker = std::thread([this] { impl_->run(); });
}

void Runtime::stop() {
    impl_->stop = true;
    impl_->wake.notify_all();
    if (impl_->ticker.joinable()) {
        impl_->ticker.join();
    }
}

void Runtime::tickOnce(double seconds) {
    impl_->tick(seconds);
}

int Runtime::submitOrder(const std::string& pickupName, const std::string& dropoffName,
                         int priority) {
    const std::optional<core::GridCell> pickup = impl_->map.station(pickupName);
    if (!pickup.has_value()) {
        throw OrderRejected("unknown station '" + pickupName + "'");
    }
    const std::optional<core::GridCell> dropoff = impl_->map.station(dropoffName);
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
    const int id = impl_->manager->submitOrder(pickup.value(), dropoff.value(), priority);
    const std::optional<core::ManagedOrder> order = impl_->manager->order(id);
    if (order.has_value()) {
        impl_->store.saveOrder(toStored(order.value()));
        impl_->store.appendEvent(
            core::StoredEvent{id, core::toString(order->state), order->updatedMs});
    }
    return id;
}

CancelOutcome Runtime::cancelOrder(int orderId) {
    const std::optional<core::ManagedOrder> order = impl_->manager->order(orderId);
    if (!order.has_value()) {
        return CancelOutcome::NotFound;
    }
    if (order->state == core::OrderState::InProgress) {
        return CancelOutcome::InProgress;
    }
    if (order->state != core::OrderState::Pending && order->state != core::OrderState::Assigned) {
        return CancelOutcome::Finished;
    }
    if (!impl_->manager->cancelOrder(orderId)) {
        return CancelOutcome::InProgress;
    }
    const std::vector<core::OrderEvent> events = impl_->manager->drainEvents();
    for (const core::OrderEvent& event : events) {
        impl_->store.appendEvent(
            core::StoredEvent{event.orderId, core::toString(event.state), event.atMs});
        impl_->persistOrder(event.orderId);
    }
    return CancelOutcome::Cancelled;
}

void Runtime::setUpdateSink(std::function<void(std::string)> sink) {
    const std::lock_guard<std::mutex> lock(impl_->sinkMutex);
    impl_->sink = std::move(sink);
}

const ServiceConfig& Runtime::config() const noexcept {
    return impl_->config;
}

std::string Runtime::mapJson() const {
    json rows = json::array();
    for (int y = 0; y < impl_->map.height(); ++y) {
        std::string row;
        row.reserve(static_cast<std::size_t>(impl_->map.width()));
        for (int x = 0; x < impl_->map.width(); ++x) {
            row.push_back(impl_->map.isFree(core::GridCell{x, y}) ? '.' : '#');
        }
        rows.push_back(std::move(row));
    }
    json stations = json::array();
    for (const auto& station : impl_->map.stations()) {
        stations.push_back(
            {{"name", station.first}, {"x", station.second.x}, {"y", station.second.y}});
    }
    json parking = json::array();
    for (const core::GridCell cell : impl_->map.parkingCells()) {
        parking.push_back({{"x", cell.x}, {"y", cell.y}});
    }
    json document;
    document["name"] = impl_->map.name();
    document["width"] = impl_->map.width();
    document["height"] = impl_->map.height();
    document["rows"] = std::move(rows);
    document["stations"] = std::move(stations);
    document["parking"] = std::move(parking);
    return document.dump();
}

std::string Runtime::fleetJson() const {
    return impl_->fleetJson();
}

std::string Runtime::ordersJson() const {
    json list = json::array();
    for (const core::ManagedOrder& order : impl_->manager->orders()) {
        list.push_back(orderToJson(order));
    }
    return list.dump();
}

std::optional<std::string> Runtime::orderJson(int id) const {
    const std::optional<core::ManagedOrder> order = impl_->manager->order(id);
    if (!order.has_value()) {
        return std::nullopt;
    }
    return orderToJson(order.value()).dump();
}

std::string Runtime::throughputJson() const {
    const core::ThroughputStats stats = impl_->manager->throughput();
    json document;
    document["orders_completed"] = stats.ordersCompleted;
    document["orders_open"] = stats.ordersOpen;
    document["orders_cancelled"] = stats.ordersCancelled;
    document["orders_failed"] = stats.ordersFailed;
    document["steps"] = stats.steps;
    document["elapsed_seconds"] = stats.elapsedSeconds;
    document["orders_per_hour"] = stats.ordersPerHour;
    document["planning_calls"] = stats.planningCalls;
    document["planning_failures"] = stats.planningFailures;
    document["plan_adoptions"] = stats.planAdoptions;
    document["plan_rejections"] = stats.planRejections;
    return document.dump();
}

std::string Runtime::healthJson() const {
    const core::ThroughputStats stats = impl_->manager->throughput();
    const judge::Report report = impl_->manager->safetyReport();
    json document;
    document["status"] = "ok";
    document["robots"] = impl_->config.fleetSize;
    document["orders_open"] = stats.ordersOpen;
    document["deadlocked"] = impl_->manager->deadlocked();
    document["collisions"] = report.collisions;
    document["swaps"] = report.swaps;
    document["planner"] = impl_->config.planner;
    return document.dump();
}

}  // namespace lotse::service
