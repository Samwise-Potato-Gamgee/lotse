// Implements the shared JSON document builders and the stored-order conversion
// declared in fleet_json.hpp.

#include "lotse/service/fleet_json.hpp"

#include <cstddef>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace lotse::service {
namespace {

using nlohmann::json;

/// Formats one order as JSON.
/// @param order the order to format
[[nodiscard]] json oneOrder(const core::ManagedOrder& order) {
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

}  // namespace

std::string mapJson(const core::Map& map) {
    json rows = json::array();
    for (int y = 0; y < map.height(); ++y) {
        std::string row;
        row.reserve(static_cast<std::size_t>(map.width()));
        for (int x = 0; x < map.width(); ++x) {
            row.push_back(map.isFree(core::GridCell{x, y}) ? '.' : '#');
        }
        rows.push_back(std::move(row));
    }
    json stations = json::array();
    for (const auto& station : map.stations()) {
        stations.push_back(
            {{"name", station.first}, {"x", station.second.x}, {"y", station.second.y}});
    }
    json parking = json::array();
    for (const core::GridCell cell : map.parkingCells()) {
        parking.push_back({{"x", cell.x}, {"y", cell.y}});
    }
    json document;
    document["name"] = map.name();
    document["width"] = map.width();
    document["height"] = map.height();
    document["rows"] = std::move(rows);
    document["stations"] = std::move(stations);
    document["parking"] = std::move(parking);
    return document.dump();
}

std::string fleetJson(const core::FleetManager& manager) {
    const std::vector<core::RobotStatus> statuses = manager.robotStatuses();
    const std::vector<int> orderIds = manager.robotOrderIds();
    const core::ThroughputStats stats = manager.throughput();
    const judge::Report report = manager.safetyReport();
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
    document["deadlocked"] = manager.deadlocked();
    document["collisions"] = report.collisions;
    document["swaps"] = report.swaps;
    document["orders_completed"] = stats.ordersCompleted;
    document["orders_open"] = stats.ordersOpen;
    document["robots"] = std::move(robots);
    return document.dump();
}

std::string ordersJson(const core::FleetManager& manager) {
    json list = json::array();
    for (const core::ManagedOrder& order : manager.orders()) {
        list.push_back(oneOrder(order));
    }
    return list.dump();
}

std::optional<std::string> orderJson(const core::FleetManager& manager, int id) {
    const std::optional<core::ManagedOrder> order = manager.order(id);
    if (!order.has_value()) {
        return std::nullopt;
    }
    return oneOrder(order.value()).dump();
}

std::string throughputJson(const core::FleetManager& manager) {
    const core::ThroughputStats stats = manager.throughput();
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
    document["no_progress"] = stats.noProgress;
    document["progress_alarm"] = stats.progressAlarm;
    document["seconds_without_progress"] = stats.secondsWithoutProgress;
    return document.dump();
}

std::string healthJson(const core::FleetManager& manager, int robots, const std::string& planner) {
    const core::ThroughputStats stats = manager.throughput();
    const judge::Report report = manager.safetyReport();
    json document;
    document["status"] = "ok";
    document["robots"] = robots;
    document["orders_open"] = stats.ordersOpen;
    document["deadlocked"] = manager.deadlocked();
    document["collisions"] = report.collisions;
    document["swaps"] = report.swaps;
    document["planner"] = planner;
    document["no_progress"] = stats.noProgress;
    document["progress_alarm"] = stats.progressAlarm;
    document["seconds_without_progress"] = stats.secondsWithoutProgress;
    return document.dump();
}

core::StoredOrder storedOrder(const core::ManagedOrder& order) {
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

}  // namespace lotse::service
