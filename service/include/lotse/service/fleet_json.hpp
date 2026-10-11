// Builds the JSON documents the dashboard and the HTTP API send, and converts a
// live order into the row the SQLite store saves. Shared by the grid service
// runtime and the ROS 2 fleet node so both serve exactly the same shapes.

#pragma once

#include <optional>
#include <string>

#include "lotse/core/fleet_manager.hpp"
#include "lotse/core/map.hpp"
#include "lotse/core/order_store.hpp"

namespace lotse::service {

/// Returns the JSON map document.
/// @param map the warehouse grid
[[nodiscard]] std::string mapJson(const core::Map& map);

/// Returns the JSON fleet document.
/// @param manager the live fleet manager
[[nodiscard]] std::string fleetJson(const core::FleetManager& manager);

/// Returns the JSON order list.
/// @param manager the live fleet manager
[[nodiscard]] std::string ordersJson(const core::FleetManager& manager);

/// Returns one order as JSON.
/// @param manager the live fleet manager
/// @param id the order id
[[nodiscard]] std::optional<std::string> orderJson(const core::FleetManager& manager, int id);

/// Returns the JSON throughput document.
/// @param manager the live fleet manager
[[nodiscard]] std::string throughputJson(const core::FleetManager& manager);

/// Returns the JSON health document.
/// @param manager the live fleet manager
/// @param robots the configured fleet size
/// @param planner the planner name shown to the caller
[[nodiscard]] std::string healthJson(const core::FleetManager& manager, int robots,
                                     const std::string& planner);

/// Converts a live order into the row the SQLite store saves.
/// @param order the order to convert
[[nodiscard]] core::StoredOrder storedOrder(const core::ManagedOrder& order);

}  // namespace lotse::service
