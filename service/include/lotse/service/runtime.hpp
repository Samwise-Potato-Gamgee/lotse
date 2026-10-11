// The service runtime: it owns the map, the grid simulator, the planner, the
// real-time fleet manager and the ticker thread, and it turns their state into
// the JSON the HTTP API and the dashboard consume.

#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "lotse/core/clock.hpp"
#include "lotse/core/order_store.hpp"
#include "lotse/service/config.hpp"
#include "lotse/service/fleet_api.hpp"

namespace lotse::service {

/// Runs the fleet manager and the grid simulator in real time.
class Runtime : public FleetApiSource {
public:
    /// Builds the map, the simulator and the manager, and restores saved orders.
    /// @param config the service settings
    /// @param store the order store, which outlives the runtime
    /// @param clock the time source, which outlives the runtime
    Runtime(ServiceConfig config, core::OrderStore& store, core::Clock& clock);
    ~Runtime();

    Runtime(const Runtime&) = delete;
    Runtime& operator=(const Runtime&) = delete;
    Runtime(Runtime&&) = delete;
    Runtime& operator=(Runtime&&) = delete;

    /// Starts the ticker thread.
    void start();

    /// Stops the ticker thread.
    void stop();

    /// Runs one grid step and persists the resulting order events.
    /// @param seconds how much simulated time the step covers
    void tickOnce(double seconds);

    /// Validates and accepts a new order.
    /// @param pickupName the named pickup station
    /// @param dropoffName the named drop-off station
    /// @param priority higher is served sooner
    /// @return the new order id
    /// @throws OrderRejected when a station is unknown or the request is invalid
    int submitOrder(const std::string& pickupName, const std::string& dropoffName,
                    int priority) override;

    /// Cancels an order that has not been picked up yet.
    /// @param orderId the order to cancel
    /// @return Cancelled, InProgress or NotFound
    CancelOutcome cancelOrder(int orderId) override;

    /// Installs the callback that receives live update messages.
    /// @param sink the callback, empty to stop updates
    void setUpdateSink(std::function<void(std::string)> sink) override;

    /// Returns the effective configuration.
    [[nodiscard]] const ServiceConfig& config() const noexcept;

    /// Returns the JSON map document.
    [[nodiscard]] std::string mapJson() const override;

    /// Returns the JSON fleet document.
    [[nodiscard]] std::string fleetJson() const override;

    /// Returns the JSON order list.
    [[nodiscard]] std::string ordersJson() const override;

    /// Returns one order as JSON.
    /// @param id the order id
    [[nodiscard]] std::optional<std::string> orderJson(int id) const override;

    /// Returns the JSON throughput document.
    [[nodiscard]] std::string throughputJson() const override;

    /// Returns the JSON health document.
    [[nodiscard]] std::string healthJson() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace lotse::service
