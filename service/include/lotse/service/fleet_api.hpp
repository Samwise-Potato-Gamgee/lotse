// The state interface the HTTP front end needs. Both the grid service runtime
// and the ROS 2 fleet node implement it, so one ApiServer serves either one.

#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>

namespace lotse::service {

/// Thrown when an order cannot be accepted. The message is a clear reason.
class OrderRejected : public std::runtime_error {
public:
    /// Creates the rejection with its reason.
    /// @param message the reason shown to the caller
    explicit OrderRejected(const std::string& message) : std::runtime_error(message) {}
};

/// What happened when a cancellation was requested.
enum class CancelOutcome : std::uint8_t { Cancelled, InProgress, Finished, NotFound };

/// The fleet state and order operations the HTTP and WebSocket front end reads.
class FleetApiSource {
public:
    FleetApiSource() = default;
    FleetApiSource(const FleetApiSource&) = delete;
    FleetApiSource& operator=(const FleetApiSource&) = delete;
    FleetApiSource(FleetApiSource&&) = delete;
    FleetApiSource& operator=(FleetApiSource&&) = delete;
    virtual ~FleetApiSource() = default;

    /// Returns the JSON map document.
    [[nodiscard]] virtual std::string mapJson() const = 0;

    /// Returns the JSON fleet document.
    [[nodiscard]] virtual std::string fleetJson() const = 0;

    /// Returns the JSON order list.
    [[nodiscard]] virtual std::string ordersJson() const = 0;

    /// Returns one order as JSON.
    /// @param id the order id
    [[nodiscard]] virtual std::optional<std::string> orderJson(int id) const = 0;

    /// Returns the JSON throughput document.
    [[nodiscard]] virtual std::string throughputJson() const = 0;

    /// Returns the JSON health document.
    [[nodiscard]] virtual std::string healthJson() const = 0;

    /// Validates and accepts a new order.
    /// @param pickupName the named pickup station
    /// @param dropoffName the named drop-off station
    /// @param priority higher is served sooner
    /// @return the new order id
    /// @throws OrderRejected when a station is unknown or the request is invalid
    virtual int submitOrder(const std::string& pickupName, const std::string& dropoffName,
                            int priority) = 0;

    /// Cancels an order that has not been picked up yet.
    /// @param orderId the order to cancel
    virtual CancelOutcome cancelOrder(int orderId) = 0;

    /// Installs the callback that receives live update messages.
    /// @param sink the callback, empty to stop updates
    virtual void setUpdateSink(std::function<void(std::string)> sink) = 0;
};

}  // namespace lotse::service
