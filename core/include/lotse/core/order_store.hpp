// Persists orders and their state transitions. The interface lives in the core
// so the core stays free of SQLite; the service provides the SQLite version.

#pragma once

#include <string>
#include <string_view>
#include <vector>

#include "lotse/core/grid.hpp"

namespace lotse::core {

/// One stored order, with its state as text.
struct StoredOrder {
    int id = 0;
    GridCell pickup{};
    GridCell dropoff{};
    int priority = 0;
    std::string state;
    long long createdMs = 0;
    long long updatedMs = 0;
};

/// One stored state transition.
struct StoredEvent {
    int orderId = 0;
    std::string state;
    long long atMs = 0;
};

/// Stores orders and their state history across restarts.
class OrderStore {
public:
    OrderStore() = default;
    OrderStore(const OrderStore&) = delete;
    OrderStore& operator=(const OrderStore&) = delete;
    OrderStore(OrderStore&&) = delete;
    OrderStore& operator=(OrderStore&&) = delete;
    virtual ~OrderStore() = default;

    /// Creates the schema if it is missing.
    virtual void open() = 0;

    /// Inserts or updates one order.
    /// @param order the order to store
    virtual void saveOrder(const StoredOrder& order) = 0;

    /// Appends one state transition.
    /// @param event the transition to append
    virtual void appendEvent(const StoredEvent& event) = 0;

    /// Returns every stored order, oldest id first.
    [[nodiscard]] virtual std::vector<StoredOrder> loadOrders() const = 0;

    /// Returns every stored transition, oldest first.
    [[nodiscard]] virtual std::vector<StoredEvent> loadEvents() const = 0;
};

}  // namespace lotse::core
