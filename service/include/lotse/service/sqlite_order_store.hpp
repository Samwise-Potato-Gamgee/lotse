// Stores orders and their state history in a local SQLite file so open orders
// survive a restart. One file, no server, no extra runtime dependency.

#pragma once

#include <memory>
#include <string>

#include "lotse/core/order_store.hpp"

namespace lotse::service {

/// A core::OrderStore backed by one SQLite database file.
class SqliteOrderStore final : public core::OrderStore {
public:
    /// Opens the file and prepares the schema.
    /// @param path the SQLite file to create or open
    explicit SqliteOrderStore(const std::string& path);
    ~SqliteOrderStore() override;

    SqliteOrderStore(const SqliteOrderStore&) = delete;
    SqliteOrderStore& operator=(const SqliteOrderStore&) = delete;
    SqliteOrderStore(SqliteOrderStore&&) = delete;
    SqliteOrderStore& operator=(SqliteOrderStore&&) = delete;

    /// Creates the tables when they are missing.
    void open() override;

    /// Inserts or replaces one order.
    /// @param order the order to store
    void saveOrder(const core::StoredOrder& order) override;

    /// Appends one state transition.
    /// @param event the transition to append
    void appendEvent(const core::StoredEvent& event) override;

    /// Returns every stored order, oldest id first.
    [[nodiscard]] std::vector<core::StoredOrder> loadOrders() const override;

    /// Returns every stored transition, oldest first.
    [[nodiscard]] std::vector<core::StoredEvent> loadEvents() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace lotse::service
