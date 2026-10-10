// Implements the SQLite order store declared in sqlite_order_store.hpp.

#include "lotse/service/sqlite_order_store.hpp"

#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <sqlite3.h>

namespace lotse::service {
namespace {

/// Runs a statement with no results, throwing on failure.
/// @param db the open database
/// @param sql the statement to run
void run(sqlite3* db, const char* sql) {
    char* message = nullptr;
    if (sqlite3_exec(db, sql, nullptr, nullptr, &message) != SQLITE_OK) {
        const std::string text = message != nullptr ? message : "unknown error";
        sqlite3_free(message);
        throw std::runtime_error("sqlite: " + text);
    }
}

/// Prepares a statement, throwing on failure.
/// @param db the open database
/// @param sql the statement text
/// @return the prepared statement, owned by the caller
[[nodiscard]] sqlite3_stmt* prepare(sqlite3* db, const char* sql) {
    sqlite3_stmt* statement = nullptr;
    if (sqlite3_prepare_v2(db, sql, -1, &statement, nullptr) != SQLITE_OK) {
        throw std::runtime_error(std::string("sqlite: ") + sqlite3_errmsg(db));
    }
    return statement;
}

/// Binds text to a statement parameter.
/// @param statement the statement
/// @param index the one based parameter index
/// @param value the text to bind
void bindText(sqlite3_stmt* statement, int index, const std::string& value) {
    sqlite3_bind_text(statement, index, value.c_str(), -1, SQLITE_TRANSIENT);
}

/// Reads a text column.
/// @param statement the statement positioned on a row
/// @param column the zero based column index
[[nodiscard]] std::string columnText(sqlite3_stmt* statement, int column) {
    const unsigned char* text = sqlite3_column_text(statement, column);
    return text != nullptr ? reinterpret_cast<const char*>(text) : std::string{};
}

}  // namespace

struct SqliteOrderStore::Impl {
    std::string path;
    sqlite3* db = nullptr;
    mutable std::mutex mutex;
};

SqliteOrderStore::SqliteOrderStore(const std::string& path) : impl_(std::make_unique<Impl>()) {
    impl_->path = path;
    if (sqlite3_open(path.c_str(), &impl_->db) != SQLITE_OK) {
        throw std::runtime_error("sqlite: cannot open " + path);
    }
    sqlite3_busy_timeout(impl_->db, 5000);
    open();
}

SqliteOrderStore::~SqliteOrderStore() {
    if (impl_->db != nullptr) {
        sqlite3_close(impl_->db);
    }
}

void SqliteOrderStore::open() {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    run(impl_->db,
        "CREATE TABLE IF NOT EXISTS orders ("
        "id INTEGER PRIMARY KEY,"
        "pickup_x INTEGER NOT NULL, pickup_y INTEGER NOT NULL,"
        "dropoff_x INTEGER NOT NULL, dropoff_y INTEGER NOT NULL,"
        "priority INTEGER NOT NULL, state TEXT NOT NULL,"
        "created_ms INTEGER NOT NULL, updated_ms INTEGER NOT NULL)");
    run(impl_->db,
        "CREATE TABLE IF NOT EXISTS order_events ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "order_id INTEGER NOT NULL, state TEXT NOT NULL,"
        "at_ms INTEGER NOT NULL)");
    run(impl_->db, "CREATE INDEX IF NOT EXISTS order_events_order ON order_events(order_id)");
}

void SqliteOrderStore::saveOrder(const core::StoredOrder& order) {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    sqlite3_stmt* statement = prepare(
        impl_->db,
        "INSERT OR REPLACE INTO orders "
        "(id, pickup_x, pickup_y, dropoff_x, dropoff_y, priority, state, created_ms, updated_ms) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)");
    sqlite3_bind_int(statement, 1, order.id);
    sqlite3_bind_int(statement, 2, order.pickup.x);
    sqlite3_bind_int(statement, 3, order.pickup.y);
    sqlite3_bind_int(statement, 4, order.dropoff.x);
    sqlite3_bind_int(statement, 5, order.dropoff.y);
    sqlite3_bind_int(statement, 6, order.priority);
    bindText(statement, 7, order.state);
    sqlite3_bind_int64(statement, 8, order.createdMs);
    sqlite3_bind_int64(statement, 9, order.updatedMs);
    const int result = sqlite3_step(statement);
    sqlite3_finalize(statement);
    if (result != SQLITE_DONE) {
        throw std::runtime_error(std::string("sqlite: ") + sqlite3_errmsg(impl_->db));
    }
}

void SqliteOrderStore::appendEvent(const core::StoredEvent& event) {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    sqlite3_stmt* statement =
        prepare(impl_->db, "INSERT INTO order_events (order_id, state, at_ms) VALUES (?, ?, ?)");
    sqlite3_bind_int(statement, 1, event.orderId);
    bindText(statement, 2, event.state);
    sqlite3_bind_int64(statement, 3, event.atMs);
    const int result = sqlite3_step(statement);
    sqlite3_finalize(statement);
    if (result != SQLITE_DONE) {
        throw std::runtime_error(std::string("sqlite: ") + sqlite3_errmsg(impl_->db));
    }
}

std::vector<core::StoredOrder> SqliteOrderStore::loadOrders() const {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    std::vector<core::StoredOrder> orders;
    sqlite3_stmt* statement =
        prepare(impl_->db,
                "SELECT id, pickup_x, pickup_y, dropoff_x, dropoff_y, priority, state, created_ms, "
                "updated_ms FROM orders ORDER BY id");
    while (sqlite3_step(statement) == SQLITE_ROW) {
        core::StoredOrder order;
        order.id = sqlite3_column_int(statement, 0);
        order.pickup =
            core::GridCell{sqlite3_column_int(statement, 1), sqlite3_column_int(statement, 2)};
        order.dropoff =
            core::GridCell{sqlite3_column_int(statement, 3), sqlite3_column_int(statement, 4)};
        order.priority = sqlite3_column_int(statement, 5);
        order.state = columnText(statement, 6);
        order.createdMs = sqlite3_column_int64(statement, 7);
        order.updatedMs = sqlite3_column_int64(statement, 8);
        orders.push_back(std::move(order));
    }
    sqlite3_finalize(statement);
    return orders;
}

std::vector<core::StoredEvent> SqliteOrderStore::loadEvents() const {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    std::vector<core::StoredEvent> events;
    sqlite3_stmt* statement =
        prepare(impl_->db, "SELECT order_id, state, at_ms FROM order_events ORDER BY id");
    while (sqlite3_step(statement) == SQLITE_ROW) {
        core::StoredEvent event;
        event.orderId = sqlite3_column_int(statement, 0);
        event.state = columnText(statement, 1);
        event.atMs = sqlite3_column_int64(statement, 2);
        events.push_back(std::move(event));
    }
    sqlite3_finalize(statement);
    return events;
}

}  // namespace lotse::service
