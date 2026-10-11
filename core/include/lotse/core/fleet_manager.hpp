// The real-time fleet manager: a long-running object that keeps a fleet moving
// in wall time. Every tick it publishes the cells each robot may enter under the
// pass-order rule, reads back true positions, and adopts a background plan only
// when that plan can be re-anchored at the robots' current cells.

#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>
#include <vector>

#include "lotse/core/clock.hpp"
#include "lotse/core/map.hpp"
#include "lotse/core/order.hpp"
#include "lotse/core/order_store.hpp"
#include "lotse/core/planner.hpp"
#include "lotse/core/robot_side.hpp"
#include "lotse/core/task_assigner.hpp"
#include "lotse/judge/judge.hpp"

namespace lotse::core {

/// The lifecycle state of one order.
enum class OrderState : std::uint8_t {
    Pending,
    Assigned,
    InProgress,
    Completed,
    Cancelled,
    Failed
};

/// Returns the text form of an order state.
/// @param state the state to format
[[nodiscard]] const char* toString(OrderState state) noexcept;

/// Parses the text form of an order state.
/// @param text the text to parse
/// @return the state, or nothing when the text is unknown
[[nodiscard]] std::optional<OrderState> orderStateFromText(std::string_view text) noexcept;

/// One order as the manager and the API see it.
struct ManagedOrder {
    Order order{};
    OrderState state = OrderState::Pending;
    int robot = -1;
    int arrivalStep = 0;
    long long createdMs = 0;
    long long updatedMs = 0;
};

/// One order state change, used for the SQLite log and the live feed.
struct OrderEvent {
    int orderId = 0;
    OrderState state = OrderState::Pending;
    int robot = -1;
    long long atMs = 0;
};

/// Running counters for the throughput view.
struct ThroughputStats {
    int ordersCompleted = 0;
    int ordersOpen = 0;
    int ordersCancelled = 0;
    int ordersFailed = 0;
    int steps = 0;
    double elapsedSeconds = 0.0;
    double ordersPerHour = 0.0;
    int planningCalls = 0;
    int planningFailures = 0;
    int planAdoptions = 0;
    int planRejections = 0;
};

/// Why a robot was held at a cell during one tick.
enum class HoldReason : std::uint8_t { None, CellOccupied, PassOrder, NoWaypoint };

/// Extra per-run counters used to diagnose slow runs. They are only filled when
/// the diagnostics switch is on, so ordinary runs pay nothing.
struct ManagerDiagnostics {
    std::vector<double> cellOccupiedWaitSeconds;
    std::vector<double> passOrderWaitSeconds;
    std::vector<double> noWaypointSeconds;
    int rightOfWayFlips = 0;
    int rightOfWayComparisons = 0;
    int planningTimeouts = 0;
    int planningRoutingFailures = 0;
    double secondsWithoutFreshPlan = 0.0;
    double maxSecondsWithoutFreshPlan = 0.0;
};

/// Tuning for the real-time manager. The three time settings are in simulated
/// seconds; the manager converts them to steps with the latest tick length.
struct FleetManagerConfig {
    double windowSeconds = 2.0;
    int planTimeLimitMs = 500;
    double priorityAgeingSeconds = 1.25;
    double deadlockPatienceSeconds = 15.0;
    bool backgroundPlanning = true;
    bool passOrderEnabled = true;
    bool diagnostics = false;
};

/// Keeps one fleet of robots serving a stream of orders in real time.
class FleetManager {
public:
    /// Owns the fleet loop; the robot side, planner, assigner and clock outlive it.
    /// @param map the warehouse grid
    /// @param side the robot side the manager drives
    /// @param planner the primary fleet planner
    /// @param assigner the task assigner
    /// @param clock the time source
    /// @param config replanning, ageing and deadlock tuning
    /// @param fallback optional second planner used when the primary fails
    FleetManager(const Map& map, RobotSide& side, Planner& planner, TaskAssigner& assigner,
                 Clock& clock, FleetManagerConfig config = {}, Planner* fallback = nullptr);
    ~FleetManager();

    FleetManager(const FleetManager&) = delete;
    FleetManager& operator=(const FleetManager&) = delete;
    FleetManager(FleetManager&&) = delete;
    FleetManager& operator=(FleetManager&&) = delete;

    /// Runs the planner inline instead of on the worker thread, so tests are
    /// repeatable without threads.
    /// @param enabled true to plan inline
    void setBackgroundPlanning(bool enabled);

    /// Reloads orders saved before a restart. Orders that were not finished go
    /// back to pending; orders that had been picked up are failed, because the
    /// robot's carried load cannot be recovered.
    /// @param stored the orders read from the store
    void restoreOrders(const std::vector<StoredOrder>& stored);

    /// Adds an order to the stream.
    /// @param pickup the pickup cell
    /// @param dropoff the drop-off cell
    /// @param priority higher is served sooner
    /// @return the new order id
    int submitOrder(GridCell pickup, GridCell dropoff, int priority = 0);

    /// Cancels an order that has not been picked up yet. An order whose robot
    /// is already carrying the load is refused.
    /// @param orderId the order to cancel
    /// @return true when the order was cancelled
    bool cancelOrder(int orderId);

    /// Advances the fleet by one grid step: settle positions, adopt a finished
    /// plan, publish the waypoints the pass-order rule allows and advance the
    /// robot side.
    /// @param seconds how much simulated time this step covers
    void tick(double seconds);

    /// Returns every order, oldest id first.
    [[nodiscard]] std::vector<ManagedOrder> orders() const;

    /// Looks up one order.
    /// @param id the order id
    [[nodiscard]] std::optional<ManagedOrder> order(int id) const;

    /// Returns the latest status of every robot.
    [[nodiscard]] std::vector<RobotStatus> robotStatuses() const;

    /// Returns the order id of every robot, or -1 when it has none.
    [[nodiscard]] std::vector<int> robotOrderIds() const;

    /// Returns true when no robot has made progress for the deadlock patience.
    [[nodiscard]] bool deadlocked() const;

    /// Returns the safety verdict over every position recorded so far.
    [[nodiscard]] judge::Report safetyReport() const;

    /// Returns the running throughput counters.
    [[nodiscard]] ThroughputStats throughput() const;

    /// Returns the diagnostic counters collected since the manager was created.
    /// The vectors are empty unless the config enabled diagnostics.
    [[nodiscard]] ManagerDiagnostics diagnostics() const;

    /// Returns one line per robot with its route index and the next few cells.
    /// The liveness benchmark uses it to explain a stalled run.
    [[nodiscard]] std::string routeDump() const;

    /// Returns and clears the order events since the last call.
    [[nodiscard]] std::vector<OrderEvent> drainEvents();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace lotse::core
