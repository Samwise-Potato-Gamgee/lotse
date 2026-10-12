// Unit tests for the stage 5 service pieces: the config reader, the SQLite order
// store, the real-time fleet manager and the runtime.

#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "lotse/core/clock.hpp"
#include "lotse/core/fleet_manager.hpp"
#include "lotse/core/map.hpp"
#include "lotse/core/order_store.hpp"
#include "lotse/core/prioritized_planner.hpp"
#include "lotse/core/task_assigner.hpp"
#include "lotse/service/config.hpp"
#include "lotse/service/runtime.hpp"
#include "lotse/service/sqlite_order_store.hpp"
#include "lotse/sim/grid_simulator.hpp"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace {

using lotse::core::FleetManager;
using lotse::core::FleetManagerConfig;
using lotse::core::GridCell;
using lotse::core::ManagedOrder;
using lotse::core::Map;
using lotse::core::NearestRobotAssigner;
using lotse::core::OrderState;
using lotse::core::PrioritizedOptions;
using lotse::core::PrioritizedPlanner;
using lotse::core::SteadyClock;
using lotse::core::StoredEvent;
using lotse::core::StoredOrder;
using lotse::service::CancelOutcome;
using lotse::service::OrderRejected;
using lotse::service::Runtime;
using lotse::service::ServiceConfig;
using lotse::sim::GridRobotSide;
using lotse::sim::GridRobotSideConfig;

/// An in-memory OrderStore, so service tests do not touch the filesystem.
class MemoryOrderStore final : public lotse::core::OrderStore {
public:
    void open() override {}

    void saveOrder(const StoredOrder& order) override { orders_[order.id] = order; }

    void appendEvent(const StoredEvent& event) override { events.push_back(event); }

    [[nodiscard]] std::vector<StoredOrder> loadOrders() const override {
        std::vector<StoredOrder> result;
        result.reserve(orders_.size());
        for (const auto& entry : orders_) {
            result.push_back(entry.second);
        }
        return result;
    }

    [[nodiscard]] std::vector<StoredEvent> loadEvents() const override { return events; }

    std::map<int, StoredOrder> orders_;
    std::vector<StoredEvent> events;
};

/// Loads one of the repository maps.
/// @param name the map file name
[[nodiscard]] Map loadMap(const std::string& name) {
    const std::optional<Map> map = Map::fromFile(std::string(LOTSE_SOURCE_DIR) + "/maps/" + name);
    return map.value();
}

/// Returns the first parking cells as the fleet's start cells.
/// @param map the warehouse grid
/// @param fleetSize how many robots
[[nodiscard]] std::vector<GridCell> startsFor(const Map& map, int fleetSize) {
    return {map.parkingCells().begin(), map.parkingCells().begin() + fleetSize};
}

/// Finds the state of one order in a manager's order list.
/// @param orders the order list
/// @param id the order id
[[nodiscard]] OrderState stateOf(const std::vector<ManagedOrder>& orders, int id) {
    for (const ManagedOrder& order : orders) {
        if (order.order.id == id) {
            return order.state;
        }
    }
    return OrderState::Failed;
}

/// Reads the state text of an order from the runtime API.
/// @param runtime the running service
/// @param id the order id
[[nodiscard]] std::string stateText(const Runtime& runtime, int id) {
    const std::optional<std::string> document = runtime.orderJson(id);
    if (!document.has_value()) {
        return "missing";
    }
    return nlohmann::json::parse(document.value()).at("state").get<std::string>();
}

/// Builds a runtime that plans inline and does not tick in the background.
/// @param store the in-memory store
/// @param clock the time source
/// @param fleetSize how many robots
[[nodiscard]] Runtime makeRuntime(MemoryOrderStore& store, SteadyClock& clock, int fleetSize) {
    ServiceConfig config;
    config.mapPath = std::string(LOTSE_SOURCE_DIR) + "/maps/warehouse_small.map";
    config.fleetSize = fleetSize;
    config.tickRateHz = 0.0;
    config.backgroundPlanning = false;
    config.dashboardDir = "";
    return {config, store, clock};
}

}  // namespace

TEST(ConfigTest, LoadsValuesAndDefaults) {
    const std::string path = std::string(::testing::TempDir()) + "/lotse-config-test.conf";
    {
        std::ofstream file(path);
        file << "# a comment\n";
        file << "map = maps/warehouse_small.map\n";
        file << "fleet_size = 3   # inline comment\n";
        file << "port = 1234\n";
        file << "pass_order = false\n";
        file << "stable_order = true\n";
        file << "slowdown_probability = 0.25\n";
    }
    const ServiceConfig config = lotse::service::loadConfig(path);
    EXPECT_EQ(config.mapPath, "maps/warehouse_small.map");
    EXPECT_EQ(config.fleetSize, 3);
    EXPECT_EQ(config.port, 1234);
    EXPECT_FALSE(config.passOrder);
    EXPECT_TRUE(config.stableOrder);
    EXPECT_DOUBLE_EQ(config.slowdownProbability, 0.25);
    EXPECT_EQ(config.planner, "prioritized");
    EXPECT_EQ(config.bindAddress, "127.0.0.1");
}

TEST(ConfigTest, StableOrderIsOffByDefaultAndOnInTheShippedConfig) {
    EXPECT_FALSE(lotse::service::defaultConfig().stableOrder);
    const ServiceConfig shipped =
        lotse::service::loadConfig(std::string(LOTSE_SOURCE_DIR) + "/config/lotse.conf");
    EXPECT_TRUE(shipped.stableOrder);
}

TEST(ConfigTest, RejectsUnknownSettingAndMissingFile) {
    const std::string path = std::string(::testing::TempDir()) + "/lotse-config-bad.conf";
    {
        std::ofstream file(path);
        file << "not_a_setting = 1\n";
    }
    EXPECT_THROW({ (void)lotse::service::loadConfig(path); }, std::runtime_error);
    EXPECT_THROW({ (void)lotse::service::loadConfig(path + ".missing"); }, std::runtime_error);
}

TEST(SqliteOrderStoreTest, RoundTripsOrdersAndEvents) {
    const std::string path = std::string(::testing::TempDir()) + "/lotse-store-test.db";
    std::filesystem::remove(path);
    {
        lotse::service::SqliteOrderStore store(path);
        StoredOrder order;
        order.id = 1;
        order.pickup = GridCell{1, 2};
        order.dropoff = GridCell{3, 4};
        order.priority = 2;
        order.state = "pending";
        order.createdMs = 10;
        order.updatedMs = 20;
        store.saveOrder(order);
        order.state = "completed";
        store.saveOrder(order);
        store.appendEvent(StoredEvent{1, "pending", 10});
        store.appendEvent(StoredEvent{1, "completed", 20});
    }
    lotse::service::SqliteOrderStore reopened(path);
    const std::vector<StoredOrder> orders = reopened.loadOrders();
    ASSERT_EQ(orders.size(), 1U);
    EXPECT_EQ(orders[0].state, "completed");
    EXPECT_EQ(orders[0].pickup, (GridCell{1, 2}));
    const std::vector<StoredEvent> events = reopened.loadEvents();
    ASSERT_EQ(events.size(), 2U);
    EXPECT_EQ(events[1].state, "completed");
}

TEST(FleetManagerTest, RestoreFailsInProgressAndRequeuesPending) {
    const Map map = loadMap("warehouse_small.map");
    GridRobotSide side(startsFor(map, 2), GridRobotSideConfig{});
    SteadyClock clock;
    PrioritizedPlanner planner(PrioritizedOptions{});
    NearestRobotAssigner assigner;
    FleetManagerConfig config;
    config.backgroundPlanning = false;
    FleetManager manager(map, side, planner, assigner, clock, config);
    manager.setBackgroundPlanning(false);

    StoredOrder pending;
    pending.id = 1;
    pending.state = "assigned";
    StoredOrder carried;
    carried.id = 2;
    carried.state = "in_progress";
    manager.restoreOrders({pending, carried});

    const std::vector<ManagedOrder> orders = manager.orders();
    EXPECT_EQ(stateOf(orders, 1), OrderState::Pending);
    EXPECT_EQ(stateOf(orders, 2), OrderState::Failed);
}

TEST(FleetManagerTest, CompletesOrdersWithZeroCollisions) {
    const Map map = loadMap("warehouse_small.map");
    GridRobotSide side(startsFor(map, 3), GridRobotSideConfig{});
    SteadyClock clock;
    PrioritizedPlanner planner(PrioritizedOptions{});
    NearestRobotAssigner assigner;
    FleetManagerConfig config;
    config.backgroundPlanning = false;
    config.windowSeconds = 20.0;
    FleetManager manager(map, side, planner, assigner, clock, config);
    manager.setBackgroundPlanning(false);

    const int first =
        manager.submitOrder(map.station("pickup0").value(), map.station("dropoff1").value());
    const int second =
        manager.submitOrder(map.station("pickup2").value(), map.station("dropoff3").value());

    for (int step = 0; step < 3000; ++step) {
        manager.tick(1.0);
        const std::vector<ManagedOrder> orders = manager.orders();
        if (stateOf(orders, first) == OrderState::Completed &&
            stateOf(orders, second) == OrderState::Completed) {
            break;
        }
    }
    const std::vector<ManagedOrder> orders = manager.orders();
    EXPECT_EQ(stateOf(orders, first), OrderState::Completed);
    EXPECT_EQ(stateOf(orders, second), OrderState::Completed);
    EXPECT_TRUE(manager.safetyReport().safe());
    EXPECT_FALSE(manager.deadlocked());
    EXPECT_EQ(manager.throughput().ordersCompleted, 2);
}

TEST(FleetManagerTest, CancelIsRefusedOncePickedUp) {
    const Map map = loadMap("warehouse_small.map");
    GridRobotSide side(startsFor(map, 1), GridRobotSideConfig{});
    SteadyClock clock;
    PrioritizedPlanner planner(PrioritizedOptions{});
    NearestRobotAssigner assigner;
    FleetManagerConfig config;
    config.backgroundPlanning = false;
    FleetManager manager(map, side, planner, assigner, clock, config);
    manager.setBackgroundPlanning(false);

    const int id =
        manager.submitOrder(map.station("pickup0").value(), map.station("dropoff2").value());
    for (int step = 0; step < 2000; ++step) {
        manager.tick(1.0);
        if (stateOf(manager.orders(), id) == OrderState::InProgress) {
            break;
        }
    }
    EXPECT_EQ(stateOf(manager.orders(), id), OrderState::InProgress);
    EXPECT_FALSE(manager.cancelOrder(id));
}

TEST(FleetManagerTest, AdoptionWhileBetweenCellsKeepsOneRobotPerCell) {
    const Map map = loadMap("warehouse_small.map");
    GridRobotSideConfig sim;
    sim.cellSeconds = 3.0;
    GridRobotSide side(startsFor(map, 4), sim);
    SteadyClock clock;
    PrioritizedPlanner planner(PrioritizedOptions{});
    NearestRobotAssigner assigner;
    FleetManagerConfig config;
    config.backgroundPlanning = false;
    config.windowSeconds = 1.0;
    FleetManager manager(map, side, planner, assigner, clock, config);
    manager.setBackgroundPlanning(false);

    for (int index = 0; index < 4; ++index) {
        const std::string pickup = "pickup" + std::to_string(index);
        const std::string dropoff = "dropoff" + std::to_string((index + 1) % 4);
        manager.submitOrder(map.station(pickup).value(), map.station(dropoff).value());
    }

    bool sawRobotBetweenCells = false;
    for (int step = 0; step < 1200; ++step) {
        manager.tick(1.0);
        const std::vector<lotse::core::RobotStatus> statuses = manager.robotStatuses();
        for (std::size_t a = 0; a < statuses.size(); ++a) {
            if (statuses[a].target != statuses[a].cell) {
                sawRobotBetweenCells = true;
            }
            for (std::size_t b = a + 1; b < statuses.size(); ++b) {
                ASSERT_NE(statuses[a].cell, statuses[b].cell);
            }
        }
    }
    EXPECT_TRUE(sawRobotBetweenCells);
    EXPECT_EQ(manager.safetyReport().collisions, 0);
    EXPECT_EQ(manager.safetyReport().swaps, 0);
}

TEST(RuntimeTest, RejectsUnknownStationsAndCompletesOrders) {
    MemoryOrderStore store;
    SteadyClock clock;
    Runtime runtime = makeRuntime(store, clock, 4);

    EXPECT_THROW(runtime.submitOrder("nope", "dropoff0", 0), OrderRejected);
    EXPECT_THROW(runtime.submitOrder("pickup0", "nope", 0), OrderRejected);
    EXPECT_THROW(runtime.submitOrder("dropoff0", "dropoff1", 0), OrderRejected);

    const int first = runtime.submitOrder("pickup0", "dropoff1", 0);
    const int second = runtime.submitOrder("pickup2", "dropoff3", 0);
    EXPECT_EQ(runtime.cancelOrder(999), CancelOutcome::NotFound);

    for (int step = 0; step < 5000; ++step) {
        runtime.tickOnce(1.0);
        if (stateText(runtime, first) == "completed" && stateText(runtime, second) == "completed") {
            break;
        }
    }
    EXPECT_EQ(stateText(runtime, first), "completed");
    EXPECT_EQ(stateText(runtime, second), "completed");
    EXPECT_NE(runtime.fleetJson().find("\"collisions\":0"), std::string::npos);
    EXPECT_NE(runtime.fleetJson().find("\"swaps\":0"), std::string::npos);
    EXPECT_EQ(runtime.cancelOrder(first), CancelOutcome::Finished);
}

TEST(RuntimeTest, CancelStopsAPendingOrder) {
    MemoryOrderStore store;
    SteadyClock clock;
    Runtime runtime = makeRuntime(store, clock, 1);
    const int id = runtime.submitOrder("pickup0", "dropoff1", 0);
    EXPECT_EQ(runtime.cancelOrder(id), CancelOutcome::Cancelled);
    EXPECT_EQ(stateText(runtime, id), "cancelled");
}
