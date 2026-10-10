// Tests the HTTP API and the WebSocket feed over a real socket: every endpoint,
// the clear rejection of invalid input, and one live fleet message.

#include "lotse/service/http_api.hpp"

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <string>
#include <thread>

#include "lotse/core/clock.hpp"
#include "lotse/service/runtime.hpp"
#include "lotse/service/sqlite_order_store.hpp"

#include <gtest/gtest.h>
#include <httplib.h>
#include <nlohmann/json.hpp>

namespace {

using lotse::core::SteadyClock;
using lotse::service::ApiServer;
using lotse::service::Runtime;
using lotse::service::ServiceConfig;
using lotse::service::SqliteOrderStore;

/// Waits until the server has a bound port.
/// @param api the server
/// @return the bound port, or 0 when it never became ready
[[nodiscard]] int waitForPort(ApiServer& api) {
    for (int attempt = 0; attempt < 300; ++attempt) {
        const int port = api.port();
        if (port > 0) {
            return port;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}

}  // namespace

TEST(HttpApiTest, ServesTheApiAndTheWebSocket) {
    const std::string database = std::string(::testing::TempDir()) + "/lotse-http-test.db";
    std::filesystem::remove(database);
    SqliteOrderStore store(database);

    ServiceConfig config;
    config.mapPath = std::string(LOTSE_SOURCE_DIR) + "/maps/warehouse_small.map";
    config.fleetSize = 3;
    config.tickRateHz = 0.0;
    config.backgroundPlanning = false;
    config.dashboardDir = std::string(LOTSE_SOURCE_DIR) + "/service/dashboard";

    SteadyClock clock;
    Runtime runtime(config, store, clock);
    ApiServer api(runtime, "127.0.0.1", 0, config.dashboardDir);
    std::thread server([&api] { (void)api.listen(); });
    const int port = waitForPort(api);
    ASSERT_GT(port, 0);

    httplib::Client client("127.0.0.1", port);
    client.set_connection_timeout(5, 0);
    client.set_read_timeout(5, 0);

    const auto health = client.Get("/health");
    ASSERT_TRUE(health);
    EXPECT_EQ(health->status, 200);
    EXPECT_EQ(nlohmann::json::parse(health->body).at("status").get<std::string>(), "ok");

    const auto map = client.Get("/map");
    ASSERT_TRUE(map);
    EXPECT_EQ(map->status, 200);
    EXPECT_EQ(nlohmann::json::parse(map->body).at("width").get<int>(), 16);

    const auto bad =
        client.Post("/orders", R"({"pickup":"nope","dropoff":"dropoff0"})", "application/json");
    ASSERT_TRUE(bad);
    EXPECT_EQ(bad->status, 400);
    EXPECT_NE(bad->body.find("unknown station"), std::string::npos);

    const auto malformed = client.Post("/orders", "not json", "application/json");
    ASSERT_TRUE(malformed);
    EXPECT_EQ(malformed->status, 400);

    const auto created = client.Post(
        "/orders", R"({"pickup":"pickup0","dropoff":"dropoff1","priority":1})", "application/json");
    ASSERT_TRUE(created);
    EXPECT_EQ(created->status, 201);
    const int id = nlohmann::json::parse(created->body).at("id").get<int>();

    const auto list = client.Get("/orders");
    ASSERT_TRUE(list);
    EXPECT_EQ(list->status, 200);
    EXPECT_EQ(nlohmann::json::parse(list->body).size(), 1U);

    const auto one = client.Get("/orders/" + std::to_string(id));
    ASSERT_TRUE(one);
    EXPECT_EQ(one->status, 200);

    const auto missing = client.Get("/orders/999");
    ASSERT_TRUE(missing);
    EXPECT_EQ(missing->status, 404);

    const auto throughput = client.Get("/throughput");
    ASSERT_TRUE(throughput);
    EXPECT_EQ(throughput->status, 200);
    EXPECT_EQ(nlohmann::json::parse(throughput->body).at("orders_open").get<int>(), 1);

    httplib::ws::WebSocketClient socket("ws://127.0.0.1:" + std::to_string(port) + "/ws");
    ASSERT_TRUE(socket.is_valid());
    ASSERT_TRUE(socket.connect());
    socket.set_read_timeout(5, 0);
    std::string message;
    EXPECT_EQ(socket.read(message), httplib::ws::Text);
    EXPECT_NE(message.find("\"type\":\"fleet\""), std::string::npos);
    socket.close();

    const auto cancelled = client.Delete("/orders/" + std::to_string(id));
    ASSERT_TRUE(cancelled);
    EXPECT_EQ(cancelled->status, 200);

    api.stop();
    server.join();
}
