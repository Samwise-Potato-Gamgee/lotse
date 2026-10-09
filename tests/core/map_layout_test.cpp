// Checks the shipped warehouse maps: they have enough parking cells, the
// parking and station cells are free and distinct, and blocking every parking
// cell still leaves the stations reachable from each other.

#include <optional>
#include <set>
#include <string>
#include <vector>

#include "lotse/core/distance.hpp"
#include "lotse/core/map.hpp"
#include "lotse/core/order.hpp"

#include <gtest/gtest.h>

namespace lotse {
namespace {

/// Loads a map from the repository's maps directory.
/// @param name the map file name
[[nodiscard]] core::Map loadMap(const std::string& name) {
    const std::string path = std::string(LOTSE_SOURCE_DIR) + "/maps/" + name;
    const std::optional<core::Map> map = core::Map::fromFile(path);
    EXPECT_TRUE(map.has_value()) << path;
    return map.value_or(core::Map::fromText("height 1\nwidth 1\nmap\n.\n"));
}

// Every warehouse map has at least 20 parking cells and 2 pickup and drop-off
// stations, and no cell is both parking and a station.
TEST(MapLayoutTest, ParkingAndStationsAreDistinct) {
    for (const std::string& name :
         {"warehouse_small.map", "warehouse_rooms.map", "warehouse_aisles.map",
          "warehouse_open.map", "warehouse_large.map"}) {
        const core::Map map = loadMap(name);
        SCOPED_TRACE(name);
        EXPECT_GE(map.parkingCells().size(), 20U);
        EXPECT_GE(core::stationCellsWithPrefix(map, "pickup").size(), 2U);
        EXPECT_GE(core::stationCellsWithPrefix(map, "dropoff").size(), 2U);

        std::set<std::pair<int, int>> parking;
        for (const core::GridCell& cell : map.parkingCells()) {
            EXPECT_TRUE(map.isFree(cell));
            parking.emplace(cell.x, cell.y);
        }
        EXPECT_EQ(parking.size(), map.parkingCells().size());
        for (const std::pair<std::string, core::GridCell>& station : map.stations()) {
            EXPECT_TRUE(map.isFree(station.second));
            EXPECT_EQ(parking.count({station.second.x, station.second.y}), 0U);
        }
    }
}

// With every parking cell blocked the stations must still be reachable.
TEST(MapLayoutTest, StationsStayReachableWithEveryoneParked) {
    for (const std::string& name :
         {"warehouse_small.map", "warehouse_rooms.map", "warehouse_aisles.map",
          "warehouse_open.map", "warehouse_large.map"}) {
        const core::Map map = loadMap(name);
        SCOPED_TRACE(name);
        const core::Map blocked = map.withObstacles(map.parkingCells());
        const std::vector<core::GridCell> stations = core::stationCellsWithPrefix(map, "pickup");
        const std::vector<core::GridCell> dropoffs = core::stationCellsWithPrefix(map, "dropoff");
        for (const core::GridCell& pickup : stations) {
            for (const core::GridCell& dropoff : dropoffs) {
                EXPECT_GE(core::shortestPathSteps(blocked, pickup, dropoff), 0);
            }
        }
    }
}

}  // namespace
}  // namespace lotse
