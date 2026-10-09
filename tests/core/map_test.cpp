// Unit tests for the map reader.

#include "lotse/core/map.hpp"

#include <cstdio>
#include <fstream>
#include <optional>
#include <string>

#include <gtest/gtest.h>

namespace lotse::core {
namespace {

constexpr const char* kTinyMap =
    "type grid\n"
    "name tiny\n"
    "height 3\n"
    "width 4\n"
    "map\n"
    "..#.\n"
    ".##.\n"
    "....\n"
    "stations\n"
    "shelf 0 0\n"
    "dock 3 2\n";

// Checks the name, width and height of a parsed map.
TEST(MapTest, ReadsHeader) {
    const Map map = Map::fromText(kTinyMap);
    EXPECT_EQ(map.name(), "tiny");
    EXPECT_EQ(map.width(), 4);
    EXPECT_EQ(map.height(), 3);
}

// Checks that obstacles and free cells are distinguished.
TEST(MapTest, ReadsObstacles) {
    const Map map = Map::fromText(kTinyMap);
    EXPECT_TRUE(map.isFree(GridCell{0, 0}));
    EXPECT_FALSE(map.isFree(GridCell{2, 0}));
    EXPECT_FALSE(map.isFree(GridCell{1, 1}));
    EXPECT_TRUE(map.isFree(GridCell{3, 2}));
    EXPECT_FALSE(map.contains(GridCell{4, 0}));
    EXPECT_FALSE(map.isFree(GridCell{-1, 0}));
}

// Checks the free cell count in row-major order.
TEST(MapTest, CountsFreeCells) {
    const Map map = Map::fromText(kTinyMap);
    EXPECT_EQ(map.freeCells().size(), 9U);
    EXPECT_EQ(map.freeCells().front(), (GridCell{0, 0}));
}

// Checks named station lookup.
TEST(MapTest, ReadsStations) {
    const Map map = Map::fromText(kTinyMap);
    const std::optional<GridCell> shelf = map.station("shelf");
    const std::optional<GridCell> dock = map.station("dock");
    ASSERT_TRUE(shelf.has_value());
    ASSERT_TRUE(dock.has_value());
    EXPECT_EQ(shelf.value(), (GridCell{0, 0}));
    EXPECT_EQ(dock.value(), (GridCell{3, 2}));
    EXPECT_FALSE(map.station("missing").has_value());
    EXPECT_EQ(map.stations().size(), 2U);
}

// Checks that the public MAPF benchmark characters are accepted.
TEST(MapTest, ReadsBenchmarkFormat) {
    const Map map = Map::fromText("type octile\nheight 2\nwidth 5\nmap\n..G.S\n@@@@@\n");
    EXPECT_EQ(map.width(), 5);
    EXPECT_EQ(map.height(), 2);
    EXPECT_TRUE(map.isFree(GridCell{2, 0}));
    EXPECT_TRUE(map.isFree(GridCell{4, 0}));
    EXPECT_FALSE(map.isFree(GridCell{0, 1}));
}

// Checks that text output can be parsed back.
TEST(MapTest, RoundTripsThroughText) {
    const Map map = Map::fromText(kTinyMap);
    const Map again = Map::fromText(map.toString());
    EXPECT_EQ(again.toString(), map.toString());
    EXPECT_EQ(again.name(), "tiny");
}

// Checks that malformed maps are rejected.
TEST(MapTest, RejectsMalformedInput) {
    EXPECT_THROW((void)Map::fromText("height 2\nwidth 2\n"), std::runtime_error);
    EXPECT_THROW((void)Map::fromText("height 2\nwidth 2\nmap\n..\n"), std::runtime_error);
    EXPECT_THROW((void)Map::fromText("height 1\nwidth 2\nmap\n...\n"), std::runtime_error);
    EXPECT_THROW((void)Map::fromText("height 1\nwidth 2\nmap\n..\nstations\ns 5 5\n"),
                 std::runtime_error);
}

// Checks file loading, success and failure.
TEST(MapTest, LoadsFromFile) {
    const std::string path = "/tmp/lotse_map_test.map";
    {
        std::ofstream out{path};
        out << kTinyMap;
    }
    const std::optional<Map> map = Map::fromFile(path);
    ASSERT_TRUE(map.has_value());
    EXPECT_EQ(map.value().name(), "tiny");
    EXPECT_FALSE(Map::fromFile("/tmp/lotse_map_test_missing.map").has_value());
    std::remove(path.c_str());
}

}  // namespace
}  // namespace lotse::core
