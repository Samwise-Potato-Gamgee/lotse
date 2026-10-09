#include "lotse/core/grid.hpp"

#include <gtest/gtest.h>

namespace lotse::core {
namespace {

TEST(GridCellTest, EqualityComparesBothCoordinates) {
    const GridCell a{1, 2};
    const GridCell same{1, 2};
    const GridCell swapped{2, 1};
    EXPECT_EQ(a, same);
    EXPECT_NE(a, swapped);
}

TEST(ManhattanDistanceTest, IsZeroForTheSameCell) {
    EXPECT_EQ(manhattan_distance(GridCell{4, 7}, GridCell{4, 7}), 0);
}

TEST(ManhattanDistanceTest, CountsHorizontalAndVerticalSteps) {
    EXPECT_EQ(manhattan_distance(GridCell{0, 0}, GridCell{3, 4}), 7);
    EXPECT_EQ(manhattan_distance(GridCell{3, 4}, GridCell{0, 0}), 7);
    EXPECT_EQ(manhattan_distance(GridCell{-2, 5}, GridCell{1, 1}), 7);
}

TEST(ManhattanDistanceTest, IsSymmetric) {
    const GridCell a{9, -3};
    const GridCell b{-4, 2};
    EXPECT_EQ(manhattan_distance(a, b), manhattan_distance(b, a));
}

TEST(ToStringTest, FormatsCoordinates) {
    EXPECT_EQ(to_string(GridCell{3, -4}), "(3, -4)");
}

}  // namespace
}  // namespace lotse::core
