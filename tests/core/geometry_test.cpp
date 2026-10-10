// Tests the grid geometry helpers: cell to metre conversion, the
// differential-drive step, heading errors and leg progress.

#include "lotse/core/geometry.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace {

using lotse::core::cellAt;
using lotse::core::cellCentre;
using lotse::core::GridCell;
using lotse::core::headingError;
using lotse::core::integrateDiffDrive;
using lotse::core::legProgress;
using lotse::core::Pose2D;

constexpr double kTolerance = 1e-9;

}  // namespace

TEST(GeometryTest, CellCentreAndCellAtRoundTrip) {
    const GridCell cell{3, 5};
    const Pose2D centre = cellCentre(cell, 1.0);
    EXPECT_DOUBLE_EQ(centre.x, 3.5);
    EXPECT_DOUBLE_EQ(centre.y, 5.5);
    EXPECT_EQ(cellAt(centre, 1.0), cell);
    EXPECT_EQ(cellAt(Pose2D{3.9, 5.1, 0.0}, 1.0), cell);
}

TEST(GeometryTest, IntegrateStraightAndTurn) {
    const Pose2D straight = integrateDiffDrive(Pose2D{0.0, 0.0, 0.0}, 1.0, 0.0, 2.0);
    EXPECT_NEAR(straight.x, 2.0, kTolerance);
    EXPECT_NEAR(straight.y, 0.0, kTolerance);
    const Pose2D turned = integrateDiffDrive(Pose2D{0.0, 0.0, 0.0}, 0.0, 1.0, 1.0);
    EXPECT_NEAR(turned.theta, 1.0, kTolerance);
}

TEST(GeometryTest, HeadingErrorWrapsAround) {
    EXPECT_NEAR(headingError(0.1, 0.0), 0.1, kTolerance);
    EXPECT_NEAR(headingError(-3.0, 3.0), 0.283185307179586, 1e-9);
    EXPECT_NEAR(headingError(0.0, 0.0), 0.0, kTolerance);
}

TEST(GeometryTest, LegProgressClampsToTheLeg) {
    const GridCell from{0, 0};
    const GridCell to{1, 0};
    EXPECT_NEAR(legProgress(Pose2D{0.5, 0.5, 0.0}, from, to, 1.0), 0.0, kTolerance);
    EXPECT_NEAR(legProgress(Pose2D{1.0, 0.5, 0.0}, from, to, 1.0), 0.5, kTolerance);
    EXPECT_NEAR(legProgress(Pose2D{1.5, 0.5, 0.0}, from, to, 1.0), 1.0, kTolerance);
    EXPECT_NEAR(legProgress(Pose2D{9.0, 9.0, 0.0}, from, to, 1.0), 1.0, kTolerance);
}
