// Grid to metre conversions and the differential-drive step shared by the ROS 2
// driver, the stand-in simulator and the ROS 2 robot side. Keeping this maths in
// the core means it can be unit tested without a ROS runtime.

#pragma once

#include "lotse/core/grid.hpp"

namespace lotse::core {

/// A planar pose in metres; theta is the heading in radians.
struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

/// Returns the centre of a cell in metres, heading zero.
/// @param cell the grid cell
/// @param cellSize metres per cell
[[nodiscard]] Pose2D cellCentre(GridCell cell, double cellSize);

/// Returns the cell whose centre is nearest to a pose.
/// @param pose the pose in metres
/// @param cellSize metres per cell
[[nodiscard]] GridCell cellAt(Pose2D pose, double cellSize);

/// Advances a differential-drive pose by a time slice.
/// @param pose the pose now
/// @param linear forward speed in metres per second
/// @param angular turn rate in radians per second
/// @param dt the time slice in seconds
[[nodiscard]] Pose2D integrateDiffDrive(Pose2D pose, double linear, double angular, double dt);

/// Returns the shortest signed turn from the current heading to a target
/// heading, in (-pi, pi].
/// @param target the heading to reach
/// @param current the heading now
[[nodiscard]] double headingError(double target, double current);

/// Returns how far a pose has moved from one cell centre toward another,
/// clamped to 0..1.
/// @param pose the pose in metres
/// @param from the cell the robot left
/// @param to the cell the robot is entering
/// @param cellSize metres per cell
[[nodiscard]] double legProgress(Pose2D pose, GridCell from, GridCell to, double cellSize);

}  // namespace lotse::core
