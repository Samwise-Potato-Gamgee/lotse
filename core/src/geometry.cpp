// Implements the grid geometry helpers declared in geometry.hpp.

#include "lotse/core/geometry.hpp"

#include <cmath>
#include <numbers>

namespace lotse::core {
namespace {

/// Keeps an angle inside (-pi, pi].
/// @param angle the angle in radians
[[nodiscard]] double normalise(double angle) {
    while (angle > std::numbers::pi) {
        angle -= 2.0 * std::numbers::pi;
    }
    while (angle <= -std::numbers::pi) {
        angle += 2.0 * std::numbers::pi;
    }
    return angle;
}

}  // namespace

Pose2D cellCentre(GridCell cell, double cellSize) {
    return Pose2D{(static_cast<double>(cell.x) + 0.5) * cellSize,
                  (static_cast<double>(cell.y) + 0.5) * cellSize, 0.0};
}

GridCell cellAt(Pose2D pose, double cellSize) {
    return GridCell{static_cast<int>(std::lround(pose.x / cellSize - 0.5)),
                    static_cast<int>(std::lround(pose.y / cellSize - 0.5))};
}

Pose2D integrateDiffDrive(Pose2D pose, double linear, double angular, double dt) {
    pose.theta = normalise(pose.theta + angular * dt);
    pose.x += linear * std::cos(pose.theta) * dt;
    pose.y += linear * std::sin(pose.theta) * dt;
    return pose;
}

double headingError(double target, double current) {
    return normalise(target - current);
}

double legProgress(Pose2D pose, GridCell from, GridCell to, double cellSize) {
    const Pose2D start = cellCentre(from, cellSize);
    const Pose2D end = cellCentre(to, cellSize);
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double lengthSquared = dx * dx + dy * dy;
    if (lengthSquared <= 0.0) {
        return 0.0;
    }
    const double progress = ((pose.x - start.x) * dx + (pose.y - start.y) * dy) / lengthSquared;
    if (progress < 0.0) {
        return 0.0;
    }
    if (progress > 1.0) {
        return 1.0;
    }
    return progress;
}

}  // namespace lotse::core
