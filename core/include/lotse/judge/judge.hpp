// The independent safety judge: reads a trajectory and reports cells shared by
// two robots and robots that swap cells. It shares no logic with the planner or
// the executor, so a bug there cannot hide a collision from the judge.

#pragma once

#include <cstdint>
#include <vector>

#include "lotse/core/grid.hpp"

namespace lotse::judge {

/// One simulation frame: the position of every robot, indexed by robot.
using Frame = std::vector<lotse::core::GridCell>;

/// The whole trajectory of a run; frame t is the state after step t.
struct RunLog {
    int agentCount = 0;
    std::vector<Frame> frames;
};

/// The kinds of safety violation the judge looks for.
enum class ViolationKind : std::uint8_t { Collision, Swap };

/// One violation, with the step and the two robots involved.
struct Violation {
    ViolationKind kind = ViolationKind::Collision;
    int step = 0;
    int robotA = 0;
    int robotB = 0;
};

/// The judge's verdict for one run.
struct Report {
    int collisions = 0;
    int swaps = 0;
    std::vector<Violation> violations;

    /// Returns true when the run had no violation at all.
    [[nodiscard]] bool safe() const noexcept { return violations.empty(); }
};

/// Counts collisions and swaps in a trajectory.
/// @param log the trajectory, frame 0 first
/// @return the verdict, including one entry per violation found
[[nodiscard]] Report analyse(const RunLog& log);

}  // namespace lotse::judge
