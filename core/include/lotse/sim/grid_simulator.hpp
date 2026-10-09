// The grid simulator: executes a plan step by step and hands the trajectory to
// the judge. In stage 1 the execution is exactly the plan.

#pragma once

#include "lotse/core/plan.hpp"
#include "lotse/judge/judge.hpp"

namespace lotse::sim {

/// Transcribes a plan into a trajectory, one plan step per frame.
/// @param plan the routes to follow
/// @param steps how many steps to simulate
/// @return the trajectory, frame 0 being the start state
[[nodiscard]] judge::RunLog followPlanExactly(const core::Plan& plan, int steps);

}  // namespace lotse::sim
