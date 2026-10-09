// Conflict-based search: the second planner behind the Planner interface. It
// searches the space of constraint sets to find routes that are optimal for the
// sum of route lengths, within a per-call time limit.

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "lotse/core/planner.hpp"

namespace lotse::core {

/// Tuning for conflict-based search.
struct CbsOptions {
    int timeLimitMs = 5000;
    int horizonSlack = 0;
    int maxHorizon = 0;
    int maxNodes = 200000;
};

/// Plans the fleet with conflict-based search, optimal for the sum of costs.
class CbsPlanner final : public Planner {
public:
    /// Creates a planner with the given options.
    /// @param options time limit, horizon and node cap
    explicit CbsPlanner(CbsOptions options = {});

    /// Plans every agent with conflict-based search.
    /// @param map the warehouse grid
    /// @param agents the fleet, one start and one goal per robot
    /// @return an optimal conflict-free plan, or nothing on failure or timeout
    [[nodiscard]] std::optional<Plan> plan(const Map& map,
                                           const std::vector<Agent>& agents) override;

    /// Returns the planner's name.
    [[nodiscard]] std::string name() const override;

    /// Returns true when the last call stopped on its time limit.
    [[nodiscard]] bool lastCallTimedOut() const noexcept override;

    /// Computes the time horizon used for one planning call.
    /// @param map the warehouse grid
    /// @param agents the fleet being planned
    [[nodiscard]] int horizonFor(const Map& map, const std::vector<Agent>& agents) const;

private:
    CbsOptions options_{};
    bool lastTimedOut_ = false;
};

}  // namespace lotse::core
