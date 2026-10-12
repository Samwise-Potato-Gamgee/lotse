// Prioritized planning: fix one robot's route at a time, hardest robot first,
// each avoiding the routes already fixed.

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "lotse/core/planner.hpp"

namespace lotse::core {

/// The order in which robots are given their routes.
enum class AgentOrder : std::uint8_t { AsGiven, FarthestFirst, NearestFirst };

/// Tuning for prioritized planning.
struct PrioritizedOptions {
    AgentOrder order = AgentOrder::FarthestFirst;
    int horizonSlack = 0;
    int maxHorizon = 0;
    int attempts = 8;
    int timeLimitMs = 0;
    bool oneStepGap = false;
    bool stableOrder = false;
};

/// The longest single-leg route a continuous run is expected to need. Limits
/// the search horizon so repeated continuous planning stays fast.
/// @param map the warehouse grid
/// @param fleetSize the number of robots
[[nodiscard]] int continuousHorizon(const Map& map, int fleetSize);

/// Plans the fleet by planning one robot at a time in priority order.
class PrioritizedPlanner final : public Planner {
public:
    /// Creates a planner with the given options.
    /// @param options agent order and horizon slack
    explicit PrioritizedPlanner(PrioritizedOptions options = {});

    /// Plans every agent in priority order, one after another.
    /// @param map the warehouse grid
    /// @param agents the fleet, one start and one goal per robot
    /// @return a conflict-free plan, or nothing when a robot cannot be routed
    [[nodiscard]] std::optional<Plan> plan(const Map& map,
                                           const std::vector<Agent>& agents) override;

    /// Returns the planner's name.
    [[nodiscard]] std::string name() const override;

    /// Returns true when the last plan call stopped on its time limit.
    [[nodiscard]] bool lastCallTimedOut() const noexcept override;

    /// Returns the robot ids in the order the last successful plan call used.
    [[nodiscard]] std::vector<int> lastPlanningOrder() const override;

    /// Returns the first attempt number that succeeded in the last call.
    [[nodiscard]] int lastPlanningAttempt() const noexcept override;

    /// Computes the time horizon used for one planning call.
    /// @param map the warehouse grid
    /// @param agents the fleet being planned
    [[nodiscard]] int horizonFor(const Map& map, const std::vector<Agent>& agents) const;

private:
    /// Builds the first-attempt order: robots whose goal is unchanged keep
    /// their previous place, and the rest are added farthest first.
    /// @param agents the fleet being planned
    [[nodiscard]] std::vector<std::size_t> stableOrder(const std::vector<Agent>& agents) const;

    PrioritizedOptions options_{};
    bool lastTimedOut_ = false;
    std::vector<int> lastOrder_;
    int lastAttempt_ = -1;
    std::vector<std::pair<int, GridCell>> stable_;
};

}  // namespace lotse::core
