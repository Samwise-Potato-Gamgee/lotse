// Implements the safety benchmark declared in safety_benchmark.hpp.

#include "lotse/benchmark/safety_benchmark.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <random>
#include <set>
#include <sstream>
#include <tuple>
#include <vector>

#include "lotse/core/executor.hpp"
#include "lotse/core/prioritized_planner.hpp"

namespace lotse::benchmark {
namespace {

/// Formats a delay probability with three fixed decimals.
/// @param value the probability to format
[[nodiscard]] std::string formatDelay(double value) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << value;
    return out.str();
}

/// Formats a share of a whole as a percentage with two decimals.
/// @param part the numerator
/// @param whole the denominator
[[nodiscard]] std::string formatShare(int part, int whole) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(2);
    if (whole <= 0) {
        out << 0.0;
    } else {
        out << (100.0 * static_cast<double>(part)) / static_cast<double>(whole);
    }
    return out.str();
}

}  // namespace

std::vector<Scenario> safetyScenarios(const std::vector<std::string>& mapNames,
                                      const std::vector<int>& fleetSizes,
                                      const std::vector<double>& delayRates,
                                      int seedsPerCombination) {
    std::vector<Scenario> scenarios;
    for (const std::string& mapName : mapNames) {
        for (const int fleetSize : fleetSizes) {
            for (const double delay : delayRates) {
                for (int seed = 0; seed < seedsPerCombination; ++seed) {
                    scenarios.push_back(
                        Scenario{mapName, fleetSize, delay, static_cast<std::uint32_t>(seed)});
                }
            }
        }
    }
    return scenarios;
}

int countBaseScenarios(const std::vector<Scenario>& scenarios) {
    std::set<std::tuple<std::string, int, std::uint32_t>> combinations;
    for (const Scenario& scenario : scenarios) {
        combinations.emplace(scenario.mapName, scenario.fleetSize, scenario.seed);
    }
    return static_cast<int>(combinations.size());
}

int countDelayLevels(const std::vector<Scenario>& scenarios) {
    std::set<double> delays;
    for (const Scenario& scenario : scenarios) {
        delays.insert(scenario.delayProbability);
    }
    return static_cast<int>(delays.size());
}

std::optional<std::vector<core::Agent>> makeFleet(const core::Map& map, int fleetSize,
                                                  std::uint32_t seed) {
    if (fleetSize <= 0) {
        return std::vector<core::Agent>{};
    }
    std::vector<core::GridCell> cells = map.freeCells();
    const auto count = static_cast<std::size_t>(fleetSize);
    if (cells.size() < 2U * count) {
        return std::nullopt;
    }

    std::mt19937 rng(seed);
    std::shuffle(cells.begin(), cells.end(), rng);

    std::vector<core::Agent> agents(count);
    for (std::size_t robot = 0; robot < count; ++robot) {
        agents[robot] = core::Agent{static_cast<int>(robot), cells[robot], cells[robot + count]};
    }
    return agents;
}

RunOutcome runScenario(const core::Map& map, const Scenario& scenario, bool passOrderEnabled) {
    RunOutcome outcome;
    outcome.scenario = scenario;
    outcome.passOrderEnabled = passOrderEnabled;

    const std::optional<std::vector<core::Agent>> fleet =
        makeFleet(map, scenario.fleetSize, scenario.seed);
    if (!fleet.has_value()) {
        outcome.initialPlanFailed = true;
        return outcome;
    }

    core::ExecutionConfig config;
    config.seed = scenario.seed;
    config.delayProbability = scenario.delayProbability;
    config.passOrderEnabled = passOrderEnabled;

    core::PrioritizedPlanner planner;
    const core::ExecutionResult result = core::execute(map, *fleet, planner, config);

    outcome.finished = result.finished;
    outcome.deadlock = result.deadlock;
    outcome.stepLimitHit = result.stepLimitHit;
    outcome.initialPlanFailed = result.initialPlanFailed;
    outcome.stepsUsed = result.stepsUsed;
    outcome.replans = result.replans;
    outcome.replanFailures = result.replanFailures;
    outcome.collisions = result.verdict.collisions;
    outcome.swaps = result.verdict.swaps;
    return outcome;
}

std::string csvHeader() {
    return "mode,map,fleet,delay,seed,finished,initial_plan_failed,deadlock,step_limit,steps,"
           "replans,replan_failures,collisions,swaps";
}

std::string toCsvRow(const RunOutcome& outcome) {
    std::ostringstream out;
    out << (outcome.passOrderEnabled ? "pass_order" : "control") << ',' << outcome.scenario.mapName
        << ',' << outcome.scenario.fleetSize << ','
        << formatDelay(outcome.scenario.delayProbability) << ',' << outcome.scenario.seed << ','
        << (outcome.finished ? 1 : 0) << ',' << (outcome.initialPlanFailed ? 1 : 0) << ','
        << (outcome.deadlock ? 1 : 0) << ',' << (outcome.stepLimitHit ? 1 : 0) << ','
        << outcome.stepsUsed << ',' << outcome.replans << ',' << outcome.replanFailures << ','
        << outcome.collisions << ',' << outcome.swaps;
    return out.str();
}

void accumulate(Totals& totals, const RunOutcome& outcome) {
    ++totals.runs;
    if (outcome.initialPlanFailed) {
        ++totals.initialPlanFailures;
    } else {
        ++totals.executedRuns;
    }
    if (outcome.finished) {
        ++totals.finishedRuns;
    }
    if (outcome.collisions > 0) {
        ++totals.runsWithCollisions;
    }
    if (outcome.swaps > 0) {
        ++totals.runsWithSwaps;
    }
    if (outcome.collisions > 0 || outcome.swaps > 0) {
        ++totals.runsWithViolations;
    }
    totals.collisionEvents += outcome.collisions;
    totals.swapEvents += outcome.swaps;
    if (outcome.deadlock) {
        ++totals.deadlocks;
    }
    if (outcome.stepLimitHit) {
        ++totals.stepLimitHits;
    }
    totals.replans += outcome.replans;
    totals.replanFailures += outcome.replanFailures;
}

std::vector<DelayTotals> groupByDelay(const std::vector<RunOutcome>& outcomes) {
    constexpr double kDelayTolerance = 1e-9;

    std::vector<const RunOutcome*> sorted;
    sorted.reserve(outcomes.size());
    for (const RunOutcome& outcome : outcomes) {
        sorted.push_back(&outcome);
    }
    std::sort(sorted.begin(), sorted.end(), [](const RunOutcome* lhs, const RunOutcome* rhs) {
        return lhs->scenario.delayProbability < rhs->scenario.delayProbability;
    });

    std::vector<DelayTotals> rows;
    for (const RunOutcome* outcome : sorted) {
        const double delay = outcome->scenario.delayProbability;
        if (rows.empty() || std::abs(delay - rows.back().delay) > kDelayTolerance) {
            DelayTotals row;
            row.delay = delay;
            rows.push_back(row);
        }
        accumulate(rows.back().totals, *outcome);
    }
    return rows;
}

std::string formatSummary(const std::string& title, const Totals& totals, int scenarioCount,
                          int delayLevelCount) {
    std::ostringstream out;
    out << title << '\n';
    out << "  " << totals.runs << " runs = " << scenarioCount << " scenarios x " << delayLevelCount
        << " delay levels\n";
    out << "  executed runs:       " << totals.executedRuns << '\n';
    out << "  finished:            " << totals.finishedRuns << '\n';
    out << "  runs w/ collisions:  " << totals.runsWithCollisions << '\n';
    out << "  collision events:    " << totals.collisionEvents << '\n';
    out << "  runs w/ swaps:       " << totals.runsWithSwaps << '\n';
    out << "  swap events:         " << totals.swapEvents << '\n';
    out << "  executed runs w/ a collision or swap: " << totals.runsWithViolations << " ("
        << formatShare(totals.runsWithViolations, totals.executedRuns) << " %)\n";
    out << "  deadlocks:           " << totals.deadlocks << '\n';
    out << "  step-limit hits:     " << totals.stepLimitHits << '\n';
    out << "  initial plan fails:  " << totals.initialPlanFailures << '\n';
    out << "  replans:             " << totals.replans << '\n';
    out << "  replan failures:     " << totals.replanFailures << '\n';
    return out.str();
}

std::string formatDelayTable(const std::vector<DelayTotals>& passRows,
                             const std::vector<DelayTotals>& controlRows) {
    std::ostringstream out;
    out << "delay-level breakdown (bad = executed runs with at least one collision or swap)\n";
    out << "  delay | pass:exec pass:bad pass:coll pass:swap | ctrl:exec ctrl:bad ctrl:coll "
           "ctrl:swap\n";
    const std::size_t count = std::min(passRows.size(), controlRows.size());
    for (std::size_t index = 0; index < count; ++index) {
        const Totals& pass = passRows[index].totals;
        const Totals& control = controlRows[index].totals;
        out << "  " << std::fixed << std::setprecision(3) << passRows[index].delay << " | "
            << std::setw(9) << pass.executedRuns << " " << std::setw(8) << pass.runsWithViolations
            << " " << std::setw(9) << pass.collisionEvents << " " << std::setw(9) << pass.swapEvents
            << " | " << std::setw(9) << control.executedRuns << " " << std::setw(8)
            << control.runsWithViolations << " " << std::setw(9) << control.collisionEvents << " "
            << std::setw(9) << control.swapEvents << '\n';
    }
    return out.str();
}

}  // namespace lotse::benchmark
