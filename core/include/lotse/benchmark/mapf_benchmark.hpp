// Runs the public MAPF benchmark instances through the planners: reads .scen
// start and goal pairs and measures solved share, total route length and time.

#pragma once

#include <string>
#include <vector>

#include "lotse/core/map.hpp"
#include "lotse/core/plan.hpp"
#include "lotse/core/planner.hpp"

namespace lotse::benchmark {

/// One measured planner call.
struct PlannerRun {
    bool solved = false;
    bool timedOut = false;
    int cost = 0;
    double seconds = 0.0;
};

/// One row of the comparison table.
struct ComparisonRow {
    std::string mapName;
    int robotCount = 0;
    int scenario = 0;
    std::string planner;
    bool solved = false;
    bool timedOut = false;
    int cost = 0;
    double seconds = 0.0;
};

/// Reads the first start and goal pairs of a MAPF .scen file.
/// @param path the scenario file
/// @param robotCount how many pairs to read
[[nodiscard]] std::vector<core::Agent> readScen(const std::string& path, int robotCount);

/// Runs one planner on one instance and measures the call.
/// @param map the warehouse grid
/// @param agents the fleet
/// @param planner the planner to time
[[nodiscard]] PlannerRun runPlanner(const core::Map& map, const std::vector<core::Agent>& agents,
                                    core::Planner& planner);

/// Returns the header line of the raw comparison CSV.
[[nodiscard]] std::string comparisonCsvHeader();

/// Formats one comparison row as a CSV line.
/// @param row the row to format
[[nodiscard]] std::string toComparisonCsvRow(const ComparisonRow& row);

/// Formats the per-planner, per-map, per-fleet-size summary.
/// @param rows every measured planner call
[[nodiscard]] std::string formatComparisonSummary(const std::vector<ComparisonRow>& rows);

}  // namespace lotse::benchmark
