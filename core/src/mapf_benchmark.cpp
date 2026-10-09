// Implements the MAPF benchmark helpers declared in mapf_benchmark.hpp.

#include "lotse/benchmark/mapf_benchmark.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace lotse::benchmark {
namespace {

/// Splits a line on whitespace.
/// @param line the line to split
[[nodiscard]] std::vector<std::string> splitWhitespace(const std::string& line) {
    std::istringstream stream{line};
    std::vector<std::string> fields;
    std::string field;
    while (stream >> field) {
        fields.push_back(field);
    }
    return fields;
}

/// Formats a double with a fixed number of decimals.
/// @param value the value to format
/// @param precision the number of decimals
[[nodiscard]] std::string formatFixed(double value, int precision) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}

/// The summary key: planner, map and fleet size.
using SummaryKey = std::tuple<std::string, std::string, int>;

/// Totals for one summary key.
struct SummaryTotals {
    int runs = 0;
    int solved = 0;
    int timeouts = 0;
    long long cost = 0;
    std::vector<double> times;
};

}  // namespace

std::vector<core::Agent> readScen(const std::string& path, int robotCount) {
    std::ifstream input{path};
    if (!input.is_open()) {
        return {};
    }
    std::vector<core::Agent> agents;
    std::string line;
    while (std::getline(input, line) &&
           agents.size() < static_cast<std::size_t>(robotCount < 0 ? 0 : robotCount)) {
        const std::vector<std::string> fields = splitWhitespace(line);
        if (fields.size() != 9U) {
            continue;
        }
        const int startX = std::stoi(fields.at(4));
        const int startY = std::stoi(fields.at(5));
        const int goalX = std::stoi(fields.at(6));
        const int goalY = std::stoi(fields.at(7));
        agents.push_back(core::Agent{static_cast<int>(agents.size()),
                                     core::GridCell{startX, startY}, core::GridCell{goalX, goalY}});
    }
    return agents;
}

PlannerRun runPlanner(const core::Map& map, const std::vector<core::Agent>& agents,
                      core::Planner& planner) {
    PlannerRun run;
    const auto started = std::chrono::steady_clock::now();
    const std::optional<core::Plan> plan = planner.plan(map, agents);
    const auto finished = std::chrono::steady_clock::now();
    run.seconds = std::chrono::duration<double>(finished - started).count();
    run.solved = plan.has_value();
    run.timedOut = !run.solved && planner.lastCallTimedOut();
    if (plan.has_value()) {
        for (const core::Route& route : plan->routes) {
            run.cost += route.steps();
        }
    }
    return run;
}

std::string comparisonCsvHeader() {
    return "planner,map,robots,scenario,solved,timed_out,cost,seconds";
}

std::string toComparisonCsvRow(const ComparisonRow& row) {
    std::ostringstream out;
    out << row.planner << ',' << row.mapName << ',' << row.robotCount << ',' << row.scenario << ','
        << (row.solved ? 1 : 0) << ',' << (row.timedOut ? 1 : 0) << ',' << row.cost << ','
        << formatFixed(row.seconds, 4);
    return out.str();
}

std::string formatComparisonSummary(const std::vector<ComparisonRow>& rows) {
    std::map<SummaryKey, SummaryTotals> grouped;
    for (const ComparisonRow& row : rows) {
        SummaryTotals& totals = grouped[SummaryKey{row.planner, row.mapName, row.robotCount}];
        ++totals.runs;
        if (row.solved) {
            ++totals.solved;
            totals.cost += row.cost;
        } else if (row.timedOut) {
            ++totals.timeouts;
        }
        totals.times.push_back(row.seconds);
    }

    std::ostringstream out;
    out << "planner comparison (MAPF benchmark scenarios)\n";
    out << "  planner | map | robots | solved | timeouts | mean cost (solved) | mean s | p95 s\n";
    for (const auto& [key, totals] : grouped) {
        const auto& [planner, mapName, robots] = key;
        const double meanCost =
            totals.solved > 0 ? static_cast<double>(totals.cost) / totals.solved : 0.0;
        double sum = 0.0;
        for (const double seconds : totals.times) {
            sum += seconds;
        }
        const double meanSeconds =
            totals.times.empty() ? 0.0 : sum / static_cast<double>(totals.times.size());
        std::vector<double> sorted = totals.times;
        std::sort(sorted.begin(), sorted.end());
        const double p95Seconds =
            sorted.empty()
                ? 0.0
                : sorted[static_cast<std::size_t>(0.95 * static_cast<double>(sorted.size() - 1U))];
        out << "  " << planner << " | " << mapName << " | " << robots << " | " << totals.solved
            << "/" << totals.runs << " | " << totals.timeouts << " | " << std::fixed
            << std::setprecision(1) << meanCost << " | " << std::setprecision(3) << meanSeconds
            << " | " << p95Seconds << '\n';
    }
    return out.str();
}

}  // namespace lotse::benchmark
