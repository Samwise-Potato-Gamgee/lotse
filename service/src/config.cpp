// Implements the service configuration reader declared in config.hpp.

#include "lotse/service/config.hpp"

#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace lotse::service {
namespace {

/// Parses a true/false setting.
/// @param value the text to parse
/// @param key the setting name, used in the error message
[[nodiscard]] bool parseBool(const std::string& value, const std::string& key) {
    if (value == "true" || value == "yes" || value == "1") {
        return true;
    }
    if (value == "false" || value == "no" || value == "0") {
        return false;
    }
    throw std::runtime_error("config: " + key + " must be true or false");
}

/// Parses an integer setting.
/// @param value the text to parse
/// @param key the setting name, used in the error message
[[nodiscard]] int parseInt(const std::string& value, const std::string& key) {
    try {
        std::size_t consumed = 0;
        const int result = std::stoi(value, &consumed);
        if (consumed != value.size()) {
            throw std::runtime_error("trailing text");
        }
        return result;
    } catch (const std::exception&) {
        throw std::runtime_error("config: " + key + " must be an integer");
    }
}

/// Parses a floating point setting.
/// @param value the text to parse
/// @param key the setting name, used in the error message
[[nodiscard]] double parseDouble(const std::string& value, const std::string& key) {
    try {
        std::size_t consumed = 0;
        const double result = std::stod(value, &consumed);
        if (consumed != value.size()) {
            throw std::runtime_error("trailing text");
        }
        return result;
    } catch (const std::exception&) {
        throw std::runtime_error("config: " + key + " must be a number");
    }
}

/// Removes spaces, tabs and line ends from both ends of a text.
/// @param text the text to trim
[[nodiscard]] std::string trim(const std::string& text) {
    const std::size_t first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const std::size_t last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

/// Returns the value of a setting, or nothing when it is absent.
/// @param values the parsed settings
/// @param key the setting name
[[nodiscard]] std::optional<std::string> find(
    const std::unordered_map<std::string, std::string>& values, const std::string& key) {
    const auto found = values.find(key);
    if (found == values.end()) {
        return std::nullopt;
    }
    return found->second;
}

}  // namespace

ServiceConfig defaultConfig() {
    ServiceConfig config;
    config.mapPath = "maps/warehouse_large.map";
    return config;
}

ServiceConfig loadConfig(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("config: cannot open " + path);
    }

    std::unordered_map<std::string, std::string> values;
    static const std::unordered_map<std::string, bool> known = {{"map", true},
                                                                {"fleet_size", true},
                                                                {"planner", true},
                                                                {"bind_address", true},
                                                                {"port", true},
                                                                {"database", true},
                                                                {"dashboard", true},
                                                                {"cell_seconds", true},
                                                                {"tick_rate_hz", true},
                                                                {"sim_seconds_per_tick", true},
                                                                {"plan_time_limit_ms", true},
                                                                {"replan_window_seconds", true},
                                                                {"priority_ageing_seconds", true},
                                                                {"deadlock_patience_seconds", true},
                                                                {"background_planning", true},
                                                                {"pass_order", true},
                                                                {"slowdown_probability", true},
                                                                {"stop_probability", true},
                                                                {"sim_seed", true}};
    std::string line;
    int lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        const std::size_t comment = line.find('#');
        if (comment != std::string::npos) {
            line.erase(comment);
        }
        line = trim(line);
        if (line.empty()) {
            continue;
        }
        if (line.front() == '[') {
            continue;
        }
        const std::size_t separator = line.find('=');
        if (separator == std::string::npos) {
            throw std::runtime_error("config: line " + std::to_string(lineNumber) +
                                     " must be key = value");
        }
        const std::string key = trim(line.substr(0, separator));
        const std::string value = trim(line.substr(separator + 1));
        if (key.empty()) {
            throw std::runtime_error("config: line " + std::to_string(lineNumber) +
                                     " has no setting name");
        }
        if (value.empty()) {
            throw std::runtime_error("config: " + key + " has no value");
        }
        if (known.find(key) == known.end()) {
            throw std::runtime_error("config: unknown setting " + key);
        }
        values[key] = value;
    }

    ServiceConfig config = defaultConfig();
    if (const auto value = find(values, "map")) {
        config.mapPath = *value;
    }
    if (const auto value = find(values, "fleet_size")) {
        config.fleetSize = parseInt(*value, "fleet_size");
    }
    if (const auto value = find(values, "planner")) {
        config.planner = *value;
    }
    if (const auto value = find(values, "bind_address")) {
        config.bindAddress = *value;
    }
    if (const auto value = find(values, "port")) {
        config.port = parseInt(*value, "port");
    }
    if (const auto value = find(values, "database")) {
        config.databasePath = *value;
    }
    if (const auto value = find(values, "dashboard")) {
        config.dashboardDir = *value;
    }
    if (const auto value = find(values, "cell_seconds")) {
        config.cellSeconds = parseDouble(*value, "cell_seconds");
    }
    if (const auto value = find(values, "tick_rate_hz")) {
        config.tickRateHz = parseDouble(*value, "tick_rate_hz");
    }
    if (const auto value = find(values, "sim_seconds_per_tick")) {
        config.simSecondsPerTick = parseDouble(*value, "sim_seconds_per_tick");
    }
    if (const auto value = find(values, "plan_time_limit_ms")) {
        config.planTimeLimitMs = parseInt(*value, "plan_time_limit_ms");
    }
    if (const auto value = find(values, "replan_window_seconds")) {
        config.replanWindowSeconds = parseDouble(*value, "replan_window_seconds");
    }
    if (const auto value = find(values, "priority_ageing_seconds")) {
        config.priorityAgeingSeconds = parseDouble(*value, "priority_ageing_seconds");
    }
    if (const auto value = find(values, "deadlock_patience_seconds")) {
        config.deadlockPatienceSeconds = parseDouble(*value, "deadlock_patience_seconds");
    }
    if (const auto value = find(values, "background_planning")) {
        config.backgroundPlanning = parseBool(*value, "background_planning");
    }
    if (const auto value = find(values, "pass_order")) {
        config.passOrder = parseBool(*value, "pass_order");
    }
    if (const auto value = find(values, "slowdown_probability")) {
        config.slowdownProbability = parseDouble(*value, "slowdown_probability");
    }
    if (const auto value = find(values, "stop_probability")) {
        config.stopProbability = parseDouble(*value, "stop_probability");
    }
    if (const auto value = find(values, "sim_seed")) {
        config.simSeed = static_cast<std::uint32_t>(parseInt(*value, "sim_seed"));
    }
    return config;
}

}  // namespace lotse::service
