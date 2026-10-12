// The service configuration: one key=value file with # comments, covering the
// map, the fleet, the simulator rates, the planner, the API and the database.

#pragma once

#include <cstdint>
#include <string>

namespace lotse::service {

/// Everything the service needs to start.
struct ServiceConfig {
    std::string mapPath;
    int fleetSize = 5;
    std::string planner = "prioritized";
    std::string bindAddress = "127.0.0.1";
    int port = 8080;
    std::string databasePath = "lotse.db";
    std::string dashboardDir = "service/dashboard";
    double cellSeconds = 1.0;
    double tickRateHz = 10.0;
    double simSecondsPerTick = 1.0;
    int planTimeLimitMs = 500;
    double replanWindowSeconds = 8.0;
    double priorityAgeingSeconds = 5.0;
    double deadlockPatienceSeconds = 40.0;
    bool backgroundPlanning = false;
    bool stableOrder = false;
    bool passOrder = true;
    double slowdownProbability = 0.0;
    double stopProbability = 0.0;
    std::uint32_t simSeed = 1;
};

/// Returns the built-in defaults, used when a setting is absent.
[[nodiscard]] ServiceConfig defaultConfig();

/// Loads a configuration file.
/// @param path the key=value file to read
/// @return the parsed configuration; throws std::runtime_error on a bad line or
///         an unknown key
[[nodiscard]] ServiceConfig loadConfig(const std::string& path);

}  // namespace lotse::service
