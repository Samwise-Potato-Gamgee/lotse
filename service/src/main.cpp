// Starts the Lotse service: reads the configuration, opens the SQLite store,
// builds the runtime with the grid simulator and the fleet manager, and serves
// the HTTP API, the WebSocket feed and the dashboard until interrupted.

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>

#include "lotse/core/clock.hpp"
#include "lotse/service/config.hpp"
#include "lotse/service/http_api.hpp"
#include "lotse/service/runtime.hpp"
#include "lotse/service/sqlite_order_store.hpp"

namespace {

/// The command line options of the service.
struct Options {
    std::string configPath = "config/lotse.conf";
    std::string bindAddress;
    int port = -1;
};

std::atomic<lotse::service::ApiServer*> g_api{nullptr};

/// Asks the server to stop on Ctrl-C or SIGTERM.
/// @param signal the signal number, unused
void handleSignal(int signal) {
    (void)signal;
    if (lotse::service::ApiServer* server = g_api.load()) {
        server->stop();
    }
}

/// Reads the --config, --bind and --port options.
/// @param argc argument count
/// @param argv argument vector
[[nodiscard]] Options parseOptions(int argc, char** argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string flag = argv[index];
        if (index + 1 >= argc) {
            throw std::runtime_error("missing value for " + flag);
        }
        if (flag == "--config") {
            options.configPath = argv[++index];
        } else if (flag == "--bind") {
            options.bindAddress = argv[++index];
        } else if (flag == "--port") {
            options.port = std::stoi(argv[++index]);
        } else {
            throw std::runtime_error("unknown option " + flag);
        }
    }
    return options;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parseOptions(argc, argv);
        lotse::service::ServiceConfig config = lotse::service::loadConfig(options.configPath);
        if (!options.bindAddress.empty()) {
            config.bindAddress = options.bindAddress;
        }
        if (options.port >= 0) {
            config.port = options.port;
        }

        lotse::core::SteadyClock clock;
        lotse::service::SqliteOrderStore store(config.databasePath);
        lotse::service::Runtime runtime(config, store, clock);
        lotse::service::ApiServer api(runtime, config.bindAddress, config.port,
                                      config.dashboardDir);
        g_api = &api;
        std::signal(SIGINT, handleSignal);
        std::signal(SIGTERM, handleSignal);

        runtime.start();
        std::printf("lotse service: map %s, %d robots, planner %s\n", config.mapPath.c_str(),
                    config.fleetSize, config.planner.c_str());
        std::printf("lotse dashboard: http://%s:%d/\n", config.bindAddress.c_str(), config.port);
        std::fflush(stdout);

        const bool served = api.listen();
        runtime.stop();
        g_api = nullptr;
        if (!served) {
            std::fprintf(stderr, "lotse service: failed to listen on %s:%d\n",
                         config.bindAddress.c_str(), config.port);
            return EXIT_FAILURE;
        }
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "lotse service failed: %s\n", error.what());
        return EXIT_FAILURE;
    }
}
