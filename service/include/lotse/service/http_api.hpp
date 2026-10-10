// Serves the HTTP JSON API, the WebSocket live feed and the dashboard files from
// the C++ service. It is a thin adapter over the runtime, which owns the state.

#pragma once

#include <memory>
#include <string>

#include "lotse/service/runtime.hpp"

namespace lotse::service {

/// The HTTP and WebSocket front end of the service.
class ApiServer {
public:
    /// Prepares the routes and the dashboard mount.
    /// @param runtime the state the handlers read and change
    /// @param bindAddress the interface to listen on, 127.0.0.1 by default
    /// @param port the TCP port, 0 to let the operating system choose one
    /// @param dashboardDir the directory with index.html, app.js and style.css
    ApiServer(Runtime& runtime, std::string bindAddress, int port, std::string dashboardDir);
    ~ApiServer();

    ApiServer(const ApiServer&) = delete;
    ApiServer& operator=(const ApiServer&) = delete;
    ApiServer(ApiServer&&) = delete;
    ApiServer& operator=(ApiServer&&) = delete;

    /// Binds and serves, blocking until stop() is called.
    [[nodiscard]] bool listen();

    /// Stops the server and disconnects the WebSocket clients.
    void stop();

    /// Returns the port the server is bound to, or the configured port before
    /// listen() has run.
    [[nodiscard]] int port() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace lotse::service
