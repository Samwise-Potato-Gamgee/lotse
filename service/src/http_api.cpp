// Implements the HTTP and WebSocket front end declared in http_api.hpp.

#include "lotse/service/http_api.hpp"

#include <atomic>
#include <cstddef>
#include <mutex>
#include <set>
#include <string>
#include <utility>

#include <httplib.h>
#include <nlohmann/json.hpp>

namespace lotse::service {
namespace {

using nlohmann::json;

/// Writes a JSON body with a status code.
/// @param response the response
/// @param status the HTTP status code
/// @param body the JSON text
void sendJson(httplib::Response& response, int status, const std::string& body) {
    response.status = status;
    response.set_header("Access-Control-Allow-Origin", "*");
    response.set_content(body, "application/json; charset=utf-8");
}

/// Writes a JSON error with a status code.
/// @param response the response
/// @param status the HTTP status code
/// @param message the reason shown to the caller
void sendError(httplib::Response& response, int status, const std::string& message) {
    json document;
    document["error"] = message;
    sendJson(response, status, document.dump());
}

/// Parses an integer path parameter.
/// @param text the parameter text
/// @return the parsed integer, or nothing when it is not a number
[[nodiscard]] std::optional<int> parseId(const std::string& text) {
    try {
        std::size_t consumed = 0;
        const int id = std::stoi(text, &consumed);
        if (consumed != text.size()) {
            return std::nullopt;
        }
        return id;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

}  // namespace

struct ApiServer::Impl {
    FleetApiSource& source;
    std::string bindAddress;
    int configuredPort;
    std::string dashboardDir;

    httplib::Server server;
    std::atomic<int> boundPort{0};
    std::atomic<bool> stopping{false};

    std::mutex clientsMutex;
    std::set<httplib::ws::WebSocket*> clients;

    Impl(FleetApiSource& sourceRef, std::string address, int port, std::string dashboard)
        : source(sourceRef),
          bindAddress(std::move(address)),
          configuredPort(port),
          dashboardDir(std::move(dashboard)) {
        if (!dashboardDir.empty()) {
            server.set_mount_point("/", dashboardDir);
        }
        registerRoutes();
        source.setUpdateSink([this](const std::string& message) { broadcast(message); });
    }

    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(Impl&&) = delete;

    ~Impl() { source.setUpdateSink(nullptr); }

    /// Sends one message to every connected dashboard.
    /// @param message the text to send
    void broadcast(const std::string& message) {
        const std::lock_guard<std::mutex> lock(clientsMutex);
        for (httplib::ws::WebSocket* client : clients) {
            if (client->is_open()) {
                client->send(message);
            }
        }
    }

    /// Adds the HTTP, WebSocket and health routes.
    void registerRoutes() {
        server.Get("/health", [this](const httplib::Request&, httplib::Response& response) {
            sendJson(response, 200, source.healthJson());
        });
        server.Get("/map", [this](const httplib::Request&, httplib::Response& response) {
            sendJson(response, 200, source.mapJson());
        });
        server.Get("/fleet", [this](const httplib::Request&, httplib::Response& response) {
            sendJson(response, 200, source.fleetJson());
        });
        server.Get("/throughput", [this](const httplib::Request&, httplib::Response& response) {
            sendJson(response, 200, source.throughputJson());
        });
        server.Get("/orders", [this](const httplib::Request&, httplib::Response& response) {
            sendJson(response, 200, source.ordersJson());
        });
        server.Get(R"(/orders/(\d+))",
                   [this](const httplib::Request& request, httplib::Response& response) {
                       const std::string idText = request.matches[1].str();
                       const std::optional<int> id = parseId(idText);
                       if (!id.has_value()) {
                           sendError(response, 400, "order id must be a number");
                           return;
                       }
                       const std::optional<std::string> document = source.orderJson(id.value());
                       if (!document.has_value()) {
                           sendError(response, 404, "unknown order '" + idText + "'");
                           return;
                       }
                       sendJson(response, 200, document.value());
                   });
        server.Post("/orders", [this](const httplib::Request& request,
                                      httplib::Response& response) {
            json body;
            try {
                body = json::parse(request.body);
            } catch (const json::parse_error&) {
                sendError(response, 400, "request body must be JSON");
                return;
            }
            if (!body.is_object() || !body.contains("pickup") || !body.contains("dropoff")) {
                sendError(response, 400, "pickup and dropoff are required");
                return;
            }
            try {
                const int priority = body.value("priority", 0);
                const int id = source.submitOrder(body.at("pickup").get<std::string>(),
                                                  body.at("dropoff").get<std::string>(), priority);
                json document;
                document["id"] = id;
                document["state"] = "pending";
                sendJson(response, 201, document.dump());
            } catch (const OrderRejected& rejection) {
                sendError(response, 400, rejection.what());
            } catch (const json::exception&) {
                sendError(response, 400, "pickup and dropoff must be strings");
            }
        });
        server.Delete(R"(/orders/(\d+))", [this](const httplib::Request& request,
                                                 httplib::Response& response) {
            const std::string idText = request.matches[1].str();
            const std::optional<int> id = parseId(idText);
            if (!id.has_value()) {
                sendError(response, 400, "order id must be a number");
                return;
            }
            switch (source.cancelOrder(id.value())) {
                case CancelOutcome::Cancelled: {
                    json document;
                    document["id"] = id.value();
                    document["state"] = "cancelled";
                    sendJson(response, 200, document.dump());
                    return;
                }
                case CancelOutcome::InProgress:
                    sendError(response, 409, "order already picked up and will be delivered");
                    return;
                case CancelOutcome::Finished:
                    sendError(response, 409, "order is already finished");
                    return;
                case CancelOutcome::NotFound:
                    sendError(response, 404, "unknown order '" + idText + "'");
                    return;
            }
        });
        server.WebSocket("/ws", [this](const httplib::Request&, httplib::ws::WebSocket& socket) {
            {
                const std::lock_guard<std::mutex> lock(clientsMutex);
                clients.insert(&socket);
            }
            socket.send(source.fleetJson());
            std::string message;
            while (!stopping.load() && socket.read(message) != httplib::ws::Fail) {
            }
            const std::lock_guard<std::mutex> lock(clientsMutex);
            clients.erase(&socket);
        });
        server.set_exception_handler([](const httplib::Request&, httplib::Response& response,
                                        const std::exception_ptr& error) {
            try {
                if (error) {
                    std::rethrow_exception(error);
                }
            } catch (const std::exception& caught) {
                sendError(response, 500, caught.what());
            }
        });
    }
};

ApiServer::ApiServer(FleetApiSource& source, std::string bindAddress, int port,
                     std::string dashboardDir)
    : impl_(std::make_unique<Impl>(source, std::move(bindAddress), port, std::move(dashboardDir))) {
}

ApiServer::~ApiServer() {
    stop();
}

bool ApiServer::listen() {
    impl_->stopping = false;
    if (impl_->configuredPort == 0) {
        const int port = impl_->server.bind_to_any_port(impl_->bindAddress);
        if (port <= 0) {
            return false;
        }
        impl_->boundPort = port;
        return impl_->server.listen_after_bind();
    }
    impl_->boundPort = impl_->configuredPort;
    return impl_->server.listen(impl_->bindAddress, impl_->configuredPort);
}

void ApiServer::stop() {
    impl_->stopping = true;
    impl_->server.stop();
    const std::lock_guard<std::mutex> lock(impl_->clientsMutex);
    for (httplib::ws::WebSocket* client : impl_->clients) {
        client->close();
    }
    impl_->clients.clear();
}

void ApiServer::publishFleet() {
    impl_->broadcast(impl_->source.fleetJson());
}

int ApiServer::port() const {
    return impl_->boundPort.load();
}

}  // namespace lotse::service
