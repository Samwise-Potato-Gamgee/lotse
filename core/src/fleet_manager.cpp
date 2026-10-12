// Implements the real-time fleet manager declared in fleet_manager.hpp.

#include "lotse/core/fleet_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

#include "lotse/core/pass_order.hpp"

namespace lotse::core {

const char* toString(OrderState state) noexcept {
    switch (state) {
        case OrderState::Pending:
            return "pending";
        case OrderState::Assigned:
            return "assigned";
        case OrderState::InProgress:
            return "in_progress";
        case OrderState::Completed:
            return "completed";
        case OrderState::Cancelled:
            return "cancelled";
        case OrderState::Failed:
            return "failed";
    }
    return "unknown";
}

std::optional<OrderState> orderStateFromText(std::string_view text) noexcept {
    if (text == "pending") {
        return OrderState::Pending;
    }
    if (text == "assigned") {
        return OrderState::Assigned;
    }
    if (text == "in_progress") {
        return OrderState::InProgress;
    }
    if (text == "completed") {
        return OrderState::Completed;
    }
    if (text == "cancelled") {
        return OrderState::Cancelled;
    }
    if (text == "failed") {
        return OrderState::Failed;
    }
    return std::nullopt;
}

namespace {

/// What a robot is heading for right now.
enum class Goal : std::uint8_t { Parked, Parking, Pickup, Dropoff };

/// A planning job handed to the worker thread.
struct PlanRequest {
    Map map{};
    std::vector<Agent> agents;
    std::vector<int> robotIds;
};

}  // namespace

struct FleetManager::Impl {
    const Map& map;
    RobotSide& side;
    Planner& planner;
    Planner* fallback;
    TaskAssigner& assigner;
    Clock& clock;
    FleetManagerConfig config;

    mutable std::mutex mutex;
    std::vector<ManagedOrder> orders;
    int nextOrderId = 1;
    std::vector<OrderEvent> pendingEvents;

    int fleetSize = 0;
    std::vector<Route> routes;
    std::vector<int> index;
    std::vector<bool> moving;
    std::vector<int> targetIndex;
    std::vector<Goal> goals;
    std::vector<int> robotOrder;
    std::vector<int> robotCompleted;
    std::vector<RobotStatus> lastStatus;
    CellSchedule schedule;

    judge::Frame previousFrame;
    judge::Report verdict;

    int step = 0;
    int stepsSincePlan = 0;
    int lastPlanStep = -1000000;
    bool needsReplan = true;
    int stagnantSteps = 0;
    bool deadlock = false;
    int lastProgressStep = 0;
    int lastProgressOrders = 0;
    int lowestGoalDistance = 0;
    bool hasProgressBaseline = false;
    bool noProgress = false;
    bool progressAlarm = false;
    double secondsWithoutProgress = 0.0;
    bool backgroundPlanning = true;
    double tickSeconds = 1.0;

    std::chrono::steady_clock::time_point startedAt;
    bool started = false;
    ThroughputStats stats;

    bool diagnosticsEnabled = false;
    ManagerDiagnostics diag;
    std::unordered_map<GridCell, std::vector<std::pair<int, int>>, GridCellHash> lastPlanCellOrder;
    bool freshPlanPending = false;
    long long freshPlanFailureStep = 0;

    std::thread worker;
    mutable std::mutex workMutex;
    std::condition_variable workCv;
    bool stopping = false;
    bool busy = false;
    bool requestWaiting = false;
    std::optional<PlanRequest> request;
    bool resultReady = false;
    std::optional<Plan> result;
    double lastPlanMs = 0.0;

    Impl(const Map& warehouse, RobotSide& robotSide, Planner& fleetPlanner,
         TaskAssigner& taskAssigner, Clock& timeSource, FleetManagerConfig managerConfig,
         Planner* fallbackPlanner)
        : map(warehouse),
          side(robotSide),
          planner(fleetPlanner),
          fallback(fallbackPlanner),
          assigner(taskAssigner),
          clock(timeSource),
          config(managerConfig),
          fleetSize(robotSide.robotCount()),
          backgroundPlanning(managerConfig.backgroundPlanning),
          diagnosticsEnabled(managerConfig.diagnostics) {
        const std::vector<GridCell> starts = side.starts();
        if (static_cast<int>(starts.size()) != fleetSize) {
            throw std::runtime_error("fleet manager: starts do not match the fleet size");
        }
        if (diagnosticsEnabled) {
            diag.cellOccupiedWaitSeconds.assign(static_cast<std::size_t>(fleetSize), 0.0);
            diag.passOrderWaitSeconds.assign(static_cast<std::size_t>(fleetSize), 0.0);
            diag.noWaypointSeconds.assign(static_cast<std::size_t>(fleetSize), 0.0);
        }
        routes.resize(static_cast<std::size_t>(fleetSize));
        index.assign(static_cast<std::size_t>(fleetSize), 0);
        moving.assign(static_cast<std::size_t>(fleetSize), false);
        targetIndex.assign(static_cast<std::size_t>(fleetSize), 0);
        goals.assign(static_cast<std::size_t>(fleetSize), Goal::Parked);
        robotOrder.assign(static_cast<std::size_t>(fleetSize), -1);
        robotCompleted.assign(static_cast<std::size_t>(fleetSize), 0);
        for (int robot = 0; robot < fleetSize; ++robot) {
            const auto slot = static_cast<std::size_t>(robot);
            routes[slot] = Route{robot, {starts[slot]}};
            lastStatus.push_back(RobotStatus{robot, starts[slot], starts[slot], 0.0, false});
        }
        previousFrame.reserve(static_cast<std::size_t>(fleetSize));
        for (const RobotStatus& status : lastStatus) {
            previousFrame.push_back(status.cell);
        }
        schedule = buildCellSchedule(routes);
        if (backgroundPlanning) {
            worker = std::thread([this] { workerLoop(); });
        }
    }

    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;
    Impl(Impl&&) = delete;
    Impl& operator=(Impl&&) = delete;

    ~Impl() {
        {
            const std::lock_guard<std::mutex> lock(workMutex);
            stopping = true;
        }
        workCv.notify_all();
        if (worker.joinable()) {
            worker.join();
        }
    }

    /// Runs the background planner until the manager is destroyed.
    void workerLoop() {
        while (true) {
            PlanRequest job;
            {
                std::unique_lock<std::mutex> lock(workMutex);
                workCv.wait(lock, [this] { return stopping || requestWaiting; });
                if (stopping && !requestWaiting) {
                    return;
                }
                if (request.has_value()) {
                    job = std::move(request.value());
                }
                request.reset();
                requestWaiting = false;
                busy = true;
            }
            const auto planStart = std::chrono::steady_clock::now();
            std::optional<Plan> planned = planner.plan(job.map, job.agents);
            if (!planned.has_value() && fallback != nullptr) {
                planned = fallback->plan(job.map, job.agents);
            }
            const double planMs = std::chrono::duration<double, std::milli>(
                                      std::chrono::steady_clock::now() - planStart)
                                      .count();
            {
                const std::lock_guard<std::mutex> lock(workMutex);
                result = std::move(planned);
                lastPlanMs = planMs;
                resultReady = true;
                busy = false;
            }
        }
    }

    /// Returns the cell a robot is heading for.
    /// @param robot the robot id
    [[nodiscard]] GridCell goalCell(int robot) const {
        const auto slot = static_cast<std::size_t>(robot);
        if (robotOrder[slot] < 0) {
            const std::vector<GridCell>& parking = map.parkingCells();
            if (parking.empty()) {
                return routes[slot].cells.front();
            }
            return parking[static_cast<std::size_t>(robot) % parking.size()];
        }
        const ManagedOrder& order = findOrder(robotOrder[slot]);
        return goals[slot] == Goal::Dropoff ? order.order.dropoff : order.order.pickup;
    }

    /// Looks up an order by id, throwing when it is unknown.
    /// @param id the order id
    [[nodiscard]] const ManagedOrder& findOrder(int id) const {
        const auto found =
            std::find_if(orders.begin(), orders.end(),
                         [id](const ManagedOrder& order) { return order.order.id == id; });
        if (found == orders.end()) {
            throw std::runtime_error("fleet manager: unknown order");
        }
        return *found;
    }

    /// Looks up a mutable order by id, throwing when it is unknown.
    /// @param id the order id
    [[nodiscard]] ManagedOrder& findOrderMutable(int id) {
        const auto found =
            std::find_if(orders.begin(), orders.end(),
                         [id](const ManagedOrder& order) { return order.order.id == id; });
        if (found == orders.end()) {
            throw std::runtime_error("fleet manager: unknown order");
        }
        return *found;
    }

    /// Returns true when a robot has no order and is parked or returning.
    /// @param robot the robot id
    [[nodiscard]] bool robotIsFree(int robot) const {
        const auto slot = static_cast<std::size_t>(robot);
        return robotOrder[slot] < 0;
    }

    /// Appends a state transition and updates the stored order.
    /// @param order the order that changed
    /// @param state the new state
    void setState(ManagedOrder& order, OrderState state) {
        if (order.state == state) {
            return;
        }
        order.state = state;
        order.updatedMs = static_cast<long long>(
            std::chrono::duration_cast<std::chrono::milliseconds>(clock.now().time_since_epoch())
                .count());
        pendingEvents.push_back(OrderEvent{order.order.id, state, order.robot, order.updatedMs});
    }

    /// Converts a duration in simulated seconds to whole manager ticks.
    /// @param seconds the duration to convert
    [[nodiscard]] int stepsFor(double seconds) const {
        if (tickSeconds <= 0.0) {
            return 1;
        }
        const long long steps = std::llround(seconds / tickSeconds);
        return static_cast<int>(steps < 1 ? 1 : steps);
    }

    /// Returns true when no other robot is on or heading for a cell. Used to
    /// keep two robots from claiming the same station cell at once.
    /// @param cell the station cell to test
    /// @param self the robot that would use the cell
    [[nodiscard]] bool stationAvailable(GridCell cell, int self) const {
        for (int other = 0; other < fleetSize; ++other) {
            if (other == self) {
                continue;
            }
            const auto slot = static_cast<std::size_t>(other);
            if (lastStatus[slot].cell == cell) {
                return false;
            }
            if (robotOrder[slot] >= 0) {
                const ManagedOrder& order = findOrder(robotOrder[slot]);
                if (order.order.pickup == cell || order.order.dropoff == cell) {
                    return false;
                }
            }
        }
        return true;
    }

    /// Assigns pending orders to free robots, highest effective priority first.
    void assignOrders() {
        bool anyPending = false;
        for (const ManagedOrder& order : orders) {
            if (order.state == OrderState::Pending) {
                anyPending = true;
                break;
            }
        }
        if (!anyPending) {
            return;
        }

        std::vector<GridCell> positions(static_cast<std::size_t>(fleetSize));
        std::vector<int> freeRobots;
        for (int robot = 0; robot < fleetSize; ++robot) {
            positions[static_cast<std::size_t>(robot)] =
                lastStatus[static_cast<std::size_t>(robot)].cell;
            if (robotIsFree(robot)) {
                freeRobots.push_back(robot);
            }
        }
        if (freeRobots.empty()) {
            return;
        }

        std::vector<std::size_t> order;
        order.reserve(orders.size());
        for (std::size_t slot = 0; slot < orders.size(); ++slot) {
            if (orders[slot].state == OrderState::Pending) {
                order.push_back(slot);
            }
        }
        const int ageing = stepsFor(config.priorityAgeingSeconds);
        std::sort(order.begin(), order.end(), [this, ageing](std::size_t lhs, std::size_t rhs) {
            const long long left = effectivePriority(orders[lhs].order, step, ageing);
            const long long right = effectivePriority(orders[rhs].order, step, ageing);
            if (left != right) {
                return left > right;
            }
            return orders[lhs].order.id < orders[rhs].order.id;
        });

        for (const std::size_t slot : order) {
            if (freeRobots.empty()) {
                break;
            }
            const std::optional<int> chosen = assigner.chooseRobot(
                map, orders[slot].order, positions, freeRobots, robotCompleted);
            if (!chosen.has_value()) {
                continue;
            }
            const int robot = chosen.value();
            if (!stationAvailable(orders[slot].order.pickup, robot) ||
                !stationAvailable(orders[slot].order.dropoff, robot)) {
                continue;
            }
            ManagedOrder& managed = orders[slot];
            managed.robot = robot;
            setState(managed, OrderState::Assigned);
            robotOrder[static_cast<std::size_t>(robot)] = managed.order.id;
            goals[static_cast<std::size_t>(robot)] = Goal::Pickup;
            needsReplan = true;
            std::erase(freeRobots, robot);
        }
    }

    /// Forms the planning problem from the robots that still have a goal.
    void buildRequest(PlanRequest& job) const {
        std::vector<GridCell> blocked;
        for (int robot = 0; robot < fleetSize; ++robot) {
            const auto slot = static_cast<std::size_t>(robot);
            const GridCell settled = lastStatus[slot].cell;
            const GridCell committed =
                moving[slot] ? routes[slot].cellAt(targetIndex[slot]) : settled;
            if (goals[slot] == Goal::Parked || committed == goalCell(robot)) {
                blocked.push_back(settled);
                if (moving[slot]) {
                    blocked.push_back(committed);
                }
            }
        }
        job.map = map.withObstacles(blocked);
        for (int robot = 0; robot < fleetSize; ++robot) {
            const auto slot = static_cast<std::size_t>(robot);
            if (goals[slot] == Goal::Parked) {
                continue;
            }
            const GridCell settled = lastStatus[slot].cell;
            const GridCell start = moving[slot] ? routes[slot].cellAt(targetIndex[slot]) : settled;
            const GridCell goal = goalCell(robot);
            if (start == goal) {
                continue;
            }
            job.agents.push_back(Agent{robot, start, goal});
            job.robotIds.push_back(robot);
        }
    }

    /// Records a failed planning call and starts the no-fresh-plan gap timer.
    void notePlanningFailure() {
        ++stats.planningFailures;
        if (diagnosticsEnabled) {
            if (planner.lastCallTimedOut()) {
                ++diag.planningTimeouts;
            } else {
                ++diag.planningRoutingFailures;
            }
            if (!freshPlanPending) {
                freshPlanPending = true;
                freshPlanFailureStep = step;
            }
        }
        needsReplan = true;
    }

    /// Records an adopted plan and closes the no-fresh-plan gap timer.
    void notePlanAdopted() {
        ++stats.planAdoptions;
        if (diagnosticsEnabled && freshPlanPending) {
            const double gap = static_cast<double>(step - freshPlanFailureStep) * tickSeconds;
            diag.secondsWithoutFreshPlan += gap;
            diag.maxSecondsWithoutFreshPlan = std::max(diag.maxSecondsWithoutFreshPlan, gap);
            freshPlanPending = false;
        }
    }

    /// Compares the right of way at every cell shared by two robots with the
    /// previous adopted plan, and counts how often a pair swaps order.
    void analyzeRightOfWay() {
        std::unordered_map<GridCell, std::vector<std::pair<int, int>>, GridCellHash> current;
        for (int robot = 0; robot < fleetSize; ++robot) {
            const auto slot = static_cast<std::size_t>(robot);
            const Route& route = routes[slot];
            std::unordered_map<GridCell, int, GridCellHash> firstVisit;
            for (int time = 0; time <= route.steps(); ++time) {
                firstVisit.try_emplace(route.cellAt(time), time);
            }
            for (const auto& [cell, time] : firstVisit) {
                current[cell].emplace_back(robot, time);
            }
        }
        for (const auto& [cell, visits] : current) {
            if (visits.size() < 2) {
                continue;
            }
            const auto previous = lastPlanCellOrder.find(cell);
            if (previous == lastPlanCellOrder.end() || previous->second.size() < 2) {
                continue;
            }
            for (std::size_t i = 0; i < visits.size(); ++i) {
                for (std::size_t j = i + 1; j < visits.size(); ++j) {
                    int leftOld = -1;
                    int rightOld = -1;
                    for (const auto& [oldRobot, oldTime] : previous->second) {
                        if (oldRobot == visits[i].first) {
                            leftOld = oldTime;
                        }
                        if (oldRobot == visits[j].first) {
                            rightOld = oldTime;
                        }
                    }
                    if (leftOld < 0 || rightOld < 0) {
                        continue;
                    }
                    const int before =
                        visits[i].second < visits[j].second ? visits[i].first : visits[j].first;
                    const int oldBefore = leftOld < rightOld ? visits[i].first : visits[j].first;
                    ++diag.rightOfWayComparisons;
                    if (before != oldBefore) {
                        ++diag.rightOfWayFlips;
                    }
                }
            }
        }
        lastPlanCellOrder = std::move(current);
    }

    /// Starts a background planning call when the window has elapsed.
    void maybeRequestPlan() {
        if (!needsReplan && stepsSincePlan < stepsFor(config.windowSeconds)) {
            return;
        }
        if (step - lastPlanStep < 1) {
            return;
        }
        if (backgroundPlanning) {
            const std::lock_guard<std::mutex> lock(workMutex);
            if (busy || requestWaiting || resultReady) {
                return;
            }
        }
        PlanRequest job;
        buildRequest(job);
        lastPlanStep = step;
        stepsSincePlan = 0;
        needsReplan = false;
        if (job.agents.empty()) {
            return;
        }
        ++stats.planningCalls;
        if (backgroundPlanning) {
            const std::lock_guard<std::mutex> lock(workMutex);
            requestIds = job.robotIds;
            request = std::move(job);
            requestWaiting = true;
            workCv.notify_one();
            return;
        }
        const std::vector<int> robotIds = job.robotIds;
        const auto planStart = std::chrono::steady_clock::now();
        std::optional<Plan> planned = planner.plan(job.map, job.agents);
        if (!planned.has_value() && fallback != nullptr) {
            planned = fallback->plan(job.map, job.agents);
        }
        if (diagnosticsEnabled) {
            diag.planningCallMs.push_back(std::chrono::duration<double, std::milli>(
                                              std::chrono::steady_clock::now() - planStart)
                                              .count());
            diag.planningOrders.push_back(planner.lastPlanningOrder());
            diag.planningAttempts.push_back(planner.lastPlanningAttempt());
        }
        if (!planned.has_value()) {
            notePlanningFailure();
            return;
        }
        if (adoptPlan(*planned, robotIds)) {
            notePlanAdopted();
        } else {
            ++stats.planRejections;
            needsReplan = true;
        }
    }

    /// Re-anchors a finished plan at every robot's current cell. A plan is
    /// rejected unless each robot's occupied cells still appear in its new
    /// route, because that is what makes the switch safe.
    /// @param planned the new plan
    /// @param robotIds the robot id of every route in the plan
    /// @return true when the plan was adopted
    bool adoptPlan(const Plan& planned, const std::vector<int>& robotIds) {
        if (planned.routes.size() != robotIds.size()) {
            return false;
        }
        std::vector<Route> newRoutes = routes;
        std::vector<int> newIndex = index;
        std::vector<int> newTarget = targetIndex;
        for (std::size_t k = 0; k < robotIds.size(); ++k) {
            const int robot = robotIds[k];
            const auto slot = static_cast<std::size_t>(robot);
            const Route& route = planned.routes[k];
            const GridCell origin = lastStatus[slot].cell;
            if (moving[slot]) {
                const GridCell committed = routes[slot].cellAt(targetIndex[slot]);
                if (route.cells.empty() || route.cellAt(0) != committed) {
                    return false;
                }
                Route merged;
                merged.agentId = robot;
                merged.cells.push_back(origin);
                merged.cells.push_back(committed);
                for (int time = 1; time <= route.steps(); ++time) {
                    merged.cells.push_back(route.cellAt(time));
                }
                newRoutes[slot] = std::move(merged);
                newIndex[slot] = 0;
                newTarget[slot] = 1;
                continue;
            }
            int anchor = -1;
            for (int time = 0; time <= route.steps(); ++time) {
                if (route.cellAt(time) == origin) {
                    anchor = time;
                    break;
                }
            }
            if (anchor < 0) {
                return false;
            }
            newRoutes[slot] = route;
            newIndex[slot] = anchor;
        }
        routes = std::move(newRoutes);
        index = std::move(newIndex);
        targetIndex = std::move(newTarget);
        schedule = buildCellSchedule(routes);
        if (diagnosticsEnabled) {
            analyzeRightOfWay();
        }
        return true;
    }

    /// Takes a finished plan from the worker and tries to adopt it.
    void collectPlan() {
        std::optional<Plan> planned;
        std::vector<int> robotIds;
        double planMs = 0.0;
        {
            const std::lock_guard<std::mutex> lock(workMutex);
            if (!resultReady) {
                return;
            }
            planned = std::move(result);
            result.reset();
            resultReady = false;
            robotIds = requestIds;
            planMs = lastPlanMs;
        }
        if (diagnosticsEnabled) {
            diag.planningCallMs.push_back(planMs);
            diag.planningOrders.push_back(planner.lastPlanningOrder());
            diag.planningAttempts.push_back(planner.lastPlanningAttempt());
        }
        if (!planned.has_value()) {
            notePlanningFailure();
            return;
        }
        if (adoptPlan(planned.value(), robotIds)) {
            notePlanAdopted();
        } else {
            ++stats.planRejections;
            needsReplan = true;
        }
    }

    std::vector<int> requestIds;

    /// Reads the true robot positions and finishes any leg that completed.
    void settleArrivals() {
        const std::vector<RobotStatus> statuses = side.statuses();
        for (int robot = 0; robot < fleetSize; ++robot) {
            const auto slot = static_cast<std::size_t>(robot);
            lastStatus[slot] = statuses[slot];
            if (!moving[slot]) {
                continue;
            }
            if (statuses[slot].cell != routes[slot].cellAt(targetIndex[slot])) {
                continue;
            }
            index[slot] = targetIndex[slot];
            moving[slot] = false;
        }
    }

    /// Runs the state transition of every robot that is standing on the cell it
    /// is heading for. This also covers a robot a new plan anchored exactly on
    /// its goal, which no completed leg would have reported.
    void handleArrivals() {
        for (int robot = 0; robot < fleetSize; ++robot) {
            const auto slot = static_cast<std::size_t>(robot);
            if (!moving[slot] && lastStatus[slot].cell == goalCell(robot)) {
                handleArrival(robot);
            }
        }
    }

    /// Runs the pickup, drop-off and parking transition of a robot that just
    /// reached the end of its current route.
    /// @param robot the robot that arrived
    void handleArrival(int robot) {
        const auto slot = static_cast<std::size_t>(robot);
        if (goals[slot] == Goal::Pickup && robotOrder[slot] >= 0) {
            ManagedOrder& order = findOrderMutable(robotOrder[slot]);
            order.robot = robot;
            setState(order, OrderState::InProgress);
            goals[slot] = Goal::Dropoff;
            needsReplan = true;
        } else if (goals[slot] == Goal::Dropoff && robotOrder[slot] >= 0) {
            ManagedOrder& order = findOrderMutable(robotOrder[slot]);
            setState(order, OrderState::Completed);
            ++stats.ordersCompleted;
            ++robotCompleted[slot];
            robotOrder[slot] = -1;
            goals[slot] = Goal::Parking;
            needsReplan = true;
        } else if (!moving[slot] && robotOrder[slot] < 0 && goals[slot] == Goal::Parking) {
            goals[slot] = Goal::Parked;
            routes[slot] = Route{robot, {lastStatus[slot].cell}};
            index[slot] = 0;
        }
    }

    /// Returns true when another robot physically occupies a cell or is already
    /// committed to enter it. This is the safety half of the entry gate: it
    /// keeps two bodies out of one cell even if a plan is adopted mid-move.
    /// @param mover the robot asking for the cell
    /// @param cell the cell being entered
    [[nodiscard]] bool cellOccupiedByOther(int mover, GridCell cell) const {
        for (int other = 0; other < fleetSize; ++other) {
            if (other == mover) {
                continue;
            }
            const auto slot = static_cast<std::size_t>(other);
            if (lastStatus[slot].cell == cell) {
                return true;
            }
            if (moving[slot] && routes[slot].cellAt(targetIndex[slot]) == cell) {
                return true;
            }
        }
        return false;
    }

    /// Publishes the cells the pass-order rule lets each robot enter now.
    void publishWaypoints() {
        std::vector<int> currentIndex(static_cast<std::size_t>(fleetSize));
        std::vector<int> nextIndex(static_cast<std::size_t>(fleetSize));
        std::vector<bool> advancing(static_cast<std::size_t>(fleetSize), false);
        std::vector<int> holdReason(static_cast<std::size_t>(fleetSize), 0);
        for (int robot = 0; robot < fleetSize; ++robot) {
            const auto slot = static_cast<std::size_t>(robot);
            currentIndex[slot] = index[slot];
            if (moving[slot]) {
                nextIndex[slot] = targetIndex[slot];
                advancing[slot] = true;
            } else if (index[slot] < routes[slot].steps()) {
                nextIndex[slot] = index[slot] + 1;
                advancing[slot] = true;
            } else {
                nextIndex[slot] = index[slot];
            }
        }

        if (config.passOrderEnabled) {
            bool changed = true;
            while (changed) {
                changed = false;
                for (int robot = 0; robot < fleetSize; ++robot) {
                    const auto slot = static_cast<std::size_t>(robot);
                    if (!advancing[slot] || nextIndex[slot] == currentIndex[slot]) {
                        continue;
                    }
                    const GridCell target = routes[slot].cellAt(nextIndex[slot]);
                    const bool occupied = cellOccupiedByOther(robot, target);
                    const bool passOrderRefused =
                        !occupied &&
                        !passOrderAllows(robot, target, nextIndex[slot], currentIndex, nextIndex,
                                         advancing, routes, schedule, false);
                    if (occupied || passOrderRefused) {
                        holdReason[slot] = occupied ? 1 : 2;
                        nextIndex[slot] = currentIndex[slot];
                        advancing[slot] = false;
                        changed = true;
                    }
                }
            }
        }

        if (diagnosticsEnabled) {
            for (int robot = 0; robot < fleetSize; ++robot) {
                const auto slot = static_cast<std::size_t>(robot);
                if (moving[slot]) {
                    continue;
                }
                if (holdReason[slot] == 1) {
                    diag.cellOccupiedWaitSeconds[slot] += tickSeconds;
                } else if (holdReason[slot] == 2) {
                    diag.passOrderWaitSeconds[slot] += tickSeconds;
                } else if (!advancing[slot]) {
                    diag.noWaypointSeconds[slot] += tickSeconds;
                }
            }
        }

        for (int robot = 0; robot < fleetSize; ++robot) {
            const auto slot = static_cast<std::size_t>(robot);
            if (moving[slot]) {
                continue;
            }
            if (advancing[slot] && nextIndex[slot] != index[slot]) {
                moving[slot] = true;
                targetIndex[slot] = nextIndex[slot];
                side.setAllowedWaypoints(robot, {routes[slot].cellAt(index[slot]),
                                                 routes[slot].cellAt(targetIndex[slot])});
            } else {
                side.setAllowedWaypoints(robot, {});
            }
        }
    }

    /// Records the true cells of this tick and updates the safety verdict. A
    /// moving robot is counted at the cell it still fully occupies, so this is a
    /// coarse grid check; the independent continuous judge is the real gate.
    void recordFrame() {
        judge::Frame frame;
        frame.reserve(static_cast<std::size_t>(fleetSize));
        for (const RobotStatus& status : lastStatus) {
            frame.push_back(status.cell);
        }
        judge::RunLog window;
        window.agentCount = fleetSize;
        window.frames.push_back(previousFrame);
        window.frames.push_back(frame);
        const judge::Report report = judge::analyse(window);
        verdict.collisions += report.collisions;
        verdict.swaps += report.swaps;
        for (const judge::Violation& violation : report.violations) {
            if (verdict.violations.size() < 200) {
                verdict.violations.push_back(violation);
            }
        }
        previousFrame = std::move(frame);
    }

    /// Returns true when the fleet still has work to do.
    [[nodiscard]] bool hasOutstandingWork() const {
        for (const ManagedOrder& order : orders) {
            if (order.state == OrderState::Pending || order.state == OrderState::Assigned ||
                order.state == OrderState::InProgress) {
                return true;
            }
        }
        return std::ranges::any_of(goals, [](Goal goal) { return goal != Goal::Parked; });
    }

    /// Runs one grid step.
    /// @param seconds how much simulated time this step covers
    void tick(double seconds) {
        tickSeconds = seconds;
        assignOrders();
        maybeRequestPlan();
        if (backgroundPlanning) {
            collectPlan();
        }
        publishWaypoints();
        side.advance(seconds);
        step++;
        stepsSincePlan++;
        settleArrivals();
        handleArrivals();
        updateProgress();

        const bool outstanding = hasOutstandingWork();
        const bool moved = anyRobotMoved();
        recordFrame();
        if (outstanding && !moved) {
            ++stagnantSteps;
        } else {
            stagnantSteps = 0;
        }
        deadlock = stagnantSteps >= stepsFor(config.deadlockPatienceSeconds);
        stats.steps = step;

        if (started) {
            const double seconds_elapsed =
                std::chrono::duration<double>(clock.now() - startedAt).count();
            stats.elapsedSeconds = seconds_elapsed;
            stats.ordersPerHour =
                seconds_elapsed > 0.0
                    ? static_cast<double>(stats.ordersCompleted) * 3600.0 / seconds_elapsed
                    : 0.0;
        }
    }

    /// Returns true when some robot's fully occupied cell changed since the
    /// previous tick.
    [[nodiscard]] bool anyRobotMoved() const {
        for (int robot = 0; robot < fleetSize; ++robot) {
            const auto slot = static_cast<std::size_t>(robot);
            if (lastStatus[slot].cell != previousFrame[slot]) {
                return true;
            }
        }
        return false;
    }

    /// Returns the total Manhattan distance from every robot with a goal to
    /// that goal.
    [[nodiscard]] int totalGoalDistance() const {
        int total = 0;
        for (int robot = 0; robot < fleetSize; ++robot) {
            const auto slot = static_cast<std::size_t>(robot);
            if (goals[slot] == Goal::Parked) {
                continue;
            }
            total += manhattan_distance(lastStatus[slot].cell, goalCell(robot));
        }
        return total;
    }

    /// Updates the no-progress alarm. Progress is a completed order or a new
    /// lowest total goal distance; the alarm fires after the configured window
    /// without either.
    void updateProgress() {
        if (config.progressWindowSeconds <= 0.0) {
            return;
        }
        const int distance = totalGoalDistance();
        if (!hasProgressBaseline) {
            hasProgressBaseline = true;
            lastProgressOrders = stats.ordersCompleted;
            lowestGoalDistance = distance;
            lastProgressStep = step;
        } else if (stats.ordersCompleted != lastProgressOrders) {
            lastProgressOrders = stats.ordersCompleted;
            lowestGoalDistance = distance;
            lastProgressStep = step;
        } else if (distance < lowestGoalDistance) {
            lowestGoalDistance = distance;
            lastProgressStep = step;
        }
        secondsWithoutProgress = static_cast<double>(step - lastProgressStep) * tickSeconds;
        noProgress = secondsWithoutProgress >= config.progressWindowSeconds;
        progressAlarm = progressAlarm || noProgress;
    }
};

FleetManager::FleetManager(const Map& map, RobotSide& side, Planner& planner,
                           TaskAssigner& assigner, Clock& clock, FleetManagerConfig config,
                           Planner* fallback)
    : impl_(std::make_unique<Impl>(map, side, planner, assigner, clock, config, fallback)) {}

FleetManager::~FleetManager() = default;

void FleetManager::setBackgroundPlanning(bool enabled) {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->backgroundPlanning = enabled;
}

void FleetManager::restoreOrders(const std::vector<StoredOrder>& stored) {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    for (const StoredOrder& record : stored) {
        ManagedOrder managed;
        managed.order.id = record.id;
        managed.order.pickup = record.pickup;
        managed.order.dropoff = record.dropoff;
        managed.order.priority = record.priority;
        managed.createdMs = record.createdMs;
        managed.updatedMs = record.updatedMs;
        const std::optional<OrderState> parsed = orderStateFromText(record.state);
        const OrderState state = parsed.value_or(OrderState::Pending);
        if (state == OrderState::Completed || state == OrderState::Cancelled ||
            state == OrderState::Failed) {
            managed.state = state;
        } else if (state == OrderState::InProgress) {
            managed.state = OrderState::Failed;
        } else {
            managed.state = OrderState::Pending;
        }
        impl_->nextOrderId = std::max(impl_->nextOrderId, record.id + 1);
        impl_->orders.push_back(managed);
    }
    std::sort(impl_->orders.begin(), impl_->orders.end(),
              [](const ManagedOrder& a, const ManagedOrder& b) { return a.order.id < b.order.id; });
    impl_->needsReplan = true;
}

int FleetManager::submitOrder(GridCell pickup, GridCell dropoff, int priority) {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    ManagedOrder managed;
    managed.order.id = impl_->nextOrderId++;
    managed.order.pickup = pickup;
    managed.order.dropoff = dropoff;
    managed.order.priority = priority;
    managed.arrivalStep = impl_->step;
    const long long nowMs = static_cast<long long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(impl_->clock.now().time_since_epoch())
            .count());
    managed.createdMs = nowMs;
    managed.updatedMs = nowMs;
    if (!impl_->started) {
        impl_->started = true;
        impl_->startedAt = impl_->clock.now();
    }
    impl_->orders.push_back(managed);
    impl_->pendingEvents.push_back(OrderEvent{managed.order.id, OrderState::Pending, -1, nowMs});
    impl_->needsReplan = true;
    return managed.order.id;
}

bool FleetManager::cancelOrder(int orderId) {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto found =
        std::find_if(impl_->orders.begin(), impl_->orders.end(),
                     [orderId](const ManagedOrder& order) { return order.order.id == orderId; });
    if (found == impl_->orders.end()) {
        return false;
    }
    if (found->state == OrderState::Pending || found->state == OrderState::Assigned) {
        if (found->robot >= 0) {
            const auto slot = static_cast<std::size_t>(found->robot);
            impl_->robotOrder[slot] = -1;
            impl_->goals[slot] = Goal::Parking;
            found->robot = -1;
        }
        impl_->setState(*found, OrderState::Cancelled);
        impl_->needsReplan = true;
        return true;
    }
    return false;
}

void FleetManager::tick(double seconds) {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->tick(seconds);
}

std::vector<ManagedOrder> FleetManager::orders() const {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->orders;
}

std::optional<ManagedOrder> FleetManager::order(int id) const {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto found =
        std::find_if(impl_->orders.begin(), impl_->orders.end(),
                     [id](const ManagedOrder& order) { return order.order.id == id; });
    if (found == impl_->orders.end()) {
        return std::nullopt;
    }
    return *found;
}

std::vector<RobotStatus> FleetManager::robotStatuses() const {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->lastStatus;
}

std::vector<int> FleetManager::robotOrderIds() const {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->robotOrder;
}

bool FleetManager::deadlocked() const {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->deadlock;
}

judge::Report FleetManager::safetyReport() const {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->verdict;
}

ThroughputStats FleetManager::throughput() const {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    ThroughputStats stats = impl_->stats;
    stats.noProgress = impl_->noProgress;
    stats.progressAlarm = impl_->progressAlarm;
    stats.secondsWithoutProgress = impl_->secondsWithoutProgress;
    stats.ordersOpen = 0;
    for (const ManagedOrder& order : impl_->orders) {
        if (order.state == OrderState::Pending || order.state == OrderState::Assigned ||
            order.state == OrderState::InProgress) {
            ++stats.ordersOpen;
        } else if (order.state == OrderState::Cancelled) {
            ++stats.ordersCancelled;
        } else if (order.state == OrderState::Failed) {
            ++stats.ordersFailed;
        }
    }
    return stats;
}

ManagerDiagnostics FleetManager::diagnostics() const {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->diag;
}

std::string FleetManager::routeDump() const {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    std::ostringstream out;
    for (int robot = 0; robot < impl_->fleetSize; ++robot) {
        const auto slot = static_cast<std::size_t>(robot);
        const Route& route = impl_->routes[slot];
        out << "r" << robot << " idx=" << impl_->index[slot] << " tgt=" << impl_->targetIndex[slot]
            << " moving=" << (impl_->moving[slot] ? 1 : 0) << " steps=" << route.steps()
            << " route=";
        const int limit = std::min(route.steps(), 14);
        for (int time = 0; time <= limit; ++time) {
            out << '(' << route.cellAt(time).x << ',' << route.cellAt(time).y << ')';
        }
        out << ';';
    }
    return out.str();
}

std::vector<OrderEvent> FleetManager::drainEvents() {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    std::vector<OrderEvent> events = std::move(impl_->pendingEvents);
    impl_->pendingEvents.clear();
    return events;
}

}  // namespace lotse::core
