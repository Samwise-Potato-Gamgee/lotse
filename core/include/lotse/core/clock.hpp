// An injectable monotonic clock so the real-time fleet manager and the service
// can be driven without wall time and tested repeatably.

#pragma once

#include <chrono>

namespace lotse::core {

/// A monotonic time source. Tests substitute a manual clock.
class Clock {
public:
    Clock() = default;
    Clock(const Clock&) = delete;
    Clock& operator=(const Clock&) = delete;
    Clock(Clock&&) = delete;
    Clock& operator=(Clock&&) = delete;
    virtual ~Clock() = default;

    /// Returns the current time on this clock.
    [[nodiscard]] virtual std::chrono::steady_clock::time_point now() const noexcept = 0;
};

/// The real monotonic clock, backed by std::chrono::steady_clock.
class SteadyClock final : public Clock {
public:
    /// Returns the wall-clock reading of std::chrono::steady_clock.
    [[nodiscard]] std::chrono::steady_clock::time_point now() const noexcept override;
};

/// A clock the caller moves forward by hand.
class ManualClock final : public Clock {
public:
    /// Creates a clock starting at the given time.
    /// @param start the initial time
    explicit ManualClock(std::chrono::steady_clock::time_point start = {}) noexcept;

    /// Returns the current manual time.
    [[nodiscard]] std::chrono::steady_clock::time_point now() const noexcept override;

    /// Moves the clock forward by a duration.
    /// @param delta how far to advance
    void advance(std::chrono::steady_clock::duration delta) noexcept;

    /// Sets the clock to an absolute time.
    /// @param time the new time
    void set(std::chrono::steady_clock::time_point time) noexcept;

private:
    std::chrono::steady_clock::time_point now_;
};

}  // namespace lotse::core
