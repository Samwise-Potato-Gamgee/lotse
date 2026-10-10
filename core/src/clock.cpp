// Implements the injectable clocks declared in clock.hpp.

#include "lotse/core/clock.hpp"

namespace lotse::core {

std::chrono::steady_clock::time_point SteadyClock::now() const noexcept {
    return std::chrono::steady_clock::now();
}

ManualClock::ManualClock(std::chrono::steady_clock::time_point start) noexcept : now_(start) {}

std::chrono::steady_clock::time_point ManualClock::now() const noexcept {
    return now_;
}

void ManualClock::advance(std::chrono::steady_clock::duration delta) noexcept {
    now_ += delta;
}

void ManualClock::set(std::chrono::steady_clock::time_point time) noexcept {
    now_ = time;
}

}  // namespace lotse::core
