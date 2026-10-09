// Implements the independent safety judge declared in judge.hpp.

#include "lotse/judge/judge.hpp"

#include <cstddef>

namespace lotse::judge {

Report analyse(const RunLog& log) {
    Report report;
    const auto agentCount = static_cast<std::size_t>(log.agentCount);

    for (std::size_t step = 0; step < log.frames.size(); ++step) {
        const Frame& frame = log.frames[step];
        for (std::size_t a = 0; a < agentCount; ++a) {
            for (std::size_t b = a + 1; b < agentCount; ++b) {
                if (frame[a] == frame[b]) {
                    ++report.collisions;
                    report.violations.push_back(
                        Violation{ViolationKind::Collision, static_cast<int>(step),
                                  static_cast<int>(a), static_cast<int>(b)});
                }
            }
        }

        if (step + 1 >= log.frames.size()) {
            continue;
        }
        const Frame& next = log.frames[step + 1];
        for (std::size_t a = 0; a < agentCount; ++a) {
            for (std::size_t b = a + 1; b < agentCount; ++b) {
                const bool exchanged = frame[a] == next[b] && frame[b] == next[a];
                if (exchanged && frame[a] != frame[b]) {
                    ++report.swaps;
                    report.violations.push_back(
                        Violation{ViolationKind::Swap, static_cast<int>(step), static_cast<int>(a),
                                  static_cast<int>(b)});
                }
            }
        }
    }
    return report;
}

}  // namespace lotse::judge
