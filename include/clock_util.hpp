// One clock, used identically by all three processes: std::chrono::
// steady_clock, which on Linux is backed by CLOCK_MONOTONIC, a single
// kernel-wide clock shared by every process on the host (see
// supervisor_core.hpp's header comment for why that is safe to rely on
// here and where it would not be).
#pragma once

#include <chrono>
#include <thread>

namespace cbs {

inline double now_steady_ms() {
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

inline void sleep_ms(double ms) {
    if (ms <= 0.0) return;
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(ms));
}

}  // namespace cbs
