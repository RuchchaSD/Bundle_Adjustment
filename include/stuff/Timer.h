#ifndef TIMER_H
#define TIMER_H

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

class Timer {
public:
    Timer() : enabled(true) {}  // Timer is enabled by default
    ~Timer() = default;

    void setVariables(const std::vector<std::string>& names) {
        for (const auto& name : names) {
            timers[name]; // Initializes timers with the given names.
        }
    }

    void startClock(const std::string& name) {
        if (!enabled) return;
        timers[name].start = std::chrono::high_resolution_clock::now();
        timers[name].isRunning = true;
    }

    void pauseClock(const std::string& name) {
        if (!enabled || !timers[name].isRunning) return;
        auto now = std::chrono::high_resolution_clock::now();
        timers[name].accumulatedDuration += now - timers[name].start;
        timers[name].isRunning = false;
    }

    void resetClock(const std::string& name) {
        if (!enabled) return;
        timers[name].accumulatedDuration = std::chrono::high_resolution_clock::duration::zero();
        timers[name].isRunning = false;
    }

    double getTimeInSeconds(const std::string& name) {
        return std::chrono::duration<double>(getAccumulatedDuration(name)).count();
    }

    double getTimeInMilliSeconds(const std::string& name) {
        return std::chrono::duration<double, std::milli>(getAccumulatedDuration(name)).count();
    }

    double getTimeInMinutes(const std::string& name) {
        return std::chrono::duration<double, std::ratio<60>>(getAccumulatedDuration(name)).count();
    }

    void setEnabled(bool state) {
        enabled = state;
    }

private:
    struct TimerInfo {
        std::chrono::high_resolution_clock::time_point start;
        std::chrono::high_resolution_clock::duration accumulatedDuration = std::chrono::high_resolution_clock::duration::zero();
        bool isRunning = false;
    };

    std::unordered_map<std::string, TimerInfo> timers;
    bool enabled;

    std::chrono::high_resolution_clock::duration getAccumulatedDuration(const std::string& name) {
        if (timers.find(name) == timers.end()) return std::chrono::high_resolution_clock::duration::zero();
        if (timers[name].isRunning) {
            auto now = std::chrono::high_resolution_clock::now();
            return timers[name].accumulatedDuration + (now - timers[name].start);
        }
        return timers[name].accumulatedDuration;
    }
};

#endif // TIMER_H
