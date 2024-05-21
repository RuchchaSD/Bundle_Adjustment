// Timer.h
#ifndef TIMER_H
#define TIMER_H

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @class Timer
 * @brief A simple timer class for measuring time intervals.
 */
class Timer {
public:
    /**
     * @brief Default constructor, enables the timer.
     */
    Timer() : enabled(true) {}
    ~Timer() = default;

    /**
     * @brief Initializes timers with the given names.
     * @param names Vector of timer names.
     */
    void setVariables(const std::vector<std::string>& names) {
        for (const auto& name : names) {
            timers[name]; // Initializes timers with the given names.
        }
    }

    /**
     * @brief Starts the timer for the given name.
     * @param name Name of the timer to start.
     */
    void startClock(const std::string& name) {
        if (!enabled) return;
        timers[name].start = std::chrono::high_resolution_clock::now();
        timers[name].isRunning = true;
    }

    /**
     * @brief Pauses the timer for the given name.
     * @param name Name of the timer to pause.
     */
    void pauseClock(const std::string& name) {
        if (!enabled || !timers[name].isRunning) return;
        auto now = std::chrono::high_resolution_clock::now();
        timers[name].accumulatedDuration += now - timers[name].start;
        timers[name].isRunning = false;
    }

    /**
     * @brief Resets the timer for the given name.
     * @param name Name of the timer to reset.
     */
    void resetClock(const std::string& name) {
        if (!enabled) return;
        timers[name].accumulatedDuration = std::chrono::high_resolution_clock::duration::zero();
        timers[name].isRunning = false;
    }

    /**
     * @brief Gets the elapsed time in seconds for the given timer.
     * @param name Name of the timer.
     * @return Elapsed time in seconds.
     */
    double getTimeInSeconds(const std::string& name) {
        return std::chrono::duration<double>(getAccumulatedDuration(name)).count();
    }

    /**
     * @brief Gets the elapsed time in milliseconds for the given timer.
     * @param name Name of the timer.
     * @return Elapsed time in milliseconds.
     */
    double getTimeInMilliSeconds(const std::string& name) {
        return std::chrono::duration<double, std::milli>(getAccumulatedDuration(name)).count();
    }

    /**
     * @brief Gets the elapsed time in minutes for the given timer.
     * @param name Name of the timer.
     * @return Elapsed time in minutes.
     */
    double getTimeInMinutes(const std::string& name) {
        return std::chrono::duration<double, std::ratio<60>>(getAccumulatedDuration(name)).count();
    }

    /**
     * @brief Enables or disables the timer.
     * @param state True to enable, false to disable.
     */
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

    /**
     * @brief Gets the accumulated duration for the given timer.
     * @param name Name of the timer.
     * @return Accumulated duration.
     */
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
