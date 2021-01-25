#ifndef TIMER_H
#define TIMER_H

#include "logger.h"
#include <chrono>
#include <cmath>
#include <thread>

using namespace std::this_thread;     // sleep_for, sleep_until
using namespace std::chrono_literals; // ns, us, ms, s, h, etc.

#define wait sleep_for
constexpr double SECOND = 1000000;
constexpr double MILLISECOND = 1000;
constexpr double TO_SECOND = 0.000001;
constexpr double TO_MILLISECOND = 0.001;

class Timer {
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;

public:
    Timer() { m_startTime = std::chrono::high_resolution_clock::now(); }
    std::string getDuration()
    {

        auto stopTime = std::chrono::high_resolution_clock::now();

        auto start = std::chrono::time_point_cast<std::chrono::microseconds>(
            m_startTime)
                         .time_since_epoch()
                         .count();
        auto stop
            = std::chrono::time_point_cast<std::chrono::microseconds>(stopTime)
                  .time_since_epoch()
                  .count();

        auto duration = stop - start; // micro seconds

        /** friendly output */
        if (duration < SECOND && duration > MILLISECOND) {
            double ms = (double)duration * TO_MILLISECOND;
            return std::to_string(ms) + "ms";
        }
        if (duration > SECOND) {
            double s = duration * TO_SECOND;
            return std::to_string(s) + "s";
        }
        return std::to_string(duration) + "us";
    }

    ~Timer() = default;
};

#endif /* TIMER_H */

/**
 * C style time delay
 * see:
 * https://stackoverflow.com/questions/158585/how-do-you-add-a-timed-delay-to-a-c-program
 * date: 2021-01-18 15:48
 */

/**
 * constexpr
 * see:
 * https://www.geeksforgeeks.org/understanding-constexper-specifier-in-c/
 * date: 2021-01-20 15:21
 */
