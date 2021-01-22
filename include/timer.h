#ifndef TIMER_H
#define TIMER_H

#include "logger.h"
#include <chrono>
#include <cmath>

class Timer {
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;

public:
    Timer() { m_startTime = std::chrono::high_resolution_clock::now(); }
    std::string getDuration () {

        auto stopTime = std::chrono::high_resolution_clock::now();

        auto start = std::chrono::time_point_cast<std::chrono::microseconds>(
                m_startTime)
                .time_since_epoch()
                .count();
        auto stop
                = std::chrono::time_point_cast<std::chrono::microseconds>(stopTime)
                        .time_since_epoch()
                        .count();

        auto duration = stop - start; // duration in micro seconds

        if (duration < 1000000 && duration > 1000 ){
            double ms = duration * 0.001;
            return std::to_string(ms) + "ms";
        }
        else if (duration > 1000000 ){
            double s = duration * 0.000001;
            return std::to_string(s) + "s";
        }
        else {
            return std::to_string(duration) + "us";
        }
    }

    ~Timer()
    {
    }

};

#endif /* TIMER_H */
