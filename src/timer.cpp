#include "timer.h"
#include <chrono>

const double SECOND = 1000000;
const double MILLISECOND = 1000;

Timer::Timer() { m_startTime = std::chrono::high_resolution_clock::now(); }
std::string Timer::getDuration()
{
    auto stopTime = std::chrono::high_resolution_clock::now();
    auto start
        = std::chrono::time_point_cast<std::chrono::microseconds>(m_startTime)
              .time_since_epoch()
              .count();
    auto stop
        = std::chrono::time_point_cast<std::chrono::microseconds>(stopTime)
              .time_since_epoch()
              .count();

    auto duration = stop - start; // duration in micro seconds

    if (duration < SECOND && duration > MILLISECOND) {
        double ms = duration / MILLISECOND;
        return std::to_string(ms) + "ms";
    }
    if (duration > SECOND) {
        double s = duration / SECOND;
        return std::to_string(s) + "s";
    }
    return std::to_string(duration) + "us";
}

std::string Timer::getSeconds()
{
    auto stopTime = std::chrono::high_resolution_clock::now();
    auto start
        = std::chrono::time_point_cast<std::chrono::microseconds>(m_startTime)
              .time_since_epoch()
              .count();
    auto stop
        = std::chrono::time_point_cast<std::chrono::microseconds>(stopTime)
              .time_since_epoch()
              .count();

    auto duration = stop - start; // duration in micro seconds
    double sec = duration / SECOND;

    return std::to_string(sec);
}
