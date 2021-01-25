#include <vector>

#include "workers.h"

Workers::Workers () = default;

Workers::Workers(const int& t_T, const std::vector<Point>& t_points)
{
    /** distribute points over specified number of threads*/
    int pointsPerThread = (int)t_points.size() / t_T;

    int threadId = 1;
    int threadPoints = 0;

    std::vector<Point> temp;
    std::vector<std::vector<Point>> work;
    for (const auto& point : t_points) {
        if (threadId != t_T) {
            threadPoints++;
            temp.push_back(point);

            if (threadPoints == pointsPerThread) {
                work.push_back(temp);
                temp.clear();
                threadId++;
                threadPoints = 0;
            }
        } else {
            temp.push_back(point);
        }
    }
    work.push_back(temp);
    sptr_work = std::make_shared<std::vector<std::vector<Point>>>(work);
}
