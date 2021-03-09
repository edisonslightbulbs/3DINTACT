#ifndef NAIVEKNN_H
#define NAIVEKNN_H

#include <vector>

#include "point.h"
#include "logger.h"
#include "timer.h"

namespace naiveknn {

std::vector<float> run(const std::vector<Point>& points, const int& K)
{
    Timer timer;
    std::vector<float> knn;
    std::vector<std::vector<Point>> neighbourhoods;

    /** create mutable clone for evaluating and collecting neighbours */
    std::vector<Point> neighbours(points);

    /** O(n^2) search */
    for(const auto& point : points){
        std::vector<Point> neighbourhood;
        for(auto& neighbour : neighbours){
           float distance = neighbour.distance(point);
           neighbour.m_distance = { point.m_id, distance };
           neighbourhood.push_back(neighbour);
        }
        neighbourhoods.push_back(neighbourhood);
    }

    /** quick sort points using distances */
    for (auto& neighourhood : neighbourhoods) {
        Point::sort(neighourhood);
    }

    /** pick out the Kth nearest neighbour */
    for (auto& neighourhood : neighbourhoods) {
        knn.emplace_back(neighourhood[K].m_distance.second);
    }

    LOG(INFO) << "1. Naive knn takes: " << timer.getDuration();

    return knn;
}

}
#endif /* NAIVEKNN_H */
