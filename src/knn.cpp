#include "knn.h"
#include "io.h"

std::vector<float> knn::compute(std::vector<Point>& points)
{
    /** list of neighbourhoods for each point */
    std::vector<std::vector<Point>> neighbourhoods;
    std::vector<Point> neighbours;

    for (auto& point : points) {
        for (auto& n : points) {
            if (point == n) {
                continue;
            }
            float distance = n.distance(point);
            n.m_distance.first = point.m_id;
            n.m_distance.second = distance;
            neighbours.push_back(n);
        }
        neighbourhoods.push_back(neighbours);
        neighbours.clear();
    }

    for (auto& neighbourhood : neighbourhoods) {
        Point::sort(neighbourhood); // please verify the output from here
    }

    std::vector<float> knn;
    for (auto& neighbourhood : neighbourhoods) {
        knn.push_back(neighbourhood[3].m_distance.second);
    }
    std::sort(knn.begin(), knn.end(), std::greater<>());

    const std::string OUTPUT = IO::pwd() + "/build/knn.csv";
    IO::write(knn, OUTPUT);

    return knn;
}
