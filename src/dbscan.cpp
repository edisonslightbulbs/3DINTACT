#include "dbscan.h"

std::vector<int> dbscan::neighbourhood(Point t_point)
{
    int index = 0;
    std::vector<int> neighbours;
    for (auto points : m_points) {
        if (t_point.distance(points) <= EPSILON) {
            neighbours.push_back(index);
        }
        index++;
    }
    return neighbours;
}

int dbscan::findNeighbors(Point t_point, int t_cluster)
{
    std::vector<int> neighbours = neighbourhood(t_point);
    if (neighbours.size() < MINIMUM_POINTS) {
        t_point.m_cluster = NOISE;
        return FAIL;
    }

    int index = 0;
    int centroid = 0;
    for (auto neighbour : neighbours) {
        m_points.at(neighbour).m_cluster = t_cluster;
        if (m_points.at(neighbour) == t_point) {
            centroid = index;
        }
        ++index;
    }
    neighbours.erase(neighbours.begin() + centroid);

    for (std::vector<int>::size_type i = 0, n = neighbours.size(); i < n; ++i) {
        std::vector<int> nextNeighbours
            = neighbourhood(m_points.at(neighbours[i]));

        if (nextNeighbours.size() >= MINIMUM_POINTS) {
            for (auto neighbour : nextNeighbours) {
                if (m_points.at(neighbour).unclassified()) {
                    neighbours.push_back(neighbour);
                    n = neighbours.size();
                    m_points.at(neighbour).m_cluster = t_cluster;
                }
            }
        }
    }
    return PASS;
}

/**
 * std::vector<int>::size_type usage
 * see
 * https://stackoverflow.com/questions/4849632/vectorintsize-type-in-c
 * date: 2020-12-30 20:22
 */
