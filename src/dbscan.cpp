#include "dbscan.h"

void Dbscan::clusterPoints()
{
    int cluster = 0;
    for (auto* ptr_point3d : m_points3d) {
        if (ptr_point3d->m_cluster == UNCLASSIFIED) {
            if (findNeighbors(ptr_point3d, cluster) != FAILURE) {
                cluster += 1;
            }
        }
    }
}

std::vector<int> Dbscan::inEpsilonNeighbourhood(Point* ptr_point3d)
{
    int index = 0;
    std::vector<int> neighbours;
    for (auto* ptr_other3d : m_points3d) {
        if (ptr_point3d->distance(ptr_other3d) <= EPSILON) {
            neighbours.push_back(index);
        }
        index++;
    }
    return neighbours;
}

int Dbscan::findNeighbors(Point* ptr_point, int t_cluster)
{
    std::vector<int> neighbours = inEpsilonNeighbourhood(ptr_point);
    if (neighbours.size() < MINIMUM_POINTS) {
        ptr_point->m_cluster = NOISE;
        return FAILURE;
    }

    int index = 0;
    int centroid = 0;
    for (auto neighbour : neighbours) {
        m_points3d.at(neighbour)->m_cluster = t_cluster;
        if (m_points3d.at(neighbour) == ptr_point) {
            centroid = index;
        }
        ++index;
    }
    neighbours.erase(neighbours.begin() + centroid);

    for (std::vector<int>::size_type i = 0, n = neighbours.size(); i < n; ++i) {
        std::vector<int> nextNeighbours
            = inEpsilonNeighbourhood(m_points3d.at(neighbours[i]));

        if (nextNeighbours.size() >= MINIMUM_POINTS) {
            for (auto neighbour : nextNeighbours) {
                if (m_points3d.at(neighbour)->unclassified()) {
                    neighbours.push_back(neighbour);
                    n = neighbours.size();
                    m_points3d.at(neighbour)->m_cluster = t_cluster;
                }
            }
        }
    }
    return SUCCESS;
}

/**
 * std::vector<int>::size_type usage
 * see
 * https://stackoverflow.com/questions/4849632/vectorintsize-type-in-c
 * date: 2020-12-30 20:22
 */
