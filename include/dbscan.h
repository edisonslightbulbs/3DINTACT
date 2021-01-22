#ifndef DBSCAN_H
#define DBSCAN_H

#include <cmath>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <vector>

#include "logger.h"
#include "point.h"
#include "timer.h"

extern const int PASS;
extern const int FAIL;
extern const int NOISE;
extern const int UNCLASSIFIED;

const int MINIMUM_POINTS = 4;
const float EPSILON = 0.75 * 0.75;

class dbscan {

private:
    int findNeighbors(Point t_point, int t_cluster);
    std::vector<int> neighbourhood(Point t_point);

public:
    std::vector<Point> m_points;

    explicit dbscan(std::vector<Point>& t_point)
        : m_points(t_point)
    {
        int cluster = 0;
        for (auto point : m_points) {
            if (point.m_cluster == UNCLASSIFIED) {
                if (findNeighbors(point, cluster) != FAIL) {
                    cluster += 1;
                }
            }
        }
    }

    ~dbscan() = default;
};
#endif /* DBSCAN_H */

/**
 * the malarky of global variables
 * see:
 * https://stackoverflow.com/questions/9702053/how-to-declare-a-global-variable-in-c
 * date: 2020-12-31 12:45
 */
