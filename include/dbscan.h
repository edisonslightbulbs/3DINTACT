#ifndef DBSCAN_H
#define DBSCAN_H

#include <cmath>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <vector>

#include "logger.h"
#include "point.h"
#include "point3d.h"
#include "timer.h"

extern const int SUCCESS;
extern const int FAILURE;

const int MINIMUM_POINTS = 4;
const float EPSILON = 0.75 * 0.75;

class Dbscan {

private:
    int findNeighbors(Point* ptr_point, int t_cluster);
    std::vector<int> inEpsilonNeighbourhood(Point* ptr_point3d);
    void clusterPoints();

public:
    std::vector<Point*> m_points3d;

    explicit Dbscan(std::vector<Point*>& t_points3d)
        : m_points3d(t_points3d)
    {
        clusterPoints();
    }

    ~Dbscan() = default;
};
#endif /* DBSCAN_H */

/**
 * the malarky of global variables
 * see:
 * https://stackoverflow.com/questions/9702053/how-to-declare-a-global-variable-in-c
 * date: 2020-12-31 12:45
 */
