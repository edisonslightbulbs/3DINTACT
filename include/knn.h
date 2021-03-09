#ifndef KNN_H
#define KNN_H

#include <memory>
#include <vector>

#include "point.h"

class knn {

public:
    knn() = default;
    ~knn() = default;
    static float elbow(std::vector<Point>& points);
};
#endif /* KNN_H */
