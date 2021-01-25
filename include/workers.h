#ifndef WORKERS_H
#define WORKERS_H

#include <memory>
#include <vector>

#include "point.h"

class Workers {

public:
    /** distributed points to work on */
    std::shared_ptr<std::vector<std::vector<Point>>> sptr_work;

    Workers();
    Workers(const int& t_T, const std::vector<Point>& t_points);
};
#endif /* WORKERS_H */
