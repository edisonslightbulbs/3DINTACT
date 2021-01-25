#ifndef KNN_H
#define KNN_H

#include <memory>
#include <mutex>
#include <vector>

#include "point.h"
#include "timer.h"

class Knn {

public:
    /** neighbours */
    std::mutex m;

    Knn() = default;
    Knn(const std::vector<Point>& t_points, std::vector<Point>& t_work,
        std::shared_ptr<std::vector<std::vector<Point>>>& sptr_neighbours)
    {
        std::vector<Point> temp;

        /** find and store each points neighbour */
        for (auto& coordinate : t_work) {
            std::vector<Point> mutableClone = t_points;

            for (auto& point : mutableClone) {
                if (coordinate == point) {
                    continue;
                }
                auto distance = (float)coordinate.distance(point);
                point.m_distance = { coordinate.m_id, distance };
                temp.push_back(point);
            }
            m.lock();
            for (int i = 0; i < sptr_neighbours->size(); ++i) {
                if ((*sptr_neighbours)[i].empty()) {
                    (*sptr_neighbours)[i] = temp;
                    break;
                }
            }
            m.unlock();
            temp.clear();
        }
    }
};
#endif /* KNN_H */

/**
 * good practice
 * see:
 * https://stackoverflow.com/questions/25274312/is-it-a-good-practice-to-define-c-functions-inside-header-files
 * date: 2021-01-18 15:48
 */

/**
 * copying a shared_ptr correctly
 * see:
 * https://stackoverflow.com/questions/22782288/how-to-properly-duplicate-an-object-given-its-shared-ptr
 * date: 2021-01-18 16:01
 */
