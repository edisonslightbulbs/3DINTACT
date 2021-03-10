#ifndef KDTREEKNN_H
#define KDTREEKNN_H

#include <map>
#include <vector>

#include "point.h"

static void sortx(std::vector<Point>& points)
{
    std::sort(points.begin(), points.end(),
        [](const Point& point, const Point& other) {
            return point.m_x < other.m_x;
        });
}

static void sorty(std::vector<Point>& points)
{
    std::sort(points.begin(), points.end(),
        [](const Point& point, const Point& other) {
            return point.m_y < other.m_y;
        });
}

static void sortz(std::vector<Point>& points)
{
    std::sort(points.begin(), points.end(),
        [](const Point& point, const Point& other) {
            return point.m_z < other.m_z;
        });
}

struct node {

    Point* m_point;
    node* m_left;
    node* m_right;

    node(std::nullptr_t ptr_void)
        : m_point(nullptr)
        , m_left(nullptr)
        , m_right(nullptr)
    {
    }

    explicit node(Point* ptr_point, node left, node right)
        : m_point(ptr_point)
        , m_left(&left)
        , m_right(&right)
    {
    }
};

std::vector<float> nearest(
    node root, Point point, d, std::vector<float> distaces)
{
    if (root == nullptr) {
        return distances;
    }

    axist = d % R;

    // and why arent these pointers?
    node next = nullptr;
    node opposite = nullptr;

    float distance = point.distance(root.m_point);
    distances.pop_back();

    if (distance < distance[0]) {
        next = root.m_left;
        opposite = root.m_right;
        distances.push_front(distance);
    } else {
        next = root.m_right;
        opposite = root.m_left;
        distances.push_front(distance);
    }
    return nearest(next, point, d + 1, distances);
}

node tree(std::vector<Point>& points, int d)
{
    /** base condition for recursive function */
    if (points.empty()) {
        return nullptr;
    }

    /** set axis based on depth axis */
    int axis = d % R;
    /** R = 3 defined in point.h (global scope) */

    /** sort point list by axis */
    switch (axis) {
    case 0:
        sortx(points);
    case 1:
        sorty(points);
    case 2:
        sortz(points);
    default:
        break;
    }

    /** find median */
    int median = (int)points.size() / 2;

    std::vector<Point> lVec;
    std::vector<Point> rVec;

    /** slice at median appropriately */
    if (points.size() >= 3) {
        lVec = std::vector<Point>(points.begin(), points.begin() + median);
        rVec = std::vector<Point>(points.begin() + median + 1, points.end());
    } else if (points.size() == 2) {
        lVec.push_back(points[0]);
        rVec.push_back(points[1]);
    }

    /** recursively create tree nodes */
    return node(&points[median], tree(lVec, d + 1), tree(rVec, d + 1));
}

namespace kdtreeknn {

/** driver interface */
std::vector<float> run(std::vector<Point>& points, const int& K)
{
    /** construct kdtree */
    node root = tree(points, 0);

    /** container for the four closest distances */
    std::vector<float> nn(K);
    for (int i = 0; i < size(); i++) {
        nn.push_back(__DBL_MAX__);
    }

    std::vector<float> knn;

    return knn;
}
};

#endif /* KDTREEKNN_H */
