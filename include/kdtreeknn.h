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
    Point m_point;
    node* m_left;
    node* m_right;

    explicit node(Point& point, node left, node right)
        : m_point(point)
        , m_left(&left)
        , m_right(&right)
    {
    }
};

node tree(std::vector<Point>& points, int d)
{
    /** query k */
    const int k = points.size();
    int axis = d % k;

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

    /** choose median as pivot point */
    int median = (int)points.size() / 2;

    std::vector<Point> vec1(points.begin(), points.begin() + median);
    std::vector<Point> vec2(points.begin() + median, points.end());

    return node(points[median], tree(vec1, d + 1), tree(vec2, d + 1));
}

namespace kdtreeknn {

std::vector<float> run(std::vector<Point>& points, const int& K)
{
    node kdtree = tree(points, 0);

    std::vector<float> knn;
    return knn;
}
};

#endif /* KDTREEKNN_H */
