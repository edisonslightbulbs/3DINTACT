#ifndef POINT_H
#define POINT_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

/**
 * A point may not belong to any cluster.
 * For such cases we deem the point to be
 * Either noise, or unclassified. */
extern const int NOISE;
extern const int UNCLASSIFIED;

// extern const int X; // vector column with x values
// extern const int Y; // vector column with y values
// extern const int Z; // vector column with Z values
// extern const int R; // dimensions

/** todo: optimize data structure for color manipulation
 * Bit values for colors*/

class Point {

public:
    bool operator==(const Point& rhs) const;
    friend std::ostream& operator<<(std::ostream& t_stream, const Point& point);
    friend std::istream& operator>>(std::istream& t_stream, Point& point);

    int m_id;
    int m_cluster;

    float m_x;
    float m_y;
    float m_z;

    Point();
    Point(float t_x, float t_y, float t_z);

    std::pair<int, float> m_distance;

    float distance(Point point) const;

    static Point centroid(std::vector<Point>& t_points);

    static void sort(std::vector<Point>& points);
};

#endif /* POINT_H */

/**
 * virtual functions
 * see:
 * https://www.geeksforgeeks.org/virtual-function-cpp/
 * date: 2020-12-10 17:11
 */

/**
 * polymorphism
 * see:
 * https://stackoverflow.com/questions/7405740/how-can-i-initialize-base-class-member-variables-in-derived-class-constructor
 * see:
 * https://www.ibm.com/support/knowledgecenter/SSLTBW_2.2.0/com.ibm.zos.v2r2.cbclx01/cplr388.htm
 * date: 2020-12-10 17:34
 */
