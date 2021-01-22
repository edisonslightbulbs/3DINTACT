#include "point.h"

extern const int NOISE = -2;
extern const int UNASSIGNED = -1;
extern const int UNCLASSIFIED = -1;

Point::Point()
    : m_x(0.0)
    , m_y(0.0)
    , m_z(0.0)
    , m_id(UNASSIGNED)
    , m_cluster(UNCLASSIFIED)
    , m_distance(0, __DBL_MAX__)
{
}

Point::Point(float t_x, float t_y, float t_z)
    : m_x(t_x)
    , m_y(t_y)
    , m_z(t_z)
    , m_id(UNASSIGNED)
    , m_cluster(UNCLASSIFIED)
    , m_distance(0, __DBL_MAX__)
{
}

bool Point::unclassified() const
{
    return (m_cluster == UNCLASSIFIED || m_cluster == NOISE);
}

float Point::distance(Point point) const
{
    float x = m_x - point.m_x;
    float y = m_y - point.m_y;
    float z = m_z - point.m_z;
    return (float)std::sqrt((x * x) + (y * y) + (z * z));
}

Point Point::centroid(std::vector<Point>& t_points)
{
    /** vec as mat */
    int cols = 3;
    Eigen::MatrixXd pointsMat(t_points.size(), cols);

    int row = 0;
    for (auto point : t_points) {
        pointsMat(row, 0) = point.m_x;
        pointsMat(row, 1) = point.m_y;
        pointsMat(row, 2) = point.m_z;
        row++;
    }

    return Point { (float)pointsMat.col(0).mean(),
        (float)pointsMat.col(1).mean(), (float)pointsMat.col(2).mean() };
}

bool Point::compare(const Point& t_point, const Point& t_other)
{
    return t_point.m_distance.second < t_other.m_distance.second;
}

bool Point::operator==(const Point& rhs) const
{
    return (m_x == rhs.m_x && m_y == rhs.m_y && m_z == rhs.m_z);
}

std::ostream& operator<<(std::ostream& t_stream, const Point& point)
{
    t_stream << point.m_x << " " << point.m_y << " " << point.m_z;
    return t_stream;
}

std::istream& operator>>(std::istream& t_stream, Point& point)
{
    t_stream >> point.m_x >> point.m_y >> point.m_z;
    return t_stream;
}
