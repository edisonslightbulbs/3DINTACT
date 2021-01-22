#ifndef POINT3D_H
#define POINT3D_H

#include "point.h"

class Point3d : public Point {

public:
    Point3d(float t_x, float t_y, float t_z)
        : Point(t_x, t_y, t_z)
    {
    }

    float distance(Point* ptr_point3d) override
    {
        return (ptr_point3d->m_x - m_x) * (ptr_point3d->m_x - m_x)
            + (ptr_point3d->m_y - m_y) * (ptr_point3d->m_y - m_y)
            + (ptr_point3d->m_z - m_z) * (ptr_point3d->m_y - m_z);
    }
};
#endif /* POINT3D_H */
