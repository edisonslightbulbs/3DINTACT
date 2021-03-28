#ifndef WORKERS_H
#define WORKERS_H

#include <exception>
#include <iostream>
#include <k4a/k4a.h>
#include <mutex>
#include <thread>
#include <vector>

#include "io.h"
#include "kinect.h"
#include "logger.h"
#include "point.h"
#include "renderer.h"
#include "segment.h"
#include "timer.h"

namespace workers {

/** In theory this thread will execute every 30 ms to assess the tabletop
 *  context for down stream processing.
 */
void context(Kinect kinect)
{
    std::vector<Point> context = segment::cut(kinect.m_points);

    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;

    for (const auto& point : kinect.m_points) {
        x.push_back(point.m_x);
        y.push_back(point.m_y);
        z.push_back(point.m_z);
    }
    float xMax = *std::max_element(x.begin(), x.end());
    float xMin = *std::min_element(x.begin(), x.end());
    float yMax = *std::max_element(y.begin(), y.end());
    float yMin = *std::min_element(y.begin(), y.end());
    float zMax = *std::max_element(z.begin(), z.end());
    float zMin = *std::min_element(z.begin(), z.end());

    Point maxPoint(xMax, yMax, zMax);
    Point minPoint(xMin, yMin, zMin);

    io::ply(kinect.m_points, context);
}

void manage(const Kinect& kinect)
{
    /** MUTEX NOT IN USE ... YET */
    std::mutex m;

    int pointNum = 640 * 576;
    auto sptr_points = std::make_shared<std::vector<float>>(pointNum * 3);

    /** preprocess in separate thread */
    std::thread preprocessing(context, std::ref(kinect));

    /** render in separate thread */
    std::thread rendering(renderer::render, std::ref(m), std::ref(kinect),
        std::ref(pointNum), std::ref(sptr_points));

    /** manage thread handles */
    preprocessing.join();
    rendering.join();

    /** release and close kinect */
    kinect.release();
    kinect.close();
}
}
#endif /* WORKERS_H */
