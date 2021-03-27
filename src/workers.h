#ifndef WORKERS_H
#define WORKERS_H

#include <atomic>
#include <iostream>
#include <k4a/k4a.h>
#include <mutex>
#include <thread>
#include <vector>

#include "io.h"
#include "kinect.h"
#include "point.h"
#include "renderer.h"
#include "segment.h"

std::atomic<bool> stop(false);

namespace workers {

/** thread 3: render points cloud in real-time */

/** thread 2: process point cloud segment */
    void process(std::mutex& m, Kinect kinect,
                 std::shared_ptr<std::vector<float>>& sptr_points)
    {
        while(!stop)
            {
                if (m.try_lock()) {
                    kinect.getPointCloud(sptr_points);
                    m.unlock();
                }
                usleep(10000);
            }
    }

/** thread 1: segment interaction context */
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

/** thread 0:  manage thread work */
    void manage(const Kinect& kinect)
    {
        std::mutex m;

        int pointNum = 640 * 576;
        auto sptr_points = std::make_shared<std::vector<float>>(pointNum * 3);

        std::thread preprocessing(context, std::ref(kinect));
        std::thread processing(
                process, std::ref(m), std::ref(kinect), std::ref(sptr_points));
             std::thread rendering(renderer::render, std::ref(m),
             std::ref(pointNum),
                 std::ref(sptr_points));


        preprocessing.join();
        processing.join();
        rendering.join();

        std::cin.get();
        stop = true;

        kinect.release();
        kinect.close();
    }
}
#endif /* WORKERS_H */
