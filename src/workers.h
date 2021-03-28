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
#include "timer.h"

namespace workers {


    void manage(Kinect& kinect)
    {
        /** MUTEX NOT IN USE ... YET */
        std::mutex m;

        const int KINECT_RESOLUTION = 640 * 576;
        auto sptr_points = std::make_shared<std::vector<float>>(KINECT_RESOLUTION * 3);

        /** render in separate worker thread */
        std::thread rendering(renderer::render, std::ref(m), std::ref(kinect),
            std::ref(KINECT_RESOLUTION), std::ref(sptr_points));

        /** manage worker threads */
        rendering.join();

        /** release and close kinect */
        kinect.release();
        kinect.close();
    }
}
#endif /* WORKERS_H */
