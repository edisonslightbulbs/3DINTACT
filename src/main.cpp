#include <mutex>
#include <thread>
#include <vector>

#include "kinect.h"
#include "intact.h"

extern const int FAIL = -3;
extern const int PASS = 0;

void work(Kinect& kinect)
{
    /** TODO: */
    std::mutex m;

    Point maxConstraint;
    Point minConstraint;
    std::pair<Point, Point> threshold(minConstraint, maxConstraint);
    const int KINECT_RESOLUTION = 640 * 576;

    /** shared pointers */
    auto sptr_points = std::make_shared<std::vector<float>>(KINECT_RESOLUTION * 3);
    auto sptr_threshold = std::make_shared<std::pair<Point, Point>>(threshold);

    /** segment in separate worker thread */
    std::thread segmenting(intact::segment, std::ref(m), std::ref(kinect),
                          std::ref(KINECT_RESOLUTION), std::ref(sptr_points), std::ref(sptr_threshold));

    /** render in separate worker thread */
    std::thread rendering(intact::render, std::ref(m), std::ref(kinect),
                          std::ref(KINECT_RESOLUTION), std::ref(sptr_points), std::ref(sptr_threshold));

    /** manage worker threads */
    segmenting.join();
    rendering.join();

    /** release and close kinect */
    kinect.release();
    kinect.close();
}

int main(int argc, char* argv[])
{
    logger(argc, argv);
    Kinect kinect;
    work(kinect);
    return PASS;
}