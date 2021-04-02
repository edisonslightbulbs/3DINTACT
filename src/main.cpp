#include <thread>

#include "kinect.h"
#include "intact.h"

extern const int FAIL = -3;
extern const int PASS = 0;

void work(Kinect& kinect)
{
    /** segment in separate worker thread */
    std::thread segmenting(intact::segment, std::ref(kinect));

    /** render in separate worker thread */
    std::thread rendering(intact::render, std::ref(kinect));

    /** manage worker threads */
    segmenting.join();
    rendering.join();
}

int main(int argc, char* argv[])
{
    logger(argc, argv);
    Kinect kinect;
    work(kinect);
    return PASS;
}