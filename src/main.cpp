#include <string>
#include <thread>

#include "intact.h"
#include "io.h"
#include "kinect.h"
#include "logger.h"

extern const int FAIL = -3;
extern const int PASS = 0;

std::shared_ptr<bool> RUN_SYSTEM;

void work(std::shared_ptr<Kinect> sptr_kinect)
{
    /** segment in separate worker thread */
    std::thread segmenting(intact::segmentContext, std::ref(sptr_kinect));

    /** render in separate worker thread */
    std::thread rendering(intact::render, std::ref(sptr_kinect));

    /** manage worker threads */
    segmenting.join();
    rendering.join();
}

int main(int argc, char* argv[])
{
    /** start system */
    RUN_SYSTEM = std::make_shared<bool>(true);
    std::cout << "Press ESC to exit." << std::endl;
    logger(argc, argv);

    /** start kinect */
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    /** grab image of scene */
    // io::write(sptr_kinect->m_rgbImage);

    /** do multi-threaded work*/
    work(sptr_kinect);

    return PASS;
}
