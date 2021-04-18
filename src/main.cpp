#include <string>
#include <thread>

#include "intact.h"
#include "io.h"
#include "kinect.h"
#include "logger.h"

std::shared_ptr<bool> RUN_SYSTEM;

void work(std::shared_ptr<Kinect> sptr_kinect)
{
    /** segment in separate worker thread */
    std::thread segmentingWorker(intact::segmentContext, std::ref(sptr_kinect));

    /** render in separate worker thread */
    std::thread renderingWorker(intact::render, std::ref(sptr_kinect));

    // todo: introduce worker for detecting epsilon value autonomously
    // std::thread parameterizingWorker(intact::render, std::ref(sptr_kinect));

    /** cluster interaction context  */
    const float epsilon = 3.372;
    std::thread clusteringWorker(intact::cluster, epsilon);

    /** manage worker threads */
    segmentingWorker.join();
    renderingWorker.join();
    clusteringWorker.join();
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

    sptr_kinect->release();
    sptr_kinect->close();

    return 0;
}
