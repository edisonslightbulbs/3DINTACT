#include <string>
#include <thread>

#include "intact.h"
#include "io.h"
#include "kinect.h"
#include "logger.h"

std::shared_ptr<bool> RUN_SYSTEM;

void segment(
    std::shared_ptr<Kinect>& sptr_kinect, std::shared_ptr<Intact>& sptr_intact)
{
    sptr_intact->segment(sptr_kinect, sptr_intact);
}

void render(
    std::shared_ptr<Kinect>& sptr_kinect, std::shared_ptr<Intact>& sptr_intact)
{
    sptr_intact->render(sptr_kinect, sptr_intact);
}

void estimate(std::shared_ptr<Intact>& sptr_intact)
{
    const int K = 5; // <- kth nearest neighbour [ core + 4 nn ]
    sptr_intact->estimateEpsilon(K, sptr_intact);
}

void cluster(
    std::shared_ptr<Kinect>& sptr_kinect, std::shared_ptr<Intact>& sptr_intact)
{
    const float E = 3.317; // <- epsilon
    const int N = 4;       // <- min points in epsilon neighbourhood
    sptr_intact->cluster(E, N, sptr_intact);
}

int main(int argc, char* argv[])
{
    /** start system */
    RUN_SYSTEM = std::make_shared<bool>(true);
    std::cout << "Press ESC to exit." << std::endl;
    logger(argc, argv);

    /** start kinect */
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    /** initialize 3DINTACT */
    std::shared_ptr<Intact> sptr_intact(new Intact(sptr_kinect->m_numPoints));

    /** segment in separate worker thread */
    std::thread segmentWorker(
        segment, std::ref(sptr_kinect), std::ref(sptr_intact));

    /** render in separate worker thread */
    std::thread renderWorker(
        render, std::ref(sptr_kinect), std::ref(sptr_intact));

    /** determine epsilon hyper-parameter */
    std::thread epsilonWorker(estimate, std::ref(sptr_intact));

    /** cluster interaction context  */
    std::thread clusterWorker(
        cluster, std::ref(sptr_kinect), std::ref(sptr_intact));

    /** join worker threads */
    segmentWorker.join();
    renderWorker.join();
    epsilonWorker.join();
    clusterWorker.join();

    /** grab image of scene */
    // io::write(sptr_kinect->m_rgbImage);

    /** release resources */
    sptr_kinect->release();
    sptr_kinect->close();
    return 0;
}
