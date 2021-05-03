#include <chrono>
#include <string>
#include <thread>

#include "intact.h"
#include "io.h"
#include "kinect.h"
#include "logger.h"

void sense(
    std::shared_ptr<Kinect>& sptr_kinect, std::shared_ptr<Intact>& sptr_intact)
{
    bool init = true;
    while (sptr_intact->isRun()) {
        /** get next frame: */
        sptr_kinect->getFrame(RGB_TO_DEPTH);
        sptr_intact->buildPcl(
            sptr_kinect->getPclImage(), sptr_kinect->getRgb2DepthImage());
        sptr_kinect->release();

        /** inform threads: init frame processing done */
        if (init) {
            init = false;
            sptr_intact->raiseKinectReadyFlag();
        }
        if (sptr_intact->isStop()) {
            sptr_intact->stop();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void calibrate(std::shared_ptr<Intact>& sptr_intact)
{
    sptr_intact->calibrate(sptr_intact);
}

void segment(std::shared_ptr<Intact>& sptr_intact)
{
    sptr_intact->segment(sptr_intact);
}

void render(std::shared_ptr<Intact>& sptr_intact)
{
    sptr_intact->render(sptr_intact);
}

void estimate(std::shared_ptr<Intact>& sptr_intact)
{
    const int K = 5; // <- kth nearest neighbour [ core + 4 nn ]
    sptr_intact->estimateEpsilon(K, sptr_intact);
}

void cluster(std::shared_ptr<Intact>& sptr_intact)
{
    const float E = 3.310; // <- epsilon
    const int N = 4;       // <- min points in epsilon neighbourhood
    sptr_intact->cluster(E, N, sptr_intact);
}

int main(int argc, char* argv[])
{
    std::cout << "Press ESC to exit." << std::endl;
    logger(argc, argv);

    /** initialize kinect */
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    /** initialize API */
    std::shared_ptr<Intact> sptr_intact(new Intact(sptr_kinect->m_numPoints));
    sptr_intact->raiseRunFlag();

    /** start sensing */
    std::thread senseWorker(sense, std::ref(sptr_kinect), std::ref(sptr_intact));
    // Here is the interesting bit:
    // Your sensor of choice (which needn't be the Kinect) should
    // handshake 3DINTACT's API. No adaptors are provided
    // casting different point cloud data structures into the
    // structure required by the API. This is something that
    // has to be done manually for each specific point-cloud
    // source. The only important thing to note is point-cloud
    // data input to the API comprises of two vectors,
    //   i) std::vector<float> points;
    //      where points =
    //      { x_0, y_0, z_0, x_1, y_1, z_1, ... }
    //  ii) std::vector<uint8_t> color;
    //      where color =
    //      { r_0, g_0, b_0, r_1, g_1, b_1, ... }


    /** sense */
    std::thread calibrateWorker(calibrate, std::ref(sptr_intact));

    /** segment */
    std::thread segmentWorker(segment, std::ref(sptr_intact));

    /** render */
    std::thread renderWorker(render, std::ref(sptr_intact));

    /** find epsilon */
    std::thread epsilonWorker(estimate, std::ref(sptr_intact));

    /** cluster */
    std::thread clusterWorker(cluster, std::ref(sptr_intact));

    senseWorker.join();
    segmentWorker.join();
    renderWorker.join();
    epsilonWorker.join();
    clusterWorker.join();
    calibrateWorker.join();

    /** grab image of scene */
    // io::write(sptr_kinect->m_rgbImage);
    return 0;
}
