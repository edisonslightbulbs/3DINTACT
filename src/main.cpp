#include <chrono>
#include <string>
#include <thread>
#include <torch/script.h>

#include "i3d.h"
#include "intact.h"
#include "io.h"
#include "kinect.h"
#include "logger.h"
#include "macros.hpp"

void render(std::shared_ptr<Intact>& sptr_intact)
{
    sptr_intact->render(sptr_intact);
}

void detect(std::shared_ptr<Intact>& sptr_intact)
{
    std::vector<std::string> classNames;
    torch::jit::script::Module module;
    i3d::configTorch(classNames, module);
    sptr_intact->showObjects(classNames, module, sptr_intact);
}

void segment(std::shared_ptr<Intact>& sptr_intact)
{
    sptr_intact->segment(sptr_intact);
}

void sift(std::shared_ptr<Intact>& sptr_intact)
{
    sptr_intact->sift(sptr_intact);
}

void chromakey(std::shared_ptr<Intact>& sptr_intact)
{
    sptr_intact->chromakey(sptr_intact);
}

void cluster(std::shared_ptr<Intact>& sptr_intact)
{
    int minPoints = 4;
    const float epsilon = 3.170;
    sptr_intact->cluster(epsilon, minPoints, sptr_intact);
}

void k4aCapture(
    std::shared_ptr<Kinect>& sptr_kinect, std::shared_ptr<Intact>& sptr_intact)
{
    int numPts = sptr_intact->m_numPoints;
    int pclsize = sptr_intact->m_pclsize;
    int imgsize = sptr_intact->m_imgsize;

    int16_t pclBuf[pclsize];
    uint8_t imgBuf_GL[pclsize];
    uint8_t imgBuf_CV[imgsize];

    START
    while (sptr_intact->isRun()) {
        sptr_kinect->getFrame(RGB_TO_DEPTH);

        auto* ptr_pcl
            = (int16_t*)(void*)k4a_image_get_buffer(sptr_kinect->getPcl());
        auto* ptr_img = k4a_image_get_buffer(sptr_kinect->getRgb2DepthImg());
        int width = k4a_image_get_width_pixels(sptr_kinect->getDepthImg());
        int height = k4a_image_get_height_pixels(sptr_kinect->getDepthImg());

        std::vector<Point> refinedPoints;
        std::vector<Point> unrefinedPoints;
        for (int i = 0; i < numPts; i++) {
            Point point;
            i3d::addPoint(i, pclBuf, ptr_pcl);
            i3d::addPixel_GL(i, imgBuf_GL, ptr_img);
            i3d::addPixel_CV(i, imgBuf_CV, ptr_img);
            i3d::adapt(i, point, pclBuf, imgBuf_CV);
            unrefinedPoints.emplace_back(point);

            if (ptr_pcl[3 * i + 2] == 0) {
                continue;
            }
            refinedPoints.emplace_back(point);
        }
        sptr_kinect->release();
        sptr_intact->setDepthImgWidth(width);
        sptr_intact->setDepthImgHeight(height);
        sptr_intact->setSensorPcl(pclBuf);
        sptr_intact->setSensorImg_GL(imgBuf_GL);
        sptr_intact->setSensorImg_CV(imgBuf_CV);
        sptr_intact->setRefinedPoints(refinedPoints);
        sptr_intact->setUnrefinedPoints(unrefinedPoints);
        KINECT_READY
        POLLING_EXIT_STATUS
    }
}

int main(int argc, char* argv[])
{
    std::cout << "-- press ESC to exit." << std::endl;
    logger(argc, argv);

    // initialize kinect
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    // initialize the 3dintact API
    std::shared_ptr<Intact> sptr_intact(new Intact(sptr_kinect->m_numPts));
    sptr_intact->raiseRunFlag();

    // capture using depth sensor
    std::thread k4aCaptureWorker(
        k4aCapture, std::ref(sptr_kinect), std::ref(sptr_intact));

    // render
    std::thread renderWorker(render, std::ref(sptr_intact));

    // detect objects
    std::thread detectWorker(detect, std::ref(sptr_intact));

    // segment
    std::thread segmentWorker(segment, std::ref(sptr_intact));

    // sift segment
    std::thread siftWorker(sift, std::ref(sptr_intact));

    // cluster
    std::thread clusterWorker(cluster, std::ref(sptr_intact));

    /* example: chroma-keying the tabletop surface */
    std::thread chromakeyWorker(chromakey, std::ref(sptr_intact));

    k4aCaptureWorker.join();
    renderWorker.join();
    detectWorker.join();
    segmentWorker.join();
    siftWorker.join();
    clusterWorker.join();
    chromakeyWorker.join();

    // ------> do stuff with tabletop environment <------

    // ------> do stuff with tabletop environment <------

    return 0;
}
