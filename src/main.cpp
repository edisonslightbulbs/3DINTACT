#include <chrono>
#include <string>
#include <thread>
#include <torch/script.h>

#include "helpers.hpp"
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
    configTorch(classNames, module);
    sptr_intact->showObjects(classNames, module, sptr_intact);
}

void segment(std::shared_ptr<Intact>& sptr_intact)
{
    sptr_intact->segment(sptr_intact);
}

void cluster(std::shared_ptr<Intact>& sptr_intact)
{
    int minPoints = 4;
    const float epsilon = 3.170;
    sptr_intact->cluster(epsilon, minPoints, sptr_intact);
}

void siftSegment(std::shared_ptr<Intact>& sptr_intact)
{
    int numPts = sptr_intact->m_numPts;
    int pclsize = sptr_intact->m_pclsize;
    int imgsize = sptr_intact->m_imgsize;

    int16_t pclBuf[pclsize];
    uint8_t imgBuf_GL[pclsize];
    uint8_t imgBuf_CV[imgsize];

    WHILE_SEGMENT_READY
    START
    while (sptr_intact->isRun()) {
        auto* ptr_pcl = *sptr_intact->getSensorPcl();
        auto* ptr_img = *sptr_intact->getSensorImg_CV();

        std::vector<Point> pcl;

        for (int i = 0; i < numPts; i++) {
            Point point;
            if (isZero(i, ptr_pcl, ptr_img)) {
                addPoint(i, ptr_pcl, pclBuf);
                addPixel_GL(i, ptr_img, imgBuf_GL);
                addPixel_CV(i, ptr_img, imgBuf_CV);
                adapt(i, point, pclBuf, imgBuf_CV);
                pcl.emplace_back(point);
                continue;
            }
            if (!inSegment(i, ptr_pcl, sptr_intact->getIntactBoundary().first,
                    sptr_intact->getIntactBoundary().second)) {
                zeroPoint(i, pclBuf);
                zeroPixel_GL(i, imgBuf_GL);
                zeroPixel_CV(i, imgBuf_CV);
                adapt(i, point, pclBuf, imgBuf_CV);
                pcl.emplace_back(point);
                continue;
            }
            addPoint(i, ptr_pcl, pclBuf);
            addPixel_GL(i, ptr_img, imgBuf_GL);
            addPixel_CV(i, ptr_img, imgBuf_CV);
            adapt(i, point, pclBuf, imgBuf_CV);
            pcl.emplace_back(point);
        }

        // sptr_intact->setIntactPts(pcl);
        sptr_intact->setIntactPcl(pclBuf);
        sptr_intact->setIntactImg_GL(imgBuf_GL);
        sptr_intact->setIntactImg_CV(imgBuf_CV);

        INTACT_READY
        POLLING_EXIT_STATUS
    }
}

void k4aCapture(
    std::shared_ptr<Kinect>& sptr_kinect, std::shared_ptr<Intact>& sptr_intact)
{
    int numPts = sptr_intact->m_numPts;
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

        std::vector<Point> pcl;

        for (int i = 0; i < numPts; i++) {
            Point point;
            if (ptr_pcl[3 * i + 2] == 0) {
                zeroPoint(i, pclBuf);
                zeroPixel_GL(i, imgBuf_GL);
                zeroPixel_CV(i, imgBuf_CV);
                continue;
            }
            addPoint(i, ptr_pcl, pclBuf);
            addPixel_GL(i, ptr_img, imgBuf_GL);
            addPixel_CV(i, ptr_img, imgBuf_CV);
            adapt(i, point, pclBuf, imgBuf_CV);
            pcl.emplace_back(point);
        }
        sptr_kinect->release();
        sptr_intact->setSensorPts(pcl);
        sptr_intact->setDepthImgWidth(width);
        sptr_intact->setDepthImgHeight(height);
        sptr_intact->setSensorPcl(pclBuf);
        sptr_intact->setSensorImg_GL(imgBuf_GL);
        sptr_intact->setSensorImg_CV(imgBuf_CV);
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
    // std::thread detectWorker(detect, std::ref(sptr_intact));

    // segment
    std::thread segmentWorker(segment, std::ref(sptr_intact));

    // sift segment
    std::thread siftSegmentWorker(siftSegment, std::ref(sptr_intact));

    // cluster
    std::thread clusterWorker(cluster, std::ref(sptr_intact));

    k4aCaptureWorker.join();
    renderWorker.join();
    // detectWorker.join();
    segmentWorker.join();
    siftSegmentWorker.join();
    clusterWorker.join();

    // ------> do stuff with tabletop environment <------

    // ------> do stuff with tabletop environment <------

    return 0;
}
