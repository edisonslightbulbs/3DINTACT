#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <torch/script.h>

#include "helpers.h"
#include "i3d.h"
#include "io.h"
#include "kinect.h"
#include "macros.hpp"

std::mutex m;

void render(std::shared_ptr<i3d>& sptr_i3d)
{
    sptr_i3d->renderRegion(sptr_i3d);
}

void detect(std::shared_ptr<i3d>& sptr_intact)
{
    std::vector<std::string> classNames;
    torch::jit::script::Module module;
    utils::configTorch(classNames, module);
    sptr_intact->findRegionObjects(classNames, module, sptr_intact);
}

void findRegion(std::shared_ptr<i3d>& sptr_i3d)
{
    sptr_i3d->proposeRegion(sptr_i3d);
}

void segment(std::shared_ptr<i3d>& sptr_i3d)
{
    sptr_i3d->segmentRegion(sptr_i3d);
}

void frame(std::shared_ptr<i3d>& sptr_i3d) { sptr_i3d->frameRegion(sptr_i3d); }

void chromakey(std::shared_ptr<i3d>& sptr_i3d)
{
    sptr_i3d->chromakey(sptr_i3d);
}

void cluster(std::shared_ptr<i3d>& sptr_i3d)
{
    int minPoints = 4;
    const float epsilon = 3.170;
    sptr_i3d->clusterRegion(epsilon, minPoints, sptr_i3d);
}

void buildPcl(std::shared_ptr<i3d>& sptr_i3d)
{
    sptr_i3d->buildPCloud(sptr_i3d);
}

void k4aCapture(
    std::shared_ptr<Kinect>& sptr_kinect, std::shared_ptr<i3d>& sptr_i3d)
{
    START
    sptr_kinect->capture();
    sptr_kinect->depthCapture();
    int w = k4a_image_get_width_pixels(sptr_kinect->m_depth);
    int h = k4a_image_get_height_pixels(sptr_kinect->m_depth);

    while (sptr_i3d->isRun()) {
        START_TIMER
        sptr_kinect->capture();
        sptr_kinect->depthCapture();
        sptr_kinect->pclCapture();
        sptr_kinect->imgCapture();
        sptr_kinect->c2dCapture();
        sptr_kinect->transform(RGB_TO_DEPTH);

        auto* ptr_k4aImgData = k4a_image_get_buffer(sptr_kinect->m_c2d);
        auto* ptr_k4aPCloudData
            = (int16_t*)(void*)k4a_image_get_buffer(sptr_kinect->m_pcl);
        auto* ptr_k4aDepthData
            = (uint16_t*)(void*)k4a_image_get_buffer(sptr_kinect->m_depth);
        auto* ptr_k4aTableData
            = (k4a_float2_t*)(void*)k4a_image_get_buffer(sptr_kinect->m_xyT);

        // share k4a resources with intact
        sptr_i3d->setDepthWidth(w);
        sptr_i3d->setDepthHeight(h);
        sptr_i3d->setSensorImgData(ptr_k4aImgData);
        sptr_i3d->setSensorTableData(ptr_k4aTableData);
        sptr_i3d->setSensorDepthData(ptr_k4aDepthData);
        sptr_i3d->setSensorPCloudData(ptr_k4aPCloudData);

        // release k4a resources
        sptr_kinect->releaseK4aImages();
        sptr_kinect->releaseK4aCapture();
        RAISE_SENSOR_RESOURCES_READY_FLAG
        POLL_EXIT_STATUS
        STOP_TIMER(" main driver thread: ")
    }
}

int main(int argc, char* argv[])
{
    logger(argc, argv);
    LOG(INFO) << "-- 3DINTACT is currently unstable and should only be used "
                 "for academic purposes!";
    LOG(INFO) << "-- press ESC to exit";

    // initialize kinect
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    // initialize the 3dintact API
    std::shared_ptr<i3d> sptr_i3d(new i3d());
    sptr_i3d->raiseRunFlag();

    // capture using depth sensor
    std::thread k4aCaptureWorker(
        k4aCapture, std::ref(sptr_kinect), std::ref(sptr_i3d));

    // build point cloud
    std::thread buildPCloudWorker(buildPcl, std::ref(sptr_i3d));

    // find region of interest
    std::thread findRegionWorker(findRegion, std::ref(sptr_i3d));

    // create GL and CV specific frames
    std::thread frameRegionWorker(frame, std::ref(sptr_i3d));

    // segment
    std::thread segmentRegionWorker(segment, std::ref(sptr_i3d));

    // render
    std::thread renderRegionWorker(render, std::ref(sptr_i3d));

    // find objects
    std::thread findRegionObjectsWorker(detect, std::ref(sptr_i3d));

    // cluster
    std::thread clusterRegionWorker(cluster, std::ref(sptr_i3d));

    /* example: chroma-keying the tabletop surface */
    std::thread chromakeyWorker(chromakey, std::ref(sptr_i3d));

    k4aCaptureWorker.join();
    buildPCloudWorker.join();
    renderRegionWorker.join();
    findRegionObjectsWorker.join();
    findRegionWorker.join();
    segmentRegionWorker.join();
    frameRegionWorker.join();
    clusterRegionWorker.join();
    chromakeyWorker.join();

    // ------> do stuff with tabletop environment <------

    // ------> do stuff with tabletop environment <------

    return 0;
}
