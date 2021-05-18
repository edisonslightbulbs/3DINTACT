#include <chrono>
#include <string>
#include <thread>
#include <torch/script.h>

#include "intact.h"
#include "io.h"
#include "kinect.h"
#include "logger.h"
#include "utility.hpp"

// main loop for getting pcl and rgba image data
void daq(
    std::shared_ptr<Kinect>& sptr_kinect, std::shared_ptr<Intact>& sptr_intact)
{
    int size = sptr_intact->getNumPoints() * 3;
    int imgSize = sptr_intact->getNumPoints() * 4; // r, g, b, a
    int pclSize = sptr_intact->getNumPoints() * 3; // x, y, z
    auto* ptr_segmentedImgData = (uint8_t*)malloc(sizeof(uint8_t) * imgSize);
    auto* ptr_segmentedPclData = (int16_t*)malloc(sizeof(int16_t) * pclSize);

    bool init = true;
    while (sptr_intact->isRun()) {
        /** capture */
        sptr_kinect->getFrame(RGB_TO_DEPTH);

        /** get raw data */
        int width = k4a_image_get_width_pixels(sptr_kinect->getDepthImg());
        int height = k4a_image_get_height_pixels(sptr_kinect->getDepthImg());

        /** std structures for arithmetic computations */
        std::vector<float> pclVec(size);            // raw pcl
        std::vector<uint8_t> imgVec(size);          // raw img
        std::vector<float> segmentedPclVec(size);   // segmented pcl
        std::vector<uint8_t> segmentedImgVec(size); // segmented img

        auto* ptr_pclData
            = (int16_t*)(void*)k4a_image_get_buffer(sptr_kinect->getPcl());
        auto* ptr_imgData
            = k4a_image_get_buffer(sptr_kinect->getRgb2DepthImg());

        /** assert valid data */
        for (int i = 0; i < sptr_intact->getNumPoints(); i++) {
            if (ptr_pclData[3 * i + 2] == 0) {
                zeroPoint(i, pclVec);
                zeroPixel(i, imgVec);
                zeroPointData(i, ptr_pclData);
                zeroPixelData(i, ptr_imgData);
                continue;
            }
            getPoint(i, pclVec, ptr_pclData);
            getPixel(i, imgVec, ptr_imgData);

            /** segment tabletop surface */
            if (outsideSegment(i, ptr_pclData,
                    sptr_intact->getSegmentBoundary().first,
                    sptr_intact->getSegmentBoundary().second)) {
                zeroPointData(i, ptr_pclData);
                zeroPixelData(i, ptr_imgData);
                continue;
            }
            getPoint(i, segmentedPclVec, ptr_pclData);
            getPixel(i, segmentedImgVec, ptr_imgData);
        }
        /** create image for segmented tabletop in cv::Mat format */
        cv::Mat frame = cv::Mat(
            height, width, CV_8UC4, (void*)ptr_imgData, cv::Mat::AUTO_STEP)
                            .clone();

        /** pass raw data to API */
        sptr_intact->setPclVec(pclVec);
        sptr_intact->setImgVec(imgVec);
        sptr_intact->setDepthImgWidth(width);
        sptr_intact->setDepthImgHeight(height);
        sptr_intact->setSegmentedImgFrame(frame);
        sptr_intact->setSegmentedImgData(
            ptr_segmentedImgData, ptr_imgData, imgSize);
        sptr_intact->setSegmentedPclData(
            ptr_segmentedPclData, ptr_pclData, pclSize);

        /** update tabletop segment */
        updateSegment(
            sptr_intact, pclVec, imgVec, segmentedPclVec, segmentedImgVec);

        /** release kinect resources */
        sptr_kinect->release();

        /** update flow-control semaphore */
        if (init) {
            init = false;
            sptr_intact->raiseKinectReadyFlag();
        }
        if (sptr_intact->isStop()) {
            sptr_intact->stop();
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
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

void chromakey(std::shared_ptr<Intact>& sptr_intact)
{
    sptr_intact->chroma(sptr_intact);
}

void detect(std::shared_ptr<Intact>& sptr_intact)
{
    /** configure torch  */
    std::vector<std::string> classNames;
    torch::jit::script::Module module;
    configTorch(classNames, module);

    /** detect objects */
    sptr_intact->detectObjects(classNames, module, sptr_intact);
}

int main(int argc, char* argv[])
{
    std::cout << "-- press ESC to exit." << std::endl;
    logger(argc, argv);

    /** initialize kinect */
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    /** initialize API */
    std::shared_ptr<Intact> sptr_intact(new Intact(sptr_kinect->m_numPoints));
    sptr_intact->raiseRunFlag();

    /** get image and point cloud data */
    std::thread dataAcquisitionWorker(
        daq, std::ref(sptr_kinect), std::ref(sptr_intact));

    /** calibrate */
    std::thread calibrationWorker(calibrate, std::ref(sptr_intact));

    /** segment */
    std::thread segmentationWorker(segment, std::ref(sptr_intact));

    /** render */
    std::thread renderingWorker(render, std::ref(sptr_intact));

    /** estimate epsilon */
    std::thread epsilonWorker(estimate, std::ref(sptr_intact));

    /** cluster */
    std::thread clusteringWorker(cluster, std::ref(sptr_intact));

    /** chromakey */
    std::thread chromakeyWorker(chromakey, std::ref(sptr_intact));

    /** detect objects */
    std::thread detectionWorker(detect, std::ref(sptr_intact));

    // ------> do stuff with raw point cloud and segment <------
    sptr_intact->getPcl();          // raw point cloud
    sptr_intact->getImg();          // raw rgb image
    sptr_intact->getSegmentedPcl(); // segmented point cloud
    sptr_intact->getSegmentedImg(); // segmented rgb image
    // ------> do stuff with raw point cloud and segment <------

    dataAcquisitionWorker.join();
    calibrationWorker.join();
    segmentationWorker.join();
    renderingWorker.join();
    epsilonWorker.join();
    clusteringWorker.join();
    detectionWorker.join();
    chromakeyWorker.join();

    return 0;
}
