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
    int pclSize = sptr_intact->getNumPoints() * 3; // x, y, z
    int imgSize = sptr_intact->getNumPoints() * 4; // r, g, b, a
    // auto* ptr_pclData = (int16_t*)malloc(sizeof(int16_t) * pclSize);
    // auto* ptr_imgData = (uint8_t*)malloc(sizeof(uint8_t) * imgSize);
    // auto* ptr_segmentedImgData = (uint8_t*)malloc(sizeof(uint8_t) * imgSize);

    bool init = true;
    while (sptr_intact->isRun()) {
        /** capture */
        sptr_kinect->getFrame(RGB_TO_DEPTH);

        /** get raw data */
        int width = k4a_image_get_width_pixels(sptr_kinect->getDepthImg());
        int height = k4a_image_get_height_pixels(sptr_kinect->getDepthImg());

        // std::memcpy(ptr_pclData,
        // (int16_t*)(void*)k4a_image_get_buffer(sptr_kinect->getPcl()),
        // sizeof(uint16_t) * pclSize); std::memcpy(ptr_imgData,
        // k4a_image_get_buffer(sptr_kinect->getRgb2DepthImg()), sizeof(uint8_t)
        // * imgSize); std::memcpy(ptr_segmentedImgData,
        // k4a_image_get_buffer(sptr_kinect->getRgb2DepthImg()), sizeof(uint8_t)
        // * imgSize);

        /** std structures for arithmetic computations */
        std::vector<float> pcl(pclSize);   // raw point-cloud vector
        std::vector<uint8_t> img(pclSize); // raw image vector
        std::vector<float> segmentedPcl(
            pclSize); // segmented point-cloud vector
        std::vector<uint8_t> segmentedImg(pclSize); // segmented image vector

        auto* ptr_pclData
            = (int16_t*)(void*)k4a_image_get_buffer(sptr_kinect->getPcl());
        auto* ptr_imgData
            = k4a_image_get_buffer(sptr_kinect->getRgb2DepthImg());
        auto* ptr_segmentedImgData
            = k4a_image_get_buffer(sptr_kinect->getRgb2DepthImg());

        /** populate data structures */
        for (int i = 0; i < sptr_intact->getNumPoints(); i++) {
            if (ptr_pclData[3 * i + 2] == 0) {
                zeroPoint(i, pcl);
                zeroPixel(i, img);
                zeroData(i, ptr_segmentedImgData);
                continue;
            }
            getPoint(i, pcl, ptr_pclData);
            getPixel(i, img, ptr_imgData);

            /** filter segment */
            if (outsideSegment(i, ptr_pclData,
                    sptr_intact->getSegmentBoundary().first,
                    sptr_intact->getSegmentBoundary().second)) {
                zeroData(i, ptr_segmentedImgData);
                continue;
            }
            getPoint(i, segmentedPcl, ptr_pclData);
            getPixel(i, segmentedImg, ptr_imgData);
        }

        /** create image for segmented tabletop in cv::Mat format */
        cv::Mat frame = cv::Mat(height, width, CV_8UC4,
            (void*)ptr_segmentedImgData, cv::Mat::AUTO_STEP)
                            .clone();

        /** pass raw data to API */
        sptr_intact->setPcl(pcl);
        sptr_intact->setImg(img);
        sptr_intact->setImgData(ptr_imgData);
        sptr_intact->setPclData(ptr_pclData);
        sptr_intact->setSegmentedImgFrame(frame);
        sptr_intact->setDepthImgWidth(width);
        sptr_intact->setDepthImgHeight(height);

        /** release kinect resources */
        sptr_kinect->release();

        /** update tabletop segment */
        updateSegment(sptr_intact, pcl, img, segmentedPcl, segmentedImg);

        /** update flow-control semaphore */
        if (init) {
            init = false;
            sptr_intact->raiseKinectReadyFlag();
        }
        if (sptr_intact->isStop()) {
            sptr_intact->stop();
        }
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

#define CHROMAKEY 0
void chromakey(std::shared_ptr<Intact>& sptr_intact)
{
#if CHROMAKEY == 1
    bool init = true;
    while (sptr_intact->isRun()) {

        const int N = sptr_intact->getNumPoints();

        // /** initialize segmented point cloud container */
        // std::vector<float> tabletopPcl(N * 3);
        // std::vector<uint8_t> tabletopImg(N * 3);

        /** get point cloud and image data */
        int16_t* tabletopPclData = *sptr_intact->getPclData();
        uint8_t* tabletopImgData = *sptr_intact->getImgData();

        // for (int i = 0; i < N; i++) {
        //     /** get raw point cloud */
        //     if (tabletopPclData[3 * i + 2] == 0) {
        //         //zeroPoint(i, pcl);
        //         //zeroPixel(i, img);
        //         //zeroData(i, tabletopImgData); // todo: [checked]
        //         continue;
        //     }
        //     //getPoint(i, pcl, pclData);
        //     //getPixel(i, img, imgData);

        //     /** get segmented tabletop region */
        //     if (outsideSegment(i, tabletopPclData,
        //                        sptr_intact->getSegmentBoundary().first,
        //                        sptr_intact->getSegmentBoundary().second)) {
        //         //zeroData(i, tabletopImgData); // todo: [checked]
        //         continue;
        //     }
        //     // getPoint(i, segmentedPcl, pclData);
        //     // getPixel(i, segmentedImg, imgData);
        // }

        //   // todo: [checked]
        // uint8
        int w = sptr_intact->getDepthImgWidth();
        int h = sptr_intact->getDepthImgHeight();
        cv::Mat frame
            = cv::Mat(h, w, CV_8UC4, (void*)tabletopImgData, cv::Mat::AUTO_STEP)
                  .clone();
        sptr_intact->setTabletopImgData(frame);

        /** update flow-control semaphore */
        if (init) {
            init = false;
            sptr_intact->raiseChromakeyedFlag();
        }
    }
#endif
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

    /** wait for completion of segmentation task */
    while (!sptr_intact->isSegmented()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }

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
