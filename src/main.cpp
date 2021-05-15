#include <chrono>
#include <string>
#include <thread>
#include <torch/script.h>

#include "intact.h"
#include "io.h"
#include "kinect.h"
#include "logger.h"
#include "utility.hpp"

// image data acquisition
void daq(
    std::shared_ptr<Kinect>& sptr_kinect, std::shared_ptr<Intact>& sptr_intact)
{
    bool init = true;
    while (sptr_intact->isRun()) {

        /** capture point cloud */
        sptr_kinect->getFrame(RGB_TO_DEPTH);
        const int N = sptr_intact->getNumPoints();

        /** initialize raw point cloud container */
        std::vector<float> pcl(N * 3);
        std::vector<uint8_t> img(N * 3);

        /** initialize segmented point cloud container */
        std::vector<float> segmentedPcl(N * 3);
        std::vector<uint8_t> segmentedImg(N * 3);
        // std::vector<uint8_t> segmentedImgData(N * 3);

        /** point cloud data */
        auto* pclData
            = (int16_t*)(void*)k4a_image_get_buffer(sptr_kinect->getPcl());

        /** image data (rgb to depth)*/
        uint8_t* imgData = k4a_image_get_buffer(sptr_kinect->getRgb2DepthImg());
        uint8_t* data = k4a_image_get_buffer(sptr_kinect->getRgb2DepthImg());

        for (int i = 0; i < N; i++) {
            /** get raw point cloud */
            if (pclData[3 * i + 2] == 0) {
                zeroPoint(i, pcl);
                zeroPixel(i, img);
                zeroData(i, data); // todo: [checked]
                continue;
            }
            getPoint(i, pcl, pclData);
            getPixel(i, img, imgData);

            /** get segmented tabletop region */
            if (outsideSegment(i, pclData,
                    sptr_intact->getSegmentBoundary().first,
                    sptr_intact->getSegmentBoundary().second)) {
                zeroData(i, data); // todo: [checked]
                continue;
            }
            getPoint(i, segmentedPcl, pclData);
            getPixel(i, segmentedImg, imgData);
        }

        // todo: [checked]
        int w = k4a_image_get_width_pixels(sptr_kinect->getDepthImg());
        int h = k4a_image_get_height_pixels(sptr_kinect->getDepthImg());
        cv::Mat frame
            = cv::Mat(h, w, CV_8UC4, (void*)data, cv::Mat::AUTO_STEP).clone();
        sptr_intact->setSegmentedImgData(frame);

        /** release kinect resources */
        sptr_kinect->release();

        /** update tabletop segment */
        updateSegment(sptr_intact, pcl, img, segmentedPcl,
            segmentedImg); // todo: [checked]

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

void detect(
    std::shared_ptr<Intact>& sptr_intact, std::shared_ptr<Kinect>& sptr_kinect)
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
    std::cout << "Press ESC to exit." << std::endl;
    logger(argc, argv);

    /** initialize kinect */
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    /** initialize API */
    std::shared_ptr<Intact> sptr_intact(new Intact(sptr_kinect->m_numPoints));
    sptr_intact->raiseRunFlag();

    try {
        /** sense */
        std::thread dataAcquisitionWorker(
            daq, std::ref(sptr_kinect), std::ref(sptr_intact));

        /** calibrate */
        std::thread calibrationWorker(calibrate, std::ref(sptr_intact));

        /** segment */
        std::thread segmentationWorker(segment, std::ref(sptr_intact));

        /** render */
        std::thread renderingWorker(render, std::ref(sptr_intact));

        /** find epsilon */
        std::thread epsilonWorker(estimate, std::ref(sptr_intact));

        /** cluster */
        std::thread clusteringWorker(cluster, std::ref(sptr_intact));

        /** find objects */
        std::thread detectionWorker(
            detect, std::ref(sptr_intact), std::ref(sptr_kinect));

        /** wait for segmentation ~15ms */
        while (!sptr_intact->isSegmented()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }

        // ------> do stuff with raw point cloud and segment <------
        // ..... access pcl and use it
        sptr_intact->getPcl(); // std::make_shared<std::vector<float>>
        sptr_intact->getImg(); // std::make_shared<std::vector<uint8_t>>
        sptr_intact->getSegmentedPcl(); // std::make_shared<std::vector<float>>
        sptr_intact
            ->getSegmentedImg(); // std::make_shared<std::vector<uint8_t>>
        // ------> do stuff with raw point cloud and segment <------

        dataAcquisitionWorker.join();
        segmentationWorker.join();
        renderingWorker.join();
        epsilonWorker.join();
        clusteringWorker.join();
        calibrationWorker.join();
        return 0;

    } catch (...) {
        sptr_kinect->release();
        // sptr_kinect->releaseClones();
        return -1;
    }
}
