#include <chrono>
#include <string>
#include <thread>
#include <torch/script.h>

#include "intact.h"
#include "io.h"
#include "kinect.h"
#include "logger.h"

void synchronize(std::shared_ptr<Intact>& sptr_intact,
    const std::vector<float>& raw, const std::vector<uint8_t>& rawColor,
    const std::vector<float>& segment, const std::vector<uint8_t>& segmentColor)
{
    sptr_intact->setPcl(raw);
    sptr_intact->setImg(rawColor);
    if (sptr_intact->getSegmentBoundary().second.m_xyz[2] == __FLT_MAX__
        || sptr_intact->getSegmentBoundary().first.m_xyz[2] == __FLT_MIN__) {
        sptr_intact->setSegmentPcl(raw);
        sptr_intact->setSegmentImg(rawColor);
    } else {
        sptr_intact->setSegmentPcl(segment);
        sptr_intact->setSegmentImg(segmentColor);
    }
}

bool outOfBounds(
    const int& index, const short* ptr_data, const Point& min, const Point& max)
{
    if (max.m_xyz[2] == __FLT_MAX__ || min.m_xyz[2] == __FLT_MIN__) {
        return true;
    }
    if ((float)ptr_data[3 * index + 0] > max.m_xyz[0]
        || (float)ptr_data[3 * index + 0] < min.m_xyz[0]
        || (float)ptr_data[3 * index + 1] > max.m_xyz[1]
        || (float)ptr_data[3 * index + 1] < min.m_xyz[1]
        || (float)ptr_data[3 * index + 2] > max.m_xyz[2]
        || (float)ptr_data[3 * index + 2] < min.m_xyz[2]) {
        return true;
    }
    return false;
}

void acquire(const int& index, std::vector<float>& pcl,
    std::vector<uint8_t>& pclCol, const short* ptr_data, const uint8_t* ptr_col)
{
    pcl[3 * index + 0] = (float)ptr_data[3 * index + 0];
    pcl[3 * index + 1] = (float)ptr_data[3 * index + 1];
    pcl[3 * index + 2] = (float)ptr_data[3 * index + 2];
    /** n.b., kinect colors reversed */
    pclCol[3 * index + 2] = ptr_col[4 * index + 0];
    pclCol[3 * index + 1] = ptr_col[4 * index + 1];
    pclCol[3 * index + 0] = ptr_col[4 * index + 2];
}

void acquire(const int& index, std::vector<float>& pcl,
    std::vector<uint8_t>& pclCol, const uint8_t* ptr_col)
{
    pcl[3 * index + 0] = 0.0f;
    pcl[3 * index + 1] = 0.0f;
    pcl[3 * index + 2] = 0.0f;
    /** n.b., kinect colors reversed */
    pclCol[3 * index + 2] = ptr_col[4 * index + 0];
    pclCol[3 * index + 1] = ptr_col[4 * index + 1];
    pclCol[3 * index + 0] = ptr_col[4 * index + 2];
}

void configTorch(
    std::vector<std::string>& classNames, torch::jit::script::Module& module)
{
    const std::string scriptName = io::pwd() + "/resources/torchscript.pt";
    const std::string cocoNames = io::pwd() + "/resources/coco.names";
    module = torch::jit::load(scriptName);
    std::ifstream f(cocoNames);
    std::string name;
    while (std::getline(f, name)) {
        classNames.push_back(name);
    }
}

// sense data acquisition
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
        std::vector<float> segment(N * 3);
        std::vector<uint8_t> segmentColor(N * 3);

        /** get point cloud */
        auto* data
            = (int16_t*)(void*)k4a_image_get_buffer(sptr_kinect->getPcl());
        uint8_t* color = k4a_image_get_buffer(sptr_kinect->getRgb2DepthImg());

        /** process point cloud */
        for (int i = 0; i < N; i++) {
            if (data[3 * i + 2] == 0) {
                /** zero invalid points */
                acquire(i, pcl, img, color);
                continue;
            }
            acquire(i, pcl, img, data, color);

            if (outOfBounds(i, data, sptr_intact->getSegmentBoundary().first,
                    sptr_intact->getSegmentBoundary().second)) {
                continue;
            }
            acquire(i, segment, segmentColor, data, color);
        }

        /** synchronize point cloud */
        synchronize(sptr_intact, pcl, img, segment, segmentColor);

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
    sptr_intact->detectObjects(classNames, module, sptr_intact, sptr_kinect);
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
    sptr_intact->getPcl();        // std::make_shared<std::vector<float>>
    sptr_intact->getImg();        // std::make_shared<std::vector<uint8_t>>
    sptr_intact->getSegmentPcl(); // std::make_shared<std::vector<float>>
    sptr_intact->getSegmentImg(); // std::make_shared<std::vector<uint8_t>>
    // ------> do stuff with raw point cloud and segment <------

    dataAcquisitionWorker.join();
    segmentationWorker.join();
    renderingWorker.join();
    epsilonWorker.join();
    clusteringWorker.join();
    calibrationWorker.join();

    /** snap scene image */
    // io::write(sptr_kinect->m_rgbImage);
    return 0;
}
