#include <Eigen/Dense>
#include <chrono>
#include <string>
#include <thread>

#include "i3d.h"
#include "io.h"
#include "kinect.h"
#include "macros.hpp"
#include "svd.h"
#include "utilities.h"
#include "viewer.h"

void viewRegion(std::shared_ptr<I3d>& sptr_i3d)
{
    viewer::render(sptr_i3d); // CTRL + c to cycle different views
}

void clusterRegion(std::shared_ptr<I3d>& sptr_i3d)
{
    int minPoints = 4;
    const float epsilon = 3.170;
    sptr_i3d->clusterRegion(epsilon, minPoints, sptr_i3d);
}

void segmentRegion(std::shared_ptr<I3d>& sptr_i3d)
{
    sptr_i3d->segmentRegion(sptr_i3d);
}

void proposeRegion(std::shared_ptr<I3d>& sptr_i3d)
{
    sptr_i3d->proposeRegion(sptr_i3d);
}

void buildPcl(std::shared_ptr<I3d>& sptr_i3d)
{
    sptr_i3d->buildPCloud(sptr_i3d);
}

void k4aCapture(
    std::shared_ptr<Kinect>& sptr_kinect, std::shared_ptr<I3d>& sptr_i3d)
{
    START
    sptr_kinect->capture();
    sptr_kinect->depthCapture();
    int w = k4a_image_get_width_pixels(sptr_kinect->m_depth);
    int h = k4a_image_get_height_pixels(sptr_kinect->m_depth);

    while (RUN) {
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
        EXIT_CALLBACK
        STOP_TIMER(" k4a driver thread: runtime @ ")
    }
}

int main(int argc, char* argv[])
{
    logger(argc, argv);
    LOG(INFO) << "-- 3DINTACT is currently unstable and should only be used "
                 "for academic purposes!";
    LOG(INFO) << "-- press ESC to exit";

    // initialize k4a kinect
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    // initialize the 3dintact
    std::shared_ptr<I3d> sptr_i3d(new I3d());
    sptr_i3d->raiseRunFlag();

    // capture using k4a depth sensor
    std::thread k4aCaptureWorker(
        k4aCapture, std::ref(sptr_kinect), std::ref(sptr_i3d));

    // build point cloud
    std::thread buildPCloudWorker(buildPcl, std::ref(sptr_i3d));

    // propose region
    std::thread proposeRegionWorker(proposeRegion, std::ref(sptr_i3d));

    // segment region
    std::thread segmentRegionWorker(segmentRegion, std::ref(sptr_i3d));

    // cluster segmented region
    std::thread clusterRegionWorker(clusterRegion, std::ref(sptr_i3d));

#define DEV_ENV 0
    // ------> do stuff with tabletop environment <------
    SLEEP_UNTIL_CLUSTERS_READY
    std::thread viewRegionWorker(viewRegion, std::ref(sptr_i3d));

#if DEV_ENV == 0
    while (RUN) {

        // get clusters
        auto clusters = sptr_i3d->getPCloudClusters();
        auto points = clusters->first;
        auto indexes = clusters->second;

        // cast 'Index clusters' to 'Point clusters'
        std::vector<std::vector<Point>> pointClusters;
        for (const auto& cluster : indexes) {
            std::vector<Point> heap;
            for (const auto& index : cluster) {
                heap.emplace_back(points[index]);
            }
            pointClusters.emplace_back(heap);
        }

        // config flags for svd computation
        int flag = Eigen::ComputeThinU | Eigen::ComputeThinV;

        // compute and heap the face normals of each cluster
        std::vector<Eigen::Vector3d> normals;
        for (const auto& cluster : pointClusters) {
            SVD usv(cluster, flag);
            normals.emplace_back(usv.getV3Normal());
        }

        // extract the vacant space and corresponding normal
        std::vector<Point> vacantSpace = pointClusters[0];
        Eigen::Vector3d n1 = normals[0];

        pointClusters.erase(pointClusters.begin());
        normals.erase(normals.begin());
        const float ARGMIN = -0.00000008;

        std::vector<std::vector<Point>> objectClusters;

        // find coplanar clusters corresponding to the vacant surface space
        int index = 0;
        for (const auto& n2 : normals) {
            double numerator = n1.dot(n2);
            double denominator = n1.norm() * n2.norm();
            double solution = std::acos(numerator / denominator);

            if (!std::isnan(solution) && solution < ARGMIN
                && solution > -ARGMIN) {
                for (const auto& point : pointClusters[index]) {
                    vacantSpace.emplace_back(point);
                }
            } else {
                objectClusters.emplace_back(pointClusters[index]);
            }
            index++;
        }

        // max number of cluster regions
        int max = 35;
        index = 0;
        std::vector<Point> pCloudSegment;
        for (auto& object : objectClusters) {
            if (index < max) {
                for (auto& point : object) {
                    pCloudSegment.emplace_back(point);
                }
                index++;
            }
        }

        int w = sptr_i3d->getDepthWidth();
        int h = sptr_i3d->getDepthHeight();

        int16_t pCloud[w * h * 3];
        uint8_t img_GL[w * h * 4];
        uint8_t img_CV[w * h * 4];

        uint8_t rgba[4] = { 27, 120, 55, 1 };

        for (auto& point : vacantSpace) {
            int id = point.m_id;
            points[id].setPixel_GL(rgba);
        }

        for (int i = 0; i < points.size(); i++) {
            utils::stitch(i, points[i], pCloud, img_GL, img_CV);
        }

        sptr_i3d->setColClusters({ pCloud, img_GL });
    }
    // ------> do stuff with tabletop environment <------
#endif

    k4aCaptureWorker.join();
    buildPCloudWorker.join();
    proposeRegionWorker.join();
    segmentRegionWorker.join();
    clusterRegionWorker.join();
    viewRegionWorker.join();
    return 0;
}
