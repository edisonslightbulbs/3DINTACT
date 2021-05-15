#ifndef MAIN_UTILITY_H
#define MAIN_UTILITY_H

#include <string>
#include <thread>
#include <torch/script.h>

#include "intact.h"

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

void getPoint(const int& index, std::vector<float>& pcl, const short* ptr_data)
{
    pcl[3 * index + 0] = (float)ptr_data[3 * index + 0];
    pcl[3 * index + 1] = (float)ptr_data[3 * index + 1];
    pcl[3 * index + 2] = (float)ptr_data[3 * index + 2];
}

void getPixel(
    const int& index, std::vector<uint8_t>& img, const uint8_t* ptr_data)
{
    /** n.b., kinect colors reversed */
    img[3 * index + 2] = ptr_data[4 * index + 0];
    img[3 * index + 1] = ptr_data[4 * index + 1];
    img[3 * index + 0] = ptr_data[4 * index + 2];
}

void zeroData(const int& index, uint8_t* data)
{
    data[4 * index + 0] = 0; // blue
    data[4 * index + 1] = 0; // green
    data[4 * index + 2] = 0; // red
    data[4 * index + 3] = 0; // alpha
}

void zeroPoint(const int& index, std::vector<float>& pcl)
{
    pcl[3 * index + 0] = 0.0f;
    pcl[3 * index + 1] = 0.0f;
    pcl[3 * index + 2] = 0.0f;
}

void zeroPixel(const int& index, std::vector<uint8_t>& img)
{
    img[3 * index + 2] = 0;
    img[3 * index + 1] = 0;
    img[3 * index + 0] = 0;
}

void getPixelData(
    const int& index, std::vector<uint8_t>& img, const uint8_t* ptr_data)
{
    img[4 * index + 0] = ptr_data[4 * index + 0];
    img[4 * index + 1] = ptr_data[4 * index + 1];
    img[4 * index + 2] = ptr_data[4 * index + 2];
    img[4 * index + 3] = ptr_data[4 * index + 3];
}
void updateSegment(std::shared_ptr<Intact>& sptr_intact,
    const std::vector<float>& pcl, const std::vector<uint8_t>& img,
    const std::vector<float>& segmentPcl,
    const std::vector<uint8_t>& segmentImg)
{
    sptr_intact->setPcl(pcl);
    sptr_intact->setImg(img);
    // sptr_intact->setSegmentedImgData(segmentedImgData); // todo: [checked]

    if (sptr_intact->getSegmentBoundary().second.m_xyz[2] == __FLT_MAX__
        || sptr_intact->getSegmentBoundary().first.m_xyz[2] == __FLT_MIN__) {
        sptr_intact->setSegmentedPcl(pcl);
        sptr_intact->setSegmentedImg(img);
    } else {
        sptr_intact->setSegmentedPcl(segmentPcl);
        sptr_intact->setSegmentedImg(segmentImg);
    }
}

bool outsideSegment(
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
#endif /*MAIN_UTILITY_H */
