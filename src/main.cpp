#include <vector>

#include "frame.h"
#include "io.h"
#include "kinect.h"
#include "lda.h"
#include "point.h"
#include "scalpel.h"
#include "svd.h"

extern const int FAIL = -3;
extern const int PASS = 0;

const float TO_PERCENTAGE = 100;

static std::vector<Point> segment(std::vector<Point> points)
{
    Timer timer;
    /** do linear analysis */
    std::vector<Point> refined = lda::analyze(points);
    std::string refineTime = timer.getDuration();

    /** grow course segment  */
    std::vector<Point> roi = svd::compute(refined);
    std::string roiTime = timer.getDuration();

    /** segment tabletop interaction context */
    std::vector<Point> context = scalpel::segment(roi);
    std::string segmentTime = timer.getDuration();

    /** readable queries */
    float refineDataReduction = (float)refined.size() / points.size() * TO_PERCENTAGE;
    float roiDataReduction = (float)roi.size() / points.size() * TO_PERCENTAGE;
    float segmentDataReduction = (float)context.size() / points.size() *TO_PERCENTAGE;

    /** output resources for downstream analysis */
    io::performance(points.size(), refined.size(), refineTime, roi.size(),
        roiTime, context.size(), segmentTime, timer.getDuration());
    io::reduction(refineDataReduction, roiDataReduction, segmentDataReduction);

    return context;
}

static std::vector<Point> plyFile()
{
    std::vector<Point> points;
    const std::string DATA = io::pwd() + "/output/tabletop.ply";
    points = io::read(points, DATA.c_str());
    return points;
}

static std::vector<Point> image()
{
    Kinect kinect;                   // <- set up kinect
    Frame frame = kinect.getImage(); // <- get surface view
    frame.release();                 // <- release resources
    kinect.close();

    // frame.m_rgb;                  // <- synchronized rgb image
    // frame.m_points;               // <- point cloud

    return frame.m_points;
}

int main(int argc, char* argv[])
{
    /** initialize logger */
    logger(argc, argv);

    /** get point cloud */
    //std::vector<Point> points = plyFile();
    std::vector<Point> points = image();

    Timer timer;
    /** segment tabletop interaction context */
    std::vector<Point> context = segment(points);
    LOG(INFO) << timer.getDuration() << " ms: segmentation runtime";

    /** output point cloud as a *.ply file */
    io::write_ply(context);

    return PASS;
}
