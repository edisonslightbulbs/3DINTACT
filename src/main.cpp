#include <vector>

#include "io.h"
#include "lda.h"
#include "point.h"
#include "scalpel.h"
#include "svd.h"

extern const int FAIL = -3;
extern const int PASS = 0;

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

    LOG(INFO) << timer.getDuration() << " ms: total runtime";

    float percent = 100;
    float refineDataReduction = (float)refined.size() / points.size() * percent;
    float roiDataReduction = (float)roi.size() / points.size() * percent;
    float segmentDataReduction = (float)context.size() / points.size();

    /** output resources for downstream analysis */
    io::performance(points.size(), refined.size(), refineTime, roi.size(),
        roiTime, context.size(), segmentTime, timer.getDuration());
    io::reduction(refineDataReduction, roiDataReduction, segmentDataReduction);

    return context;
}

int main(int argc, char* argv[])
{
    /** initialize logger */
    logger(argc, argv);

    /** developer testing point cloud */
    std::vector<Point> points;
    const std::string DATA = io::pwd() + "/output/tabletop.ply";
    points = io::read(points, DATA.c_str());

    /** segment tabletop interaction context */
    std::vector<Point> context = segment(points);

    /** output point cloud as a *.ply file */
    io::write_ply(context);

    return PASS;
}
