#include <vector>

#include "io.h"
#include "point.h"
#include "lda.h"
#include "svd.h"
#include "edge.h"

extern const int FAIL = -3;
extern const int PASS = 0;
const int tweak = 60;

static std::vector<Point> segment(std::vector<Point> points){
    Timer timer;
    /** do linear analysis */
    std::vector<Point> refined = lda::analyze(points);

    /** grow course segment  */
    std::vector<Point> roi = svd::compute(refined);

    /** segment tabletop interaction context */
    std::vector<Point> context = edge::segment(roi);
    LOG(INFO) << "..........................................................";
    LOG(INFO) << timer.getDuration() << " ms: total segmentation runtime (!unoptimized!)";

    return context;
}

int main(int argc, char* argv[])
{
    /** initialize logger */
    logger(argc, argv);

    /** developer testing point cloud */
    std::vector<Point> points;
    const std::string DATA = io::pwd() + "/resources/inputdata.ply";
    points = io::read(points, DATA.c_str());

    /** segment tabletop interaction context */
    std::vector<Point> context = segment(points);

    return PASS;
}
