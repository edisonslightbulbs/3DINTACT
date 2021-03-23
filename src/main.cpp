#include <vector>

#include "frame.h"
#include "io.h"
#include "kinect.h"
#include "outliers.h"
#include "point.h"
#include "segment.h"
#include "svd.h"

extern const int FAIL = -3;
extern const int PASS = 0;


static std::vector<Point> define(std::vector<Point>& points)
{
    Timer timer;
    float rawSize = points.size();

    /** remove outliers*/
    std::vector<Point> denoised = outliers::remove(points);
    std::string removeTime = timer.getDuration();

    /** grow course segment  */
    std::vector<Point> proposal = svd::compute(denoised);
    std::string computeTime = timer.getDuration();

    /** segment interaction context */
    std::vector<Point> context = segment::cut(proposal);
    std::string cutTime = timer.getDuration();

    /** log performance */
    io::performance(rawSize, denoised.size(), removeTime, proposal.size(), computeTime, context.size(), cutTime, timer.getDuration());
    return context;
}

static std::vector<Point> plyFile()
{
    std::vector<Point> points;
    const std::string DATA = io::pwd() + "/output/tabletop.ply";
    points = io::read(points, DATA.c_str());
    return points;
}
#if __linux__
static std::vector<Point> getKinectImage()
{
    Kinect kinect;                   // <- set up kinect
    Frame frame = kinect.getImage(); // <- get surface view
    io::write(frame.m_rgb);          // <- synchronized rgb image
    frame.release();                 // <- release resources
    kinect.close();                  // <- close kinect
    return frame.m_points;           // <- point cloud
}
#endif

int main(int argc, char* argv[])
{
    /** initialize logger */
    logger(argc, argv);

    /** get point cloud */
    std::vector<Point> points;
#if __linux__
     points = getKinectImage();
#elif __APPLE__
     points = plyFile();
#endif

    Timer timer;
    /** segment tabletop interaction context */
    std::vector<Point> context = define(points);
    LOG(INFO) << timer.getDuration() << " ms: segmentation runtime";

    io::write_ply(context);
    return PASS;
}
