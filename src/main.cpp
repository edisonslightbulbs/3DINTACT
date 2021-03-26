#include <vector>

#include "frame.h"
#include "io.h"
#include "kinect.h"
#include "logger.h"
#include "point.h"
#include "segment.h"

extern const int FAIL = -3;
extern const int PASS = 0;

static std::vector<Point> plyFile()
{
    std::vector<Point> points;
    const std::string DATA = io::pwd() + "/output/tabletop.ply";
    points = io::read(points, DATA.c_str());
    return points;
}

/** Iff on mac, emulate kinect using resource in ./output/ */
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

/** Iff on mac, emulate kinect using resource in ./output/ */
#if __linux__
    points = getKinectImage();
#elif __APPLE__
    points = plyFile();
#endif

    /** segment tabletop interaction context */
    Timer timer;
    std::vector<Point> context = segment::cut(points);
    LOG(INFO) << "Interaction context segmented in: " << timer.getDuration()
              << " ms";

    /** write point cloud to ./output/context.ply file */
    io::ply(points, context);
    return PASS;
}
