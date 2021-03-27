#include <vector>

#include "frame.h"
#include "io.h"
#include "kinect.h"
#include "logger.h"
#include "point.h"
#include "segment.h"

extern const int FAIL = -3;
extern const int PASS = 0;

int main(int argc, char* argv[])
{
    /** initialize logger */
    logger(argc, argv);
    Kinect kinect;                   // <- set up kinect
    Frame frame = kinect.getImage(); // <- get surface view
    io::write(frame.m_rgb);          // <- synchronized rgb image
    frame.release();                 // <- release resources
    kinect.close();                  // <- close kinect

    /** segment tabletop interaction context */
    std::vector<Point> context = segment::cut(frame.m_points);

    /** output and visualize segment */
    io::ply(frame.m_points, context);
    return PASS;
}