#include <vector>

#include "dbscan.h"
#include "io.h"
#include "knn.h"
#include "logger.h"
#include "point.h"
#include "timer.h"


int main(int argc, char* argv[])
{
    /** initialize logger */
    logger(argc, argv);

    /** initialize point cloud container */
    std::vector<Point> points;

    /** read data */
    const std::string DATA = IO::pwd() + "/resources/inputdata.ply";
    points = IO::read(points, DATA.c_str());

    /** estimate epsilon neighbourhood */
    float epsilon = knn::elbow(points);
    std::cout << epsilon << std::endl;

    /** cluster using dbscan */
    // std::vector<Point> clusteredPoints;
    // clusteredPoints = dbscan::cluster(points, MIN_POINTS);
    // IO::write(clusteredPoints, IO::pwd() + "/build/results.csv");
    return 0;
}
