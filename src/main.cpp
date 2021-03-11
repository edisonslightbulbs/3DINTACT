#include <vector>

//#include "dbscan.h"
#include "io.h"
#include "knn.h"
#include "logger.h"
#include "point.h"
#include "lda.h"
#include "timer.h"


int main(int argc, char* argv[])
{

    /** initialize logger */
    logger(argc, argv);

    /** point cloud */
    std::vector<Point> points;
    const std::string DATA = io::pwd() + "/resources/inputdata.ply";
    points = io::read(points, DATA.c_str());

    /** perform linear discriminant analysis */
    points = lda::reduce(points); // <-- ~ 59 ms O(N^2)

    /** estimate epsilon neighbourhood */
    {
        Timer timer;
        float epsilon = knn::elbow(points); // <-- best time 292 ms O(N^2)
        std::cout << epsilon << std::endl;
        //LOG(INFO) << timer.getDuration() << "ms : knn analysis";
    }

    /** cluster using dbscan */
    // std::vector<Point> clusteredPoints;
    // clusteredPoints = dbscan::cluster(points, MIN_POINTS);
    // IO::write(clusteredPoints, IO::pwd() + "/build/results.csv");


    return 0;
}
