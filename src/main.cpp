#include <vector>

//#include "dbscan.h"
#include "io.h"
#include "knn.h"
#include "logger.h"
#include "point.h"
#include "lda.h"
#include "timer.h"
#include "seed.h"
#include "elbow.h"
#include "iqr.h"

int main(int argc, char* argv[])
{
    /** initialize logger */
    logger(argc, argv);

    /** point cloud */
    std::vector<Point> points;
    const std::string DATA = io::pwd() + "/resources/inputdata.ply";
    points = io::read(points, DATA.c_str());

    /** linear discriminant analysis: dimension reduction */
    points = lda::reduce(points);

    /** outlier removal */
    points = iqr::denoise(points);

    /** sample */
    std::vector<Point> sample(seed::sample(points));

    /** query sample distances to the 4th nearest neighbours */
    std::vector<float> knn4 = knn::compute(sample);

    /** estimate epsilon neighbourhood */
    float epsilon = elbow::find(knn4); // <-- best time 292 ms O(N^2)
    std::cout << epsilon << std::endl;




    /** cluster using dbscan */
    // std::vector<Point> clusteredPoints;
    // clusteredPoints = dbscan::cluster(points, MIN_POINTS);
    // IO::write(clusteredPoints, IO::pwd() + "/build/results.csv");


    return 0;
}
