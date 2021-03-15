#include <vector>

#include "dbscan.h"
#include "io.h"
#include "knn.h"
#include "point.h"
#include "lda.h"
#include "sampler.h"
#include "elbow.h"
#include "iqr.h"
#include "svd.h"
#include "segmenter.h"

extern const int FAIL = -3;
extern const int PASS = 0;

static std::vector<Point> segment(std::vector<Point> points){
    Timer timer;
    /** reduce dimensions  */
    std::vector<Point> analyzedPoints = lda::reduce(points);

    /** remove outliers */
    std::vector<Point> denoisedPoints = iqr::denoise(analyzedPoints);

    /** grow region of interest */
    std::vector<Point> roi = svd::compute(denoisedPoints);

    /** sample grown region */
    std::vector<Point> sample = sampler::sample(roi);

    /** accumulate distances to 4th nearest neighbours */
    std::vector<float> knn4 = knn::compute(sample);

    /** find epsilon query size */
    float epsilon = elbow::find(knn4);
    LOG(INFO) << epsilon << ": ≈ epsilon";

    /** run: return clustered points and number of clusters */
    const int MIN_POINTS = 4;
    std::pair<std::vector<Point>, int> clusters = dbscan::run(roi, MIN_POINTS, epsilon);

    /** segment tabletop interaction context */
    std::vector<Point> context = segmenter::segment(clusters);

    LOG(INFO) << "..........................................................";
    LOG(INFO) << timer.getDuration() << " ms: total segmentation runtime (!unoptimized!)";

    /** external resources for downstream analysis */
    io::log_performance(points.size(), denoisedPoints.size(), roi.size(), context.size(), timer.getDuration());
    io::write_ply(context);

    return context;
}


int main(int argc, char* argv[])
{
    /** initialize logger */
    logger(argc, argv);

    /** point cloud */
    std::vector<Point> points;
    const std::string DATA = io::pwd() + "/resources/inputdata.ply";
    points = io::read(points, DATA.c_str());

    /** segment tabletop interaction context */
    std::vector<Point> tic = segment(points);

    return PASS;
}
