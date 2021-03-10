#include <algorithm>
#include <cmath>

#include "io.h"
#include "kdtreeknn.h"
#include "knn.h"
#include "naiveknn.h"

void csv(std::vector<float>& knn)
{
    /** output path */
    const std::string OUTPUT = IO::pwd() + "/build/knn.csv";

    /** first: sort in descending order */
    std::sort(knn.begin(), knn.end(), std::greater<>());
    IO::write(knn, OUTPUT);
}

std::vector<Point> seed(std::vector<Point> points)
{
    const int SAMPLE_SIZE = 10;

    /** find centroid */
    Point center = Point::centroid(points);

    /** compute distances from centroid for each point */
    for (auto& point : points) {
        float distance = point.distance(center);
        point.m_distance.second = distance;
    }

    /** sort points using computed distance */
    Point::sort(points);

    /** sample edge of cluster points */
    std::vector<Point> sample = std::vector<Point>(
        points.begin() + points.size() - SAMPLE_SIZE, points.end());

    /** visual check of sample size */
    std::cout << "sampled " << sample.size() << " points" << std::endl;

    return sample;
}

std::vector<float> compute(std::vector<Point>& points, const int& K)
{
    /** sample points (todo: revise sampling strategy)  */
    std::vector<Point> sample(seed(points));

    /** list of Kth nearest neighbours for all points */
    std::vector<float> knearest;

    /***************************** performance testing ************************/
    /** 1. naive: */
    knearest = naiveknn::run(sample, K);

    /** 2. kdtree: */
    kdtreeknn::run(sample, K);

    /***************************** performance testing ************************/

    csv(knearest); // graphable Kth nearest neighbours

    return knearest;
}

float knn::elbow(std::vector<Point>& points)
{
    const int K = 5; // <---- 4 excluding core point

    /** Kth nearest distance for every point */
    std::vector<float> knearest = compute(points, K);

    /** use Snell's Law to evaluate relative angular distances */
    std::vector<std::pair<float, float>> angles;

    float vec2;
    for (int i = 1; i < knearest.size() - 1; i++) {
        vec2 = knearest[i + 1] - knearest[i];
        float angle = std::asin(vec2);
        angles.emplace_back(knearest[i], angle);
    }

    /** find the maximum absolute second derivative (masd) */
    float masd = 0;
    for (int i = 1; i < angles.size() - 1; i++) {
        float d1 = angles[i - 1].second - angles[i].second;
        float d2 = angles[i + 1].second - angles[i].second;
        float cd = 2 * angles[i].second;

        /** evaluate the second derivative (sd) - central difference (cd) */
        float sd = d1 + d2 - cd;

        /** determine elbow as it corresponds to the distance with the masd */
        if (sd > masd) {
            masd = angles[i].first;
        }
    }
    return masd;
}
