#include <iostream>

#include "dbscan.h"
#include "io.h"
#include "logger.h"
#include "point.h"

const int MIN_POINTS = 4;

int main()
{
    /** initialize data containers */
    std::vector<Point> points;
    std::vector<Point> clusteredPoints;

    /** read data */
    const std::string DATA = IO::pwd() + "/resources/somedata.txt";
    points = IO::read(points, DATA.c_str());

    /** cluster data */
    clusteredPoints = dbscan::cluster(points, MIN_POINTS);
    IO::write(clusteredPoints, IO::pwd() + "/build/results.csv");

    return 0;
}
