#include <gflags/gflags.h>

#include "io.h"
#include "logger.h"
#include "point.h"

#include "dbscan.h"

extern const int PASS = 0;
extern const int FAIL = -3;

int main(int argc, char* argv[])
{
    logger(argc, argv);
    std::vector<Point> points;

    /** resources */
    const std::string INPUT_FILE = pwd() + "/resources/somedata.txt";
    const std::string OUTPUT_FILE = pwd() + "/build/results.csv";

    /** parse data */
    points = IO::read(points, INPUT_FILE.c_str());

    /** run dbscan */
    dbscan ds(points);

    /** write graph-able output */
    IO::write(ds.m_points, OUTPUT_FILE);

    return PASS;
}
