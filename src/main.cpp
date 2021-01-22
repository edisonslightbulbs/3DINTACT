#include <fstream>
#include <iostream>

#include <gflags/gflags.h>

#include "dbscan.h"
#include "logger.h"
#include "point.h"

// Getting absolute path to
// the working directory
//
#ifdef WINDOWS
#include <direct.h>
#define getCurrentDir _getcwd
#else
#include <unistd.h>
#define getCurrentDir getcwd
#endif

extern const int SUCCESS = 0;
extern const int FAILURE = -3;

std::string pwd()
{
    char buff[FILENAME_MAX];
    getCurrentDir(buff, FILENAME_MAX);
    std::string workingDir(buff);
    return workingDir;
}

void readData(std::vector<Point*>& t_points, const char* t_file)
{
    /* type definitions for the data */
    float x = 0;
    float y = 0;
    float z = 0;

    /* file stream and string for parsing csv file */
    std::ifstream data(t_file);
    std::string line;

    /* get each line in csv */
    while (std::getline(data, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> row;

        /* parse each line */
        while (std::getline(ss, cell, ',')) {
            row.push_back(cell);
        }
        /* create data points */
        x = (float)std::stof(row[0]);
        y = (float)std::stof(row[1]);
        z = (float)std::stof(row[2]); // <---
        Point* ptr_point = new Point3d(x, y, z);
        t_points.push_back(ptr_point);
    }
}

void write3d(const std::vector<Point*>& t_points, const std::string& t_file)
{
    std::ofstream filestream;
    filestream.open(t_file);
    filestream << "x,y,z,label" << std::endl;

    for (auto* ptr_point : t_points) {
        filestream << ptr_point->m_x << "," << ptr_point->m_y << ","
                   << ptr_point->m_z << "," << ptr_point->m_cluster
                   << std::endl;
    }
    filestream.close();
}

int main(int argc, char* argv[])
{
    logger(argc, argv);
    std::vector<Point*> points3d;
    const std::string INPUT_FILE = pwd() + "/resources/somedata.txt";
    const std::string OUTPUT_FILE = pwd() + "/build/results.csv";

    /* read data into __std::vector<Point*>__ */
    {
        Timer timer;
        readData(points3d, INPUT_FILE.c_str());
        LOG(INFO) << "reading input file took: " << timer.getDuration();
    }

    /**
     * create Dbscan object:
     *   on creation, ds runs the density-based
     *   spatial clustering (DBSCAN) on specified data. */
    {
        Timer timer;
        Dbscan ds(points3d);
        LOG(INFO) << "running dbscan on " << points3d.size()
                  << " points(3D) took: " << timer.getDuration();
    }

    /* write output <--- graphed using an external python script */
    {
        Timer timer;
        write3d(points3d, OUTPUT_FILE);
        LOG(INFO) << "writing output file to results.csv file took: "
                  << timer.getDuration();
    }
    return SUCCESS;
}
