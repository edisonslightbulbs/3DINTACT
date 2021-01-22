#ifndef IO_H
#define IO_H

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "point.h"

#ifdef WINDOWS
#include <direct.h>
#define wd _getcwd
#else
#include <unistd.h>
#define wd getcwd
#endif

static std::string pwd()
{
    char buff[FILENAME_MAX];
    wd(buff, FILENAME_MAX);
    std::string workingDir(buff);
    return workingDir;
}

class IO {

public:
    static std::vector<Point> read(
        std::vector<Point> t_points, const char* t_file);

    static void sortNeighbours(
        const int& ID, std::vector<std::vector<Point>>& t_neighbours);

    static void write(const int& ID, const int& K,
        std::vector<std::vector<Point>>& t_neighbours,
        const std::string& t_file);

    static void write(
        const std::vector<Point>& t_points, const std::string& t_file);
};

#endif /* IO_H */
