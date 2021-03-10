#ifndef IO_H
#define IO_H

#include "point.h"

#include <fstream>
#include <string>
#include <vector>

class IO {

public:
    static std::string pwd();

    static std::vector<Point> read(
        std::vector<Point> t_points, const char* t_file);

    static void write(
        const std::vector<float>& distances, const std::string& file);

    static void write(
        const std::vector<Point>& t_points, const std::string& t_file);
};

#endif /* IO_H */
