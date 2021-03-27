
#include "kinect.h"
#include "workers.h"

extern const int FAIL = -3;
extern const int PASS = 0;

int main(int argc, char* argv[])
{
    logger(argc, argv);
    Kinect kinect;
    workers::manage(kinect);
    return PASS;
}
