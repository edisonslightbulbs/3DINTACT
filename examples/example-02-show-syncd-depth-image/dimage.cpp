#include <thread>

#include "i3d.h"
#include "kinect.h"
#include "scene.h"
#include "usage.h"

int main(int argc, char* argv[])
{
    // init logger, kinect, and i3d
    logger(argc, argv);
    usage::prompt(ABOUT);
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);
    std::shared_ptr<I3d> sptr_i3d(new I3d());
    std::thread work(scene::context, std::ref(sptr_kinect), std::ref(sptr_i3d));

    // write synchronized depth image
    WAIT_FOR_COLOR_POINTCLOUD
    int w = sptr_i3d->getDWidth();
    int h = sptr_i3d->getDHeight();
    // uint8_t* bgra = sptr_i3d->getBGRA()->data();
    // io::write(w, h, bgra, "./output/synchronized-depth-image.ply");

    sptr_i3d->stop();
    work.join();
    return 0;
}
