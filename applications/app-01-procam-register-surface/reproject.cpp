#include <thread>

#include "i3d.h"
#include "i3dscene.h"
#include "io.h"
#include "kinect.h"
#include "usage.h"
#include "surface.h"

int main(int argc, char* argv[])
{
    // init logger, kinect, and i3d
    logger(argc, argv);
    usage::prompt(ABOUT);
    std::shared_ptr<kinect> sptr_kinect(new kinect);
    std::shared_ptr<i3d> sptr_i3d(new i3d());

    // start i3d worker
    std::thread work(
            i3dscene::context, std::ref(sptr_kinect), std::ref(sptr_i3d));

    std::vector<cv::Mat> captures;

    // setup projector window
    int w = 1366;
    int h = 768;
    const std::string window = "re-projection window";
    surface::contrast(sptr_i3d, window, w, h, captures);

    // find area of projection (aop)
    cv::Rect boundary
            = surface::reg(captures[1], captures[0]);
    cv::Mat background = captures[1];

    // scene::undistort(background);
    cv::Mat roi = background(boundary);

    // reproject aop
    cv::Mat R, t;
    // scene::predistort(background);
    surface::project(window, w, h, roi, R, t);
    // cv::imshow("Region of interest", roi);

    work.join();
    return 0;
}
