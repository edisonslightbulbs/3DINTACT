#include <opencv2/core/mat.hpp>
#include <thread>
#include <torch/script.h>

#include "i3d.h"
#include "i3dscene.h"
#include "i3dutils.h"
#include "io.h"
#include "kinect.h"
#include "od.h"
#include "usage.h"

int main(int argc, char* argv[])
{
    // init logger, kinect, and i3d
    logger(argc, argv);
    usage::prompt(ABOUT);
    std::shared_ptr<kinect> sptr_kinect(new kinect);
    std::shared_ptr<i3d> sptr_i3d(new i3d());
    std::thread work(
        i3dscene::context, std::ref(sptr_kinect), std::ref(sptr_i3d));

    // initialize object detection resources
    torch::jit::script::Module module;
    std::vector<std::string> classnames;
    std::string torchscript = io::pwd() + "/resources/best.pt";
    std::string classes = io::pwd() + "/resources/class.names";
    od::setup(classnames, module, torchscript, classes);

    // segment and cluster interaction regions in fish tank
    WAIT_FOR_CLUSTERS

    // process synchronized point cloud and images
    while (RUN) {
        int w = sptr_i3d->getDWidth();
        int h = sptr_i3d->getDHeight();

        std::vector<Point> pCloud = *sptr_i3d->getPCloud();
        uint8_t* pCloudSynchImage = *sptr_i3d->getC2DBGRAData();

        uint8_t processedImage[w * h * 4];
        for (int i = 0; i < pCloud.size(); i++) {
            // do some operation of images using RGBD here
            // ++
            i3dutils::stitch(i, pCloud[i], processedImage);
        }
        od::detect(h, w, pCloudSynchImage, classnames, module, sptr_i3d);
    }
    work.join();
    return 0;
}
