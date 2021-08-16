#include <thread>
#include <opencv2/core/mat.hpp>

#include "i3d.h"
#include "i3dutils.h"
#include "i3dscene.h"
#include "kinect.h"
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

    std::vector<std::string> classnames;
    torch::jit::script::Module module;
    i3dutils::configTorch(classnames, module);
    WAIT_FOR_CLUSTERS

    while (RUN) {
        int w = sptr_i3d->getDWidth();
        int h = sptr_i3d->getDHeight();

        std::vector<Point> pCloud = *sptr_i3d->getPCloud();

        uint8_t* unmodifiedImage = *sptr_i3d->getC2DBGRAData();

        uint8_t modifiedImage[w * h * 4 ];
        for(int i = 0; i < pCloud.size(); i ++){
            // do some operation of images using RGBD here
            // ++
            i3dutils::stitch(i, pCloud[i], modifiedImage);
        }
        //i3dutils::findObjects(h, w, modifiedImage, classnames, module, sptr_i3d);
        i3dutils::findObjects(h, w, unmodifiedImage, classnames, module, sptr_i3d);
    }
    work.join();
    return 0;
}
