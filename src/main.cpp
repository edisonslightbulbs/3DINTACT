#include <thread>
#include <k4a/k4a.h>
#include <string>

#include "intact.h"
#include "kinect.h"
#include "io.h"
#include "logger.h"
#include "transformation_helpers.h"

extern const int FAIL = -3;
extern const int PASS = 0;
std::shared_ptr<bool> RUN_SYSTEM;

static bool point_cloud_color_to_depth(k4a_transformation_t transformation_handle,
                                       const k4a_image_t depth_image,
                                       const k4a_image_t color_image,
                                       std::string file_name)
{
    int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
    int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);
    k4a_image_t transformed_color_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                 depth_image_width_pixels,
                                                 depth_image_height_pixels,
                                                 depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
                                                 &transformed_color_image))
    {
        printf("Failed to create transformed color image\n");
        return false;
    }

    k4a_image_t point_cloud_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                                                 depth_image_width_pixels,
                                                 depth_image_height_pixels,
                                                 depth_image_width_pixels * 3 * (int)sizeof(int16_t),
                                                 &point_cloud_image))
    {
        printf("Failed to create point cloud image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation_handle,
                                                                               depth_image,
                                                                               color_image,
                                                                               transformed_color_image))
    {
        printf("Failed to compute transformed color image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
                                                                              depth_image,
                                                                              K4A_CALIBRATION_TYPE_DEPTH,
                                                                              point_cloud_image))
    {
        printf("Failed to compute point cloud\n");
        return false;
    }

    tranformation_helpers_write_point_cloud(point_cloud_image, transformed_color_image, file_name.c_str());

    k4a_image_release(transformed_color_image);
    k4a_image_release(point_cloud_image);

    return true;
}

static bool point_cloud_depth_to_color(k4a_transformation_t transformation_handle,
                                       const k4a_image_t depth_image,
                                       const k4a_image_t color_image,
                                       std::string file_name)
{
    // transform color image into depth camera geometry
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
    k4a_image_t transformed_depth_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                 color_image_width_pixels,
                                                 color_image_height_pixels,
                                                 color_image_width_pixels * (int)sizeof(uint16_t),
                                                 &transformed_depth_image))
    {
        printf("Failed to create transformed depth image\n");
        return false;
    }

    k4a_image_t point_cloud_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                                                 color_image_width_pixels,
                                                 color_image_height_pixels,
                                                 color_image_width_pixels * 3 * (int)sizeof(int16_t),
                                                 &point_cloud_image))
    {
        printf("Failed to create point cloud image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image))
    {
        printf("Failed to compute transformed depth image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
                                                                              transformed_depth_image,
                                                                              K4A_CALIBRATION_TYPE_COLOR,
                                                                              point_cloud_image))
    {
        printf("Failed to compute point cloud\n");
        return false;
    }

    tranformation_helpers_write_point_cloud(point_cloud_image, color_image, file_name.c_str());

    k4a_image_release(transformed_depth_image);
    k4a_image_release(point_cloud_image);

    return true;
}

static int capture(std::shared_ptr<Kinect>& sptr_kinect)
{
    const std::string C2D = io::pwd() + "/output/color2depth.ply";
    if (point_cloud_color_to_depth(sptr_kinect->m_transformation, sptr_kinect->m_depthImage, sptr_kinect->m_rgbImage, C2D) ==false)
    {
        sptr_kinect->release();
        sptr_kinect->close();
        throw std::runtime_error("Failed to create point cloud image!");
    }

    // Compute color point cloud by warping depth image into color camera geometry
    const std::string D2C = io::pwd() + "/output/depth2color.ply";
    if (point_cloud_depth_to_color(sptr_kinect->m_transformation, sptr_kinect->m_depthImage, sptr_kinect->m_rgbImage, D2C) ==false)
    {
        sptr_kinect->release();
        sptr_kinect->close();
        throw std::runtime_error("Failed to create point cloud image!");
    }

    // Compute color point cloud by warping depth image into color camera geometry with downscaled color image and
    // downscaled calibration. This example's goal is to show how to configure the calibration and use the
    // transformation API as it is when the user does not need a point cloud from high resolution transformed depth
    // image. The downscaling method here is naively to average binning 2x2 pixels, user should choose their own
    // appropriate downscale method on the color image, this example is only demonstrating the idea. However, no matter
    // what scale you choose to downscale the color image, please keep the aspect ratio unchanged (to ensure the
    // distortion parameters from original calibration can still be used for the downscaled image).
    memcpy(&sptr_kinect->calibration_color_downscaled, & sptr_kinect->m_calibration, sizeof(k4a_calibration_t));
    sptr_kinect->calibration_color_downscaled.color_camera_calibration.resolution_width /= 2;
    sptr_kinect->calibration_color_downscaled.color_camera_calibration.resolution_height /= 2;
    sptr_kinect->calibration_color_downscaled.color_camera_calibration.intrinsics.parameters.param.cx /= 2;
    sptr_kinect->calibration_color_downscaled.color_camera_calibration.intrinsics.parameters.param.cy /= 2;
    sptr_kinect->calibration_color_downscaled.color_camera_calibration.intrinsics.parameters.param.fx /= 2;
    sptr_kinect->calibration_color_downscaled.color_camera_calibration.intrinsics.parameters.param.fy /= 2;
    sptr_kinect->transformation_color_downscaled = k4a_transformation_create(&sptr_kinect->calibration_color_downscaled);
    sptr_kinect->color_image_downscaled = downscale_image_2x2_binning(sptr_kinect->m_rgbImage);
    if (sptr_kinect->color_image_downscaled == 0)
    {
        printf("Failed to downscaled color image\n");
        sptr_kinect->release();
        sptr_kinect->close();
        throw std::runtime_error("Failed to create point cloud image!");
    }

    const std::string SCALED = io::pwd() + "/output/downscaled.ply";
    if (point_cloud_depth_to_color(sptr_kinect->transformation_color_downscaled,
                                   sptr_kinect->m_depthImage,
                                   sptr_kinect->color_image_downscaled,
                                   SCALED) == false)
    {
        sptr_kinect->release();
        sptr_kinect->close();
        throw std::runtime_error("Failed to create point cloud image!");
    }

}

void work(std::shared_ptr<Kinect> sptr_kinect)
{
    /** segment in separate worker thread */
    std::thread segmenting(intact::segment, std::ref(sptr_kinect));

    /** render in separate worker thread */
    std::thread rendering(intact::render, std::ref(sptr_kinect));

    /** manage worker threads */
    segmenting.join();
    rendering.join();
}

int main(int argc, char* argv[])
{
    /** start system */
    RUN_SYSTEM = std::make_shared<bool>(true);
    std::cout << "Press ESC to exit." << std::endl;
    logger(argc, argv);

    /** start kinect */
    std::shared_ptr<Kinect> sptr_kinect(new Kinect);

    capture(sptr_kinect);

    /** do multi-threaded work*/
    //work(sptr_kinect);

    return PASS;
}

