#include "camera.hpp"

Camera::Camera(int captureWidth, int captureHeight, int framerate) : captureWidth(captureWidth),
                                                                     captureHeight(captureHeight),
                                                                     framerate(framerate),
                                                                     cap(createGstreamerPipeline(), cv::CAP_GSTREAMER)
{
}

Camera::~Camera()
{
    cap.release();
}

cv::Mat &operator>>(Camera &camera, cv::Mat &image)
{
    camera >> image;
    return image;
}

std::string Camera::createGstreamerPipeline()
{
    return " libcamerasrc ! video/x-raw, "
           " width=(int)" +
           std::to_string(captureWidth) + ","
                                          " height=(int)" +
           std::to_string(captureHeight) + ","
                                           " framerate=(fraction)" +
           std::to_string(framerate) + "/1 !"
                                       " videoconvert ! videoscale !"
                                       " video/x-raw,"
                                       " width=(int)" +
           std::to_string(captureWidth) + ","
                                          " height=(int)" +
           std::to_string(captureHeight) + " ! appsink";
}

bool Camera::isOpened()
{
    return cap.isOpened();
}
