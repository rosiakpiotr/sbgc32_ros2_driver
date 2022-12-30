#include "camera.hpp"

using namespace std;


Camera::Camera()
    : cap(0)
{
    if (!cap.isOpened()) {
        throw runtime_error("Failed to open camera.");
    }
}

Camera::Camera(const std::string &filename, int apiPreference)
    : cap(filename, apiPreference)
{
    if (!cap.isOpened()) {
        throw runtime_error("Failed to open camera.");
    }
}

Camera::~Camera()
{
    cap.release();
}

cv::Mat &operator>>(Camera &camera, cv::Mat &image)
{
    camera.cap >> image;
    return image;
}

Camera getRaspberyPiCamera(int captureWidth, int captureHeight, int framerate)
{
    string filename = " libcamerasrc ! video/x-raw, "
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

    return Camera(filename, cv::CAP_GSTREAMER);
}

Camera getDefaultCamera()
{
    return Camera();
}
