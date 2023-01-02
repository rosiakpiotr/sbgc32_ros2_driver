#include "camera.hpp"

using namespace std;

string getFileName(const string &source, int captureWidth, int captureHeight, int framerate)
{
    string filename = source + "  ! video/x-raw, "
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

    return filename;
}

Camera::Camera(const string &source, int captureWidth, int captureHeight, int framerate)
{
    cap.open(getFileName(source, captureWidth, captureHeight, framerate), cv::CAP_GSTREAMER);
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