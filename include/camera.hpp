#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <string>

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

const std::string V4L2 = "v4l2src";
const std::string LIBCAMERA = "libcamerasrc";

class Camera
{
private:
    cv::VideoCapture cap;

public:
    Camera(const std::string &source, int captureWidth, int captureHeight, int framerate = 30);

    ~Camera();

    friend cv::Mat &operator>>(Camera &camera, cv::Mat &image);
};

cv::Mat &operator>>(Camera &camera, cv::Mat &image);

#endif