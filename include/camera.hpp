#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <string>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

class Camera
{
public:
    Camera(int captureWidth, int captureHeight, int framerate = 30);

    ~Camera();

    bool isOpened();

    friend cv::Mat &operator>>(Camera &camera, cv::Mat &image);

private:
    std::string createGstreamerPipeline();

    int captureWidth;
    int captureHeight;
    int framerate;

    cv::VideoCapture cap;
};

cv::Mat &operator>>(Camera &camera, cv::Mat &image);

#endif