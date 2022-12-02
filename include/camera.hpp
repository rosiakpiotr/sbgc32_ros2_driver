#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <string>

#include "opencv2/opencv.hpp"

class Camera
{
public:
    Camera(int captureWidth, int captureHeight, int framerate = 30);

    bool isOpened();

private:
    std::string createGstreamerPipeline();

    int captureWidth;
    int captureHeight;
    int framerate;

    cv::VideoCapture cap;
};

#endif