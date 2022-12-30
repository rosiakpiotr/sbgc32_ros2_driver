#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <string>

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

class Camera
{
private:
    cv::VideoCapture cap;

public:
    Camera();
    Camera(const std::string &filename, int apiPreference = cv::CAP_ANY);

    ~Camera();

    friend cv::Mat &operator>>(Camera &camera, cv::Mat &image);
};

cv::Mat &operator>>(Camera &camera, cv::Mat &image);

Camera getRaspberyPiCamera(int captureWidth, int captureHeight, int framerate = 30);
Camera getDefaultCamera();

#endif