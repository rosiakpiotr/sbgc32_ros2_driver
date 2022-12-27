#include <iostream>
#include <exception>

#include <opencv2/opencv.hpp>

#include "gimbal.hpp"
#include "camera.hpp"
#include "cascade-face-detector.hpp"
#include "helpers.hpp"
#include "constants.hpp"
#include "angles.hpp"

int main()
{
    cv::CascadeClassifier cascade;
    bool success = cascade.load("./face.xml");
    if (!success)
    {
        std::cout << "Problem while loading face model." << std::endl;
        return -1;
    }

    Gimbal gimbal;
    try
    {
        gimbal.initializeDriver();
    }
    catch (const std::runtime_error &e)
    {
        std::cerr << e.what() << std::endl;
        return -1;
    }

    gimbal.configControl();
    gimbal.motorsOn();

    Camera camera(CAPTURE_WIDTH, CAPTURE_HEIGHT, FRAMERATE);
    if (!camera.isOpened())
    {
        std::cout << "Failed to open camera." << std::endl;
        return -1;
    }

    cv::namedWindow("Camera feed", cv::WINDOW_AUTOSIZE);
    const int minFaceSize = 100;
    const int maxFaceSize = 600;
    const float detectionCondidence = 1.0;
    char c;
    do
    {
        c = (char)cv::waitKey(25);

        cv::Mat frame;
        camera >> frame;
        if (frame.empty())
            break;

        auto detector = CascadeFaceDetector(&cascade, frame,
                                            minFaceSize, maxFaceSize,
                                            detectionCondidence);
        detector.showResults(frame);
        imshow("Frame", frame);

        if (!detector.faceDetected())
            continue;

        cv::Point faceCenter = detector.centerOfFirstHit();
        // Move origin of the detected face's center from frame's left top to frame's center.
        // This way we get offsets from the center giving us information on how to move
        // the camera to keep detected object in the center.
        cv::Point globalCenterOffset = faceCenter - cv::Point(CAPTURE_WIDTH / 2, CAPTURE_HEIGHT / 2);
        // Euler angles that gimbal should move by to keep detected object in the center of captured image.
        Angles offsetAngles = globalOffsetToCameraAngles(globalCenterOffset);

        Angles currentPos = gimbal.getCurrentPosition();
        gimbal.moveToAngles(currentPos + offsetAngles);

    } while (c != 0x1B); // 0x1B is 'Escape' key's ASCII code

    gimbal.motorsOff();
    cv::destroyAllWindows();

    return 0;
}
