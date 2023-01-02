#include <iostream>
#include <exception>
#include <memory>

#include <opencv2/opencv.hpp>

#include "gimbal.hpp"
#include "gimbals/real.hpp"
#include "gimbals/fake.hpp"

#include "camera.hpp"

#include "detector.hpp"
#include "detectors/cascade.hpp"

#include "helpers.hpp"
#include "constants.hpp"
#include "angles.hpp"


using namespace std;

int main()
{
    try {
        // Camera camera = Camera(LIBCAMERA,CAPTURE_WIDTH, CAPTURE_HEIGHT, FRAMERATE);
        // shared_ptr<Gimbal> gimbal = make_shared<RealGimbal>();

        Camera camera = Camera(V4L2,CAPTURE_WIDTH, CAPTURE_HEIGHT, FRAMERATE);
        shared_ptr<Gimbal> gimbal = make_shared<FakeGimbal>();

        shared_ptr<Detector> detector = make_shared<CascadeDetector>("./face.xml", 100, 600, 1.0, true);

        gimbal->motorsOn();

        cv::namedWindow("Frame", cv::WINDOW_AUTOSIZE);
        char c;
        do {
            c = (char) cv::waitKey(25);

            cv::Mat frame;
            camera >> frame;
            if (frame.empty())
                break;

            boost::optional<cv::Point> point = detector->detect(frame);
            if(point) {
                gimbal->moveToAngles(Angles(30, 20, 10), 40);
            }

            imshow("Frame", frame);

        }
        while (c != 0x1B); // 0x1B is 'Escape' key's ASCII code

        gimbal->motorsOff();
    }
    catch (const exception &e) {
        cerr << e.what() << endl;
        return -1;
    }

    cv::destroyAllWindows();
    return 0;

//    Gimbal gimbal;
//    try
//    {
//        gimbal.initializeDriver();
//    }
//    catch (const std::runtime_error &e)
//    {
//        std::cerr << e.what() << std::endl;
//        return -1;
//    }
//
//    gimbal.configControl();
//    gimbal.motorsOn();
//
//
//    Camera camera(CAPTURE_WIDTH, CAPTURE_HEIGHT, FRAMERATE);
//    if (!camera.isOpened())
//    {
//        std::cout << "Failed to open camera." << std::endl;
//        return -1;
//    }
//
//    cv::namedWindow("Camera feed", cv::WINDOW_AUTOSIZE);
//    const int minFaceSize = 100;
//    const int maxFaceSize = 600;
//    const float detectionCondidence = 1.0;
//    char c;
//    do
//    {
//        c = (char)cv::waitKey(25);
//
//        cv::Mat frame;
//        camera >> frame;
//        if (frame.empty())
//            break;
//
//        auto detector = CascadeFaceDetector(&cascade, frame,
//                                            minFaceSize, maxFaceSize,
//                                            detectionCondidence);
//        detector.showResults(frame);
//        imshow("Frame", frame);
//
//        if (!detector.faceDetected())
//            continue;
//
//        cv::Point faceCenter = detector.centerOfFirstHit();
//        // Move origin of the detected face's center from frame's left top to frame's center.
//        // This way we get offsets from the center giving us information on how to move
//        // the camera to keep detected object in the center.
//        cv::Point globalCenterOffset = faceCenter - cv::Point(CAPTURE_WIDTH / 2, CAPTURE_HEIGHT / 2);
//        // Euler angles that gimbal should move by to keep detected object in the center of captured image.
//        Angles offsetAngles = globalOffsetToCameraAngles(globalCenterOffset);
//
//        Angles currentPos = gimbal.getCurrentPosition();
//        gimbal.moveToAngles(currentPos + offsetAngles);
//
//    } while (c != 0x1B); // 0x1B is 'Escape' key's ASCII code
//
//    gimbal.motorsOff();
//    cv::destroyAllWindows();

}
