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

#include "controller.hpp"

#include "strategies/global.hpp"
#include "strategies/local.hpp"

#include "constants.hpp"

using namespace std;

int main()
{
    try {
        // Camera camera = Camera(LIBCAMERA, CAPTURE_WIDTH, CAPTURE_HEIGHT, FRAMERATE);
        // shared_ptr<Gimbal> gimbal = make_shared<RealGimbal>();

        // Camera camera = Camera(V4L2, CAPTURE_WIDTH, CAPTURE_HEIGHT, FRAMERATE);
        Camera camera;
        shared_ptr<Gimbal> gimbal = make_shared<FakeGimbal>();

        shared_ptr<Detector> detector = make_shared<CascadeDetector>("./face.xml", 100, 600, 1.0, true);

        shared_ptr<Strategy> strategy = make_shared<GlobalStrategy>(CAMERA_FOCAL_LENGTH, true);
        // shared_ptr<Strategy> strategy = make_shared<LocalStrategy>(2.0,true);

        Controller controller(detector, gimbal, strategy, 70, 1, true);

        gimbal->motorsOn();

        cv::namedWindow("Frame", cv::WINDOW_AUTOSIZE);
        char c;
        unsigned long long count = 0;
        do {
            c = (char) cv::waitKey(25);

            cv::Mat frame;
            camera >> frame;
            if (frame.empty())
                break;

            controller.control(count, frame);

            imshow("Frame", frame);

            count++;
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

}
