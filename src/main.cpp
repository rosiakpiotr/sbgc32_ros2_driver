#include <iostream>
#include <exception>
#include <math.h>

#include "gimbal.hpp"
#include "camera.hpp"

#include <opencv2/objdetect/objdetect.hpp>

#define CAPTURE_WIDTH 640
#define CAPTURE_HEIGHT 480
#define FRAMERATE 15

using namespace std;

cv::Point przesuw_kat(0, 0), new_pos(0, 0);

std::string gstreamer_pipeline(int capture_width, int capture_height, int framerate, int display_width, int display_height)
{
    return " libcamerasrc ! video/x-raw, "
           " width=(int)" +
           std::to_string(capture_width) + ","
                                           " height=(int)" +
           std::to_string(capture_height) + ","
                                            " framerate=(fraction)" +
           std::to_string(framerate) + "/1 !"
                                       " videoconvert ! videoscale !"
                                       " video/x-raw,"
                                       " width=(int)" +
           std::to_string(display_width) + ","
                                           " height=(int)" +
           std::to_string(display_height) + " ! appsink";
}

cv::Point detect(cv::Mat &frame)
{
    cv::Mat image = frame.clone();

    cv::medianBlur(image, image, 5);
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    //        captured_frame_bgr = cv2.cvtColor(captured_frame, cv2.COLOR_BGRA2BGR)
    //    # First blur to reduce noise prior to color space conversion
    //    captured_frame_bgr = cv2.medianBlur(captured_frame_bgr, 3)
    //    # Convert to Lab color space, we only need to check one channel (a-channel) for red here
    //    captured_frame_lab = cv2.cvtColor(captured_frame_bgr, cv2.COLOR_BGR2Lab)
    // captured_frame_lab_red = cv2.inRange(captured_frame_lab, np.array([20, 150, 150]), np.array([190, 255, 255]))

    int w = hsv_image.cols;
    int h = hsv_image.rows;
    //    cout << w << " " << h << endl;
    auto color = hsv_image.at<cv::Vec3b>(w / 2, h / 2);
    cout << int(color[0]) << " " << int(color[1]) << " " << int(color[2]) << endl;

    int minS = 128;
    int minV = 128;
    cv::Mat lower_red_hue_range;
    cv::Mat upper_red_hue_range;
    cv::inRange(hsv_image, cv::Scalar(0, minS, minV), cv::Scalar(10, 255, 255), lower_red_hue_range);
    cv::inRange(hsv_image, cv::Scalar(170, minS, minV), cv::Scalar(179, 255, 255), upper_red_hue_range);
    cv::Mat red_hue_image;
    cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);

    cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(red_hue_image, circles, cv::HOUGH_GRADIENT, 1, red_hue_image.rows / 8, 100, 20, 100, 200);

    cv::Point przesuw;
    if (circles.size() > 0)
    {
        for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle)
        {
            cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
            int radius = std::round(circles[current_circle][2]);

            przesuw = center;

            cv::circle(frame, center, radius, cv::Scalar(255, 0, 0), 2);
        }

        cv::cvtColor(red_hue_image, red_hue_image, cv::COLOR_GRAY2BGR);
        cv::Mat result;
        cv::addWeighted(red_hue_image, 1.0, frame, 1.0, 0.0, result);
        result.copyTo(frame);

        przesuw -= cv::Point(640 / 2, 480 / 2);

        return przesuw;
    }
    else
        return cv::Point(0, 0);
}

void katy(cv::Point &new_pos, Gimbal &gimbal)
{
    gimbal.movePitchTo(new_pos.y, 10);
    gimbal.moveYawTo(new_pos.x, 10);
}

cv::Point move(cv::Point &przesuw, Angles current_pos)
{
    /*przesuw kat - roznica pozycji przedmiotu i środka obrazu w stopniach
     * new_pos - globalny przesuw gimbala
     */
    /*
   if (przesuw.x > -50 && przesuw.x < 50)
   {
       cout << "Jest w srodku po osi x" << endl
            << "obecny kat " << current_pos.yaw << endl;
       new_pos.x = current_pos.yaw;
   }
   else
   {
       przesuw_kat.x = (przesuw.x * 63) / 640;
       cout << "Przesuniecie w osi x o " << przesuw.x << endl
            << "przesun o kat " << przesuw_kat.x << "stopni" << endl
            << "od " << current_pos.yaw << endl;

       new_pos.x = current_pos.yaw + przesuw_kat.x;
   }

   if (przesuw.y > -50 && przesuw.y < 50)
   {
       cout << "Jest w srodku po osi y" << endl
            << "obecny kat " << current_pos.pitch << endl;
       new_pos.x = current_pos.pitch;
   }
   else
   {

       przesuw_kat.y = (przesuw.y * 63) / 640;
       cout << "Przesuniecie w osi y o " << przesuw.y << endl
            << "przesun o kat " << przesuw_kat.y << "stopni" << endl
            << "od " << current_pos.pitch << endl;

       new_pos.y = current_pos.pitch + przesuw_kat.y;
   }
   return new_pos;
   */
    double kat = 10;
    /*
    if (przesuw.x > -50 && przesuw.x < 50)
    {
        cout << "Jest w srodku po osi x" << endl
             << "obecny kat " << current_pos.yaw << endl;
        new_pos.y = 0;
    }
    else
    {
        double r = przesuw.x/przesuw.y;
        przesuw_kat.x = (przesuw.x * 63) / 640;
        cout << "Przesuniecie w osi x o " << przesuw.x << endl
             << "przesun o kat " << przesuw_kat.x << "stopni" << endl
             << "od " << current_pos.yaw << endl;

        new_pos.x = current_pos.yaw+r*kat;
    }
    */

    if (przesuw.y > -50 && przesuw.y < 50)
    {
        cout << "Jest w srodku po osi y" << endl
             << "obecny kat " << current_pos.pitch << endl;
        new_pos.y = 0;
    }
    else
    {

        przesuw_kat.y = (przesuw.y * 63) / 640;
        cout << "Przesuniecie w osi y o " << przesuw.y << endl
             << "przesun o kat " << przesuw_kat.y << "stopni" << endl
             << "od " << current_pos.pitch << endl;

        new_pos.y = current_pos.pitch - kat;
    }
    return new_pos;
}

cv::Point detect_face(cv::Mat &frame, cv::CascadeClassifier &cascade, int minSize, int maxSize, double confidence)
{
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    vector<cv::Rect> result;
    vector<int> levels;
    vector<double> weights;
    cascade.detectMultiScale(gray, result, levels, weights, 1.1, 3, 0 | cv::CASCADE_SCALE_IMAGE,
                             cv::Size(minSize, minSize), cv::Size(maxSize, maxSize), true);
    cv::Point przesuw;
    if (result.size() > 0)
    {
        for (int i = 0; i < result.size(); i++)
        {
            if (weights[i] > confidence)
            {
                cv::rectangle(frame, result[i], cv::Scalar(0, 255, 0), 2);
                int x = result[i].x + result[i].width / 2;
                int y = result[i].y + result[i].height / 2;
                cv::circle(frame, cv::Point(x, y), 10, cv::Scalar(0, 255, 0), 2);
                przesuw = cv::Point(x, y);
            }
        }
        przesuw -= cv::Point(640 / 2, 480 / 2);

        return przesuw;
    }
    else
        return cv::Point(0, 0);
}

int main()
{
    cv::CascadeClassifier cascade;
    bool success = cascade.load("./face.xml");
    if (!success)
    {
        throw runtime_error("Problem while loading face model");
    }

    Gimbal gimbal;

    try
    {
        gimbal.initializeDriver();
    }
    catch (const std::runtime_error &e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }

    gimbal.configControl();
    gimbal.initializeRealTimeData(150);
    gimbal.motorsOn();
    // std::cout << cv::getBuildInformation();

    // pipeline parameters

    int capture_width = 640;  // 1280 ;
    int capture_height = 480; // 720 ;
    int framerate = 15;
    int display_width = 640;  // 1280 ;
    int display_height = 480; // 720 ;

    // reset frame average
    std::string pipeline = gstreamer_pipeline(capture_width, capture_height, framerate,
                                              display_width, display_height);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n\n\n";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened())
    {
        std::cout << "Failed to open camera." << std::endl;
        return (-1);
    }

    cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);

    // cv::VideoCapture cap(0);
    // cv::VideoCapture cap("v4l2src device=/dev/video0 io-mode=2 ! image/jpeg, width=(int)320, height=(int)240 ! nvjpegdec ! video/x-raw, format=I420 ! appsink", cv::CAP_GSTREAMER);
    // cv::VideoCapture cap("v4l2src device=/dev/video0", cv::CAP_GSTREAMER);
    // cv::VideoCapture cap("v4l2src device=/dev/video0 io-mode=2 ! video/x-raw, width=640, height=480, framerate=30/1 ! videoconvert ! videoscale ! appsink", cv::CAP_GSTREAMER);

    // if (!cap.isOpened())
    // {
    //     cout << "Error opening video stream or file" << endl;
    //     return -1;
    // }
    // gimbal.movePitchTo(0);
    // gimbal.moveYawTo(0);
    // gimbal.moveRollTo(0);
    while (1)
    {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty())
            break;

        system("clear");
        auto przesuw = detect_face(frame, cascade, 100, 600, 0.0);
        // auto przesuw = detect(frame); //wykrywanie koloru
        cv::Point kat = move(przesuw, gimbal.getCurrentPosition());
        katy(kat, gimbal);

        imshow("Frame", frame);
        // Angles angles = gimbal.getCurrentPosition();
        // cout << "Pitch: " << angles.pitch << endl
        //      << "Yaw" << angles.yaw << endl
        //      << "Roll" << angles.roll << endl;

        char c = (char)cv::waitKey(25);
        if (c == 27)
            break;
    }

    cap.release();
    cv::destroyAllWindows();

    gimbal.motorsOff();
    return 0;
}
