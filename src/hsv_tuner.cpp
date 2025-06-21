#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

using namespace cv;

// Parametri HSV per rosso e giallo
int h_min = 0, s_min = 100, v_min = 100;
int h_max = 10, s_max = 255, v_max = 255;
int h2_min = 160, h2_max = 180;
int yellow_h_min = 20, yellow_h_max = 30;
int show_red = 1, show_yellow = 1;

// Parametri warping 
int top_x, top_y, bottom_x, bottom_y; // Coordinate per la trasformazione prospettica
void nothing(int, void*) {}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (...) {
        ROS_ERROR("cv_bridge exception");
        return;
    }

    Mat frame = cv_ptr->image;
    int width = frame.cols, height = frame.rows;

    // Padding
    int border_size = 300;
    Mat padded_frame;
    copyMakeBorder(frame, padded_frame, border_size, border_size, border_size, border_size,
                   BORDER_CONSTANT, Scalar(0, 0, 0));

    std::vector<Point2f> src_points = {
        Point2f(width - top_x + border_size, top_y + border_size),
        Point2f(top_x + border_size, top_y + border_size),
        Point2f(bottom_x + border_size, bottom_y + border_size),
        Point2f(width - bottom_x + border_size, bottom_y + border_size)
    };

    int warp_width = 500, warp_height = 400;

    std::vector<Point2f> dst_points = {
        Point2f(0, 0),
        Point2f(warp_width, 0),
        Point2f(warp_width, warp_height),
        Point2f(0, warp_height)
    };

    Mat H = getPerspectiveTransform(src_points, dst_points);
    Mat warped;
    warpPerspective(padded_frame, warped, H, Size(warp_width, warp_height));
    warped=frame;
    // Conversione in HSV
    Mat hsv;
    cvtColor(warped, hsv, COLOR_BGR2HSV);

    // Maschere rosso/giallo
    Mat red1, red2, red_mask, yellow_mask, combined_mask;
    inRange(hsv, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), red1);
    inRange(hsv, Scalar(h2_min, s_min, v_min), Scalar(h2_max, s_max, v_max), red2);
    bitwise_or(red1, red2, red_mask);
    inRange(hsv, Scalar(yellow_h_min, s_min, v_min), Scalar(yellow_h_max, s_max, v_max), yellow_mask);

    if (show_red && show_yellow)
        bitwise_or(red_mask, yellow_mask, combined_mask);
    else if (show_red)
        combined_mask = red_mask;
    else if (show_yellow)
        combined_mask = yellow_mask;
    else
        combined_mask = Mat::zeros(warped.size(), CV_8UC1);

    // Visualizzazioni
    imshow("frame", frame);
    imshow("Mask", combined_mask);
    imshow("warped", warped);
    waitKey(1);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "hsv_calibration_gui");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/image", 1, imageCallback);

    nh.param("calibration_params/top_x", top_x, 259);  
    nh.param("calibration_params/top_y", top_y, 137);
    nh.param("calibration_params/bottom_x", bottom_x, 551);
    nh.param("calibration_params/bottom_y", bottom_y, 239);
    
    namedWindow("Mask", WINDOW_AUTOSIZE);
    namedWindow("Warped", WINDOW_AUTOSIZE);
    namedWindow("Trackbars", WINDOW_NORMAL);

    // Trackbar per HSV
    createTrackbar("H min 1", "Trackbars", &h_min, 180, nothing);
    createTrackbar("H max 1", "Trackbars", &h_max, 180, nothing);
    createTrackbar("H min 2", "Trackbars", &h2_min, 180, nothing);
    createTrackbar("H max 2", "Trackbars", &h2_max, 180, nothing);
    createTrackbar("S min", "Trackbars", &s_min, 255, nothing);
    createTrackbar("S max", "Trackbars", &s_max, 255, nothing);
    createTrackbar("V min", "Trackbars", &v_min, 255, nothing);
    createTrackbar("V max", "Trackbars", &v_max, 255, nothing);
    createTrackbar("Show Red", "Trackbars", &show_red, 1, nothing);
    createTrackbar("Y H min", "Trackbars", &yellow_h_min, 180, nothing);
    createTrackbar("Y H max", "Trackbars", &yellow_h_max, 180, nothing);
    createTrackbar("Show Yellow", "Trackbars", &show_yellow, 1, nothing);

    ros::spin();
    destroyAllWindows();
    return 0;
}

