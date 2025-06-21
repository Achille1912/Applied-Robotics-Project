#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

ros::Publisher factor_pub;
ros::Time last_processed_time;

double process_interval=0.5;

// Funzione per filtrare le maschere in base alla forma
cv::Mat filterByShape(const cv::Mat& mask) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat filtered = cv::Mat::zeros(mask.size(), CV_8UC1);

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < 50) continue;

        cv::Rect bbox = cv::boundingRect(contour);
        double aspect_ratio = static_cast<double>(bbox.width) / bbox.height;
        if (aspect_ratio < 0.5 || aspect_ratio > 1.5) continue;

        double perimeter = cv::arcLength(contour, true);
        double circularity = 4 * CV_PI * area / (perimeter * perimeter + 1e-6);
        if (circularity < 0.6) continue;

        cv::drawContours(filtered, std::vector<std::vector<cv::Point>>{contour}, -1, 255, cv::FILLED);
    }

    return filtered;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

    ros::Time now=ros::Time::now();
    if((now-last_processed_time).toSec()<process_interval){
    	return;
    }
    last_processed_time=now;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (...) {
        ROS_ERROR("cv_bridge exception");
        return;
    }

    cv::Mat frame = cv_ptr->image;
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    
    // ROS_INFO("x=%.d, y=%.d", hsv.rows, hsv.cols);

    int x_start = 0;
    int y_start = 100;
    int width = frame.cols;
    int height = frame.rows-100;
    cv::Rect roi(x_start, y_start, width, height);
    cv::Mat cropped = hsv(roi);

    cv::Mat red1, red2, red_mask, yellow_mask, green_mask;
    cv::inRange(cropped, cv::Scalar(139, 27, 75), cv::Scalar(180, 255, 255), red1);
    //cv::inRange(cropped, cv::Scalar(255, 255, 255), cv::Scalar(255, 255, 255), red2);
    //cv::bitwise_or(red1, red2, red_mask);
    cv::inRange(cropped, cv::Scalar(0, 82, 136), cv::Scalar(55, 255, 255), yellow_mask);
    cv::inRange(cropped, cv::Scalar(40, 100, 100), cv::Scalar(90, 255, 255), green_mask);

    red_mask = filterByShape(red1);
    yellow_mask = filterByShape(yellow_mask);
    green_mask = filterByShape(green_mask);

    double red_pixels = cv::countNonZero(red_mask);
    double yellow_pixels = cv::countNonZero(yellow_mask);
    double green_pixels = cv::countNonZero(green_mask);

    std_msgs::Float32 msg_out;
    
    
    float velocity_factor = 1.0;
    if (red_pixels > 10) {
        velocity_factor = 0.0;
         ROS_INFO("Semaforo ROSSO");

    } else if (yellow_pixels > 10) {
        velocity_factor = 0.5;
        msg_out.data = velocity_factor;
    } else if (green_pixels > 10) {
        velocity_factor = 1.0;
        msg_out.data = velocity_factor;
    }


            msg_out.data = velocity_factor;
        factor_pub.publish(msg_out);

    cv::Mat red_bgr, yellow_bgr, green_bgr;
    cv::cvtColor(red_mask, red_bgr, cv::COLOR_GRAY2BGR);
    cv::cvtColor(yellow_mask, yellow_bgr, cv::COLOR_GRAY2BGR);
    cv::cvtColor(green_mask, green_bgr, cv::COLOR_GRAY2BGR);

    red_bgr.setTo(cv::Scalar(0, 0, 255), red_mask);
    yellow_bgr.setTo(cv::Scalar(0, 255, 255), yellow_mask);
    green_bgr.setTo(cv::Scalar(0, 255, 0), green_mask);

    cv::Mat debug;
    cv::addWeighted(red_bgr, 1.0, yellow_bgr, 1.0, 0.0, debug);
    cv::addWeighted(debug, 1.0, green_bgr, 1.0, 0.0, debug);

     //cv::imshow("Traffic Light Masks", debug);
     //cv::imshow("Semaforo", cropped);
     //cv::waitKey(1);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "traffic_light_detector");
    ros::NodeHandle nh;

    ros::Subscriber image_sub = nh.subscribe("/camera/image", 50, imageCallback);
    factor_pub = nh.advertise<std_msgs::Float32>("/velocity_factor", 1);
 /*
    ros::Rate loop_rate(2);  // 2 Hz

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
*/
ros::spin();
    return 0;
}

