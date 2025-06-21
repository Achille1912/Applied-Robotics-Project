#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <std_msgs/Float32.h>
#include <ros/package.h>


cv::Mat current_frame;
int top_x = 200;
int top_y = 100;
int bottom_x = 440;
int bottom_y = 380;


void onTrack(int, void*) {

    // Salva i parametri quando vengono modificati
    YAML::Node config;
    config["top_x"] = top_x;
    config["top_y"] = top_y;
    config["bottom_x"] = bottom_x;
    config["bottom_y"] = bottom_y;

    // Salva i parametri in un file YAML
    std::ofstream fout("/src/line_follower/config/calibration_params.yaml");
    fout << config;
    fout.close();
}

void loadParameters() {

    // Carica i parametri dal file YAML se esiste
    try {
        std::string path = ros::package::getPath("line_follower") + "/config/calibration_params.yaml";
        YAML::Node config = YAML::LoadFile(path);

        if (config["top_x"]) top_x = config["top_x"].as<int>();
        if (config["top_y"]) top_y = config["top_y"].as<int>();
        if (config["bottom_x"]) bottom_x = config["bottom_x"].as<int>();
        if (config["bottom_y"]) bottom_y = config["bottom_y"].as<int>();

        ROS_INFO("Loaded calibration parameters from file.");
    } catch (const std::exception& e) {
        ROS_WARN("Failed to load parameters: %s. Using default values.", e.what());
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        current_frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        int width = current_frame.cols;
        int height = current_frame.rows;

        // Padding nero
        int border_size = 300;
        cv::Mat padded_frame;
        cv::copyMakeBorder(current_frame, padded_frame, border_size, border_size, border_size, border_size, 
                           cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        // Coordinate dei punti per la regione da proiettare (nell'immagine originale)
        std::vector<cv::Point2f> src_points = {
            cv::Point2f(width - top_x, top_y),
            cv::Point2f(top_x, top_y),
            cv::Point2f(bottom_x, bottom_y),
            cv::Point2f(width - bottom_x, bottom_y)
            
        };

        // Aggiusta le coordinate per il padding
        std::vector<cv::Point2f> padded_src_points;
        for (const auto& p : src_points) {
            padded_src_points.push_back(cv::Point2f(p.x + border_size, p.y + border_size));
        }

        // Dimensione dell'immagine warped
        int warp_width = 500;
        int warp_height = 400;

        // Punti di destinazione (rettangolo)
        std::vector<cv::Point2f> dst_points = {
            cv::Point2f(0, 0),
            cv::Point2f(warp_width, 0),
            cv::Point2f(warp_width, warp_height),
            cv::Point2f(0, warp_height)
        };

        // Calcola la matrice di omografia
        cv::Mat H = cv::getPerspectiveTransform(padded_src_points, dst_points);

        // Applica l'omografia all'immagine con padding
        cv::Mat warped;
        cv::warpPerspective(padded_frame, warped, H, cv::Size(warp_width, warp_height), 
                           cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        // Visualizza l'immagine originale con il trapezio tracciato
        cv::Mat display;
        padded_frame.copyTo(display);

        // Disegna il trapezio sull'immagine
        if (!padded_src_points.empty()) {
            std::vector<cv::Point> padded_src_points_int;
            for (const auto& p : padded_src_points) {
                padded_src_points_int.push_back(cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)));
            }
            cv::polylines(display, padded_src_points_int, true, cv::Scalar(0, 255, 0), 2);
        }

        // Mostra l'immagine originale con il trapezio
        cv::imshow("Projection Calibration", display);

        // Mostra l'immagine "warped"
        cv::imshow("Warped Image", warped);

        cv::waitKey(1);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "projection_calibration_gui");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/image", 1, imageCallback);

    // Carica i parametri prima di iniziare
    loadParameters();

    cv::namedWindow("Projection Calibration");
    cv::createTrackbar("Top X", "Projection Calibration", &top_x, 1500, onTrack);
    cv::createTrackbar("Top Y", "Projection Calibration", &top_y, 1000, onTrack);
    cv::createTrackbar("Bottom X", "Projection Calibration", &bottom_x, 1500, onTrack);
    cv::createTrackbar("Bottom Y", "Projection Calibration", &bottom_y, 1000, onTrack);

    ROS_INFO("Projection calibration GUI started.");
    ros::spin();
    cv::destroyAllWindows();
    return 0;
}

