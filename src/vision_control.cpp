// =============================
// INCLUDES
// =============================

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <deque>
#include <cmath>
#include <std_msgs/Float32.h>


// =============================
// GLOBAL VARIABLES
// =============================

ros::Publisher qd_pub;
ros::Publisher dqd_pub;

double corsia_larghezza = -1.0;
double smooth_error=0;
int top_x, top_y, bottom_x, bottom_y;

float velocity_factor = 1.0;

cv::Mat last_frame;
bool new_image = false;
sensor_msgs::ImageConstPtr last_msg;
bool stop=false;

// =============================
// UTILITY FUNCTIONS
// =============================

// Filtra i contorni in base ad area, aspect ratio e circolarità (per i semafori)
cv::Mat filterByShape(const cv::Mat& cropped, const cv::Mat& mask) {

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat filtered = cv::Mat::zeros(mask.size(), CV_8UC1);

    for (const auto& contour : contours) {
        cv::Mat white_mask;
        cv::inRange(cropped, cv::Scalar(138, 5, 67), cv::Scalar(180, 136, 189), white_mask);
    	cv::Rect bbox=cv::boundingRect(contour);
    	cv::Rect safeBbox = bbox & cv::Rect(0,0,white_mask.cols,white_mask.rows);
    	
    	if(!safeBbox.empty()){
    		cv::Mat Roi = white_mask(safeBbox);
    		int white_pixels=cv::countNonZero(Roi);
    		if(white_pixels>10){
    			continue;
    		}
    	}
        double area = cv::contourArea(contour);
        if (area < 50) continue;

        double aspect_ratio = static_cast<double>(bbox.width) / bbox.height;
        if (aspect_ratio < 0.5 || aspect_ratio > 1.5) continue;

        double perimeter = cv::arcLength(contour, true);
        double circularity = 4 * CV_PI * area / (perimeter * perimeter + 1e-6);
        if (circularity < 0.6) continue;

        cv::drawContours(filtered, std::vector<std::vector<cv::Point>>{contour}, -1, 255, cv::FILLED);
    }

    return filtered;
}


// Media pesata degli errori recenti per smoothing
double getSmoothedError(double new_error) {

    static std::deque<double> error_history;
    const int window_size = 3;
    const double alpha = 0.8; // più vicino a 0 = più peso al recente

    error_history.push_back(new_error);
    if (error_history.size() > window_size) {
        error_history.pop_front();
    }

    double weighted_sum = 0.0;
    double total_weight = 0.0;
    int n = error_history.size();
    for (int i = 0; i < n; ++i) {
    
        // peso maggiore per i più recenti
        int age = n - i - 1; 
        double weight = std::pow(alpha, age);
        weighted_sum += weight * error_history[i];
        total_weight += weight;
    }

    return weighted_sum / total_weight;
}


// Calcola la media pesata delle colonne con pixel attivi in una riga
int getLineX(const cv::Mat& mask, int row, int col_start, int col_end) {

    if (row < 0 || row >= mask.rows) return -1;
    col_start = std::max(0, col_start);
    col_end = std::min(mask.cols, col_end);

    const uchar* ptr = mask.ptr<uchar>(row);
    double weighted_sum = 0.0;
    double total = 0.0;

    for (int col = col_start; col < col_end; ++col) {
        if (ptr[col] > 0) {
            weighted_sum += col;
            total += 1.0;
        }
    }

    if (total > 0)
        return static_cast<int>(weighted_sum / total);
    else
        return -1;
}

// Media di getLineX su più righe attorno a center_row
int getLineXMean(const cv::Mat& mask, int center_row, int band_half_height, int col_start, int col_end) {

    int start_row = std::max(0, center_row - band_half_height);
    int end_row = std::min(mask.rows - 1, center_row + band_half_height);

    double sum = 0.0;
    int count = 0;

    for (int row = start_row; row <= end_row; ++row) {
        int x = getLineX(mask, row, col_start, col_end);
        if (x != -1) {
            sum += x;
            count++;
        }
    }

    if (count > 0)
        return static_cast<int>(sum / count);
    else
        return -1;  // Nessuna riga utile
}


// =============================
// SEMAFORI
// =============================

// Rilevamento del colore del semaforo e aggiornamento del fattore di velocità
void Traffic_lights() {

    if (!new_image) return;
    new_image = false;

    cv::Mat frame = last_frame;
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // Definizione ROI (ignoriamo la parte superiore)
    int x_start = 0;
    int y_start = 100;
    int width = frame.cols;
    int height = frame.rows-100;
    cv::Rect roi(x_start, y_start, width, height);
    cv::Mat cropped = hsv(roi);

    // Maschere per i colori del semaforo
    cv::Mat red_mask, yellow_mask, green_mask,white_mask;
    cv::inRange(cropped, cv::Scalar(139, 27, 75), cv::Scalar(180, 255, 255), red_mask);
    cv::inRange(cropped, cv::Scalar(0, 82, 136), cv::Scalar(55, 255, 255), yellow_mask);
    cv::inRange(cropped, cv::Scalar(40, 100, 100), cv::Scalar(90, 255, 255), green_mask);

    
    // Filtro dei blob in base alla forma
    red_mask = filterByShape(cropped,red_mask);
    yellow_mask = filterByShape(cropped,yellow_mask);
    green_mask = filterByShape(cropped,green_mask);
    
    
    double red_pixels = cv::countNonZero(red_mask);
    double yellow_pixels = cv::countNonZero(yellow_mask);
    double green_pixels = cv::countNonZero(green_mask);
    double white_pixels = cv::countNonZero(white_mask);
    
    
    
    if(!stop){
        velocity_factor = 1.0;
	    if (red_pixels > 10) {
		velocity_factor = 0.0;
		 ROS_INFO("Semaforo ROSSO");
	    } else if (yellow_pixels > 10) {
		velocity_factor = 0.5;
		ROS_INFO("Semaforo GIALLO");
	    } else if (green_pixels > 10) {
		velocity_factor = 1.0;
		ROS_INFO("Semaforo VERDE");
	    }
    }


    // Visualizzazione delle maschere
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

    cv::imshow("Traffic Light Masks", debug);
    cv::waitKey(1);
}


// =============================
// CARTELLI
// =============================

// Rilevamento del cartello
void Traffic_sign() {
    
    if (!new_image) return;

    cv::Mat frame = last_frame;
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // Definizione ROI 
    int x_start = 50;
    int y_start = 0;
    int width = frame.cols-100;
    int height = frame.rows-100;
    cv::Rect roi(x_start, y_start, width, height);
    cv::Mat cropped = hsv(roi);

    // Maschere per i colori del cartello
    cv::Mat red_mask, white_mask;
    cv::inRange(cropped, cv::Scalar(161, 146, 123), cv::Scalar(180, 255, 255), red_mask);
    cv::inRange(cropped, cv::Scalar(138, 5, 67), cv::Scalar(180, 136, 189), white_mask);
   
    
    double red_pixels = cv::countNonZero(red_mask);
    double white_pixels = cv::countNonZero(white_mask);
    
    ROS_INFO("red=%.2f, bianchi=%.2f", red_pixels, white_pixels);
    if(white_pixels>1500 && red_pixels > 2500){
		velocity_factor = 0.0;
		stop=true;
		ROS_INFO("STOP");
    }
    

    // Visualizzazione delle maschere
    cv::Mat red_bgr, white_bgr;
    cv::cvtColor(red_mask, red_bgr, cv::COLOR_GRAY2BGR);
    cv::cvtColor(white_mask, white_bgr, cv::COLOR_GRAY2BGR);

    red_bgr.setTo(cv::Scalar(0, 0, 255), red_mask);
    white_bgr.setTo(cv::Scalar(255, 255, 255), white_mask);

    cv::Mat debug;
    cv::addWeighted(red_bgr, 1.0, white_bgr, 1.0, 0.0, debug);

    cv::imshow("Traffic sign Masks", debug);
    cv::waitKey(1);
}


// =============================
// PROCESSING PRINCIPALE
// =============================

void processFrame() {

    if (!new_image) return;
    
    // Conversione ROS -> OpenCV
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(last_msg, sensor_msgs::image_encodings::BGR8);
    } catch (...) {
        ROS_ERROR("cv_bridge exception");
        return;
    }

    cv::Mat frame = cv_ptr->image;
    last_frame=frame;
    int width = frame.cols, height = frame.rows;
	
    // Padding dell'immagine
    int border_size = 300;
    cv::Mat padded_frame;
    cv::copyMakeBorder(frame, padded_frame, border_size, border_size, border_size, border_size, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    // Warping prospettico
    std::vector<cv::Point2f> src_points = {
        cv::Point2f(width - top_x + border_size, top_y + border_size),
        cv::Point2f(top_x + border_size, top_y + border_size),
        cv::Point2f(bottom_x + border_size, bottom_y + border_size),
        cv::Point2f(width - bottom_x + border_size, bottom_y + border_size)
    };

    // Dimensione dell'immagine warped
    int warp_width = 500;
    int warp_height = 400;
    
    std::vector<cv::Point2f> dst_points = {
        cv::Point2f(0, 0),
        cv::Point2f(warp_width, 0),
        cv::Point2f(warp_width, warp_height),
        cv::Point2f(0, warp_height)
    };
    
    cv::Mat H = cv::getPerspectiveTransform(src_points, dst_points);
    cv::Mat warped;
    cv::warpPerspective(padded_frame, warped, H, cv::Size(warp_width, warp_height));
    
    // Conversione in HSV per il masking
    cv::Mat hsv;
    cv::cvtColor(warped, hsv, cv::COLOR_BGR2HSV);

    // Mask per rosso e giallo
    cv::Mat red1, red2, red_mask, yellow_mask;
    cv::inRange(hsv, cv::Scalar(0, 54, 118), cv::Scalar(33, 255, 245), red1);      
    cv::inRange(hsv, cv::Scalar(122, 54, 118), cv::Scalar(180, 255, 245), red2);   
    cv::bitwise_or(red1, red2, red_mask);
    cv::inRange(hsv, cv::Scalar(0, 0, 172), cv::Scalar(180, 20, 255), yellow_mask);  

    // Operazioni morfologiche 
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

    cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(yellow_mask, yellow_mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(yellow_mask, yellow_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel);
   
    
    // Rilevamento centro corsia
    int band_half_height = 2;
    int max_row = warped.rows - 1;
    int half_height = warped.rows / 2;

    int best_row = -1;
    int red_cx = -1;
    int yellow_cx = -1;

    // Prima cerca la riga più bassa con entrambe le linee
    for (int row = warped.rows-50; row >= half_height+100; row=row-4) {
    // Solo metà destra per i gialli
        int yellow_x = getLineXMean(yellow_mask, row, band_half_height, warped.cols / 2, warped.cols);

	// Solo metà sinistra per i rossi
	int red_x = getLineXMean(red_mask, row, band_half_height, 0, warped.cols / 2);
	
	if (red_x != -1 && yellow_x != -1) {
            best_row = row;
	    red_cx = red_x;
	    yellow_cx = yellow_x;
	    break;
	}
    }

    // Se non trovata, cerca la riga più bassa entro metà immagine con almeno una linea
    if (best_row == -1) {
        for (int row = warped.rows-50; row >= half_height+100; row=row-4) {
            
            int red_x = getLineXMean(red_mask, row, band_half_height, 0, warped.cols / 2);
            int yellow_x = getLineXMean(yellow_mask, row, band_half_height, warped.cols / 2, warped.cols);
		
	    if (red_x != -1 || yellow_x != -1) {
		 best_row = row;
		 red_cx = red_x;
	         yellow_cx = yellow_x;
	         break;
	    }
        } 
    }

    int scan_row = best_row;
    int lane_center = -1;

    // Calcolo centro corsia
    if (red_cx >= 0 && yellow_cx >= 0) {
    
        lane_center = (red_cx + yellow_cx) / 2;
        
        if ((yellow_cx - red_cx) > 80) {
               corsia_larghezza = (yellow_cx - red_cx);
    	}
    	
    } else if (yellow_cx >= 0 && corsia_larghezza > 0) {
        lane_center = yellow_cx - static_cast<int>(corsia_larghezza / 2);
        
    } else if (red_cx >= 0 && corsia_larghezza > 0) {
        lane_center = red_cx + static_cast<int>(corsia_larghezza / 2);
    }

    // Se è stato trovato un centro corsia valido
    if (lane_center != -1) {
    
        // Trasformazione inversa (da warped a camera frame)
        cv::Point2f warped_point(static_cast<float>(lane_center), static_cast<float>(warp_height / 2));
        std::vector<cv::Point2f> warped_pts = {warped_point};
        std::vector<cv::Point2f> cam_pts_padded;
        cv::Mat H_inv = H.inv();
        cv::perspectiveTransform(warped_pts, cam_pts_padded, H_inv);
        cv::Point2f cam_point = cam_pts_padded[0];
        cam_point.x -= border_size;
        cam_point.y -= border_size;

        // Calcolo errore normalizzato
        double desired_x_camera = cam_point.x / static_cast<double>(width);
        double raw_error_camera = (cam_point.x - width / 2.0) / (width / 2.0);
        
        if (velocity_factor!=0){
        	smooth_error = getSmoothedError(raw_error_camera);
	}
        
        // Pubblica qd e dqd
        geometry_msgs::Pose2D qd_msg;
        qd_msg.x = 0;
        qd_msg.y = 0;
        qd_msg.theta = -smooth_error*velocity_factor;
        qd_pub.publish(qd_msg);

        geometry_msgs::Twist dqd_msg;
        dqd_msg.linear.x = 0.26 * velocity_factor;
        dqd_msg.angular.z = -0.2 * smooth_error *velocity_factor; 
        dqd_pub.publish(dqd_msg);

        ROS_INFO("Errore raw=%.2f, smooth=%.2f, angular=%.2f", raw_error_camera, smooth_error, dqd_msg.angular.z);
    } else {
        // Nessuna linea trovata, ruota nella direzione dell'errore
        geometry_msgs::Pose2D qd_msg;
        qd_msg.x = 0;
        qd_msg.y = 0;
        qd_msg.theta = -smooth_error*velocity_factor;
        qd_pub.publish(qd_msg);

        geometry_msgs::Twist dqd_msg;
        dqd_msg.linear.x = 0;
        dqd_msg.angular.z = -0.2*smooth_error*velocity_factor;
        dqd_pub.publish(dqd_msg);

        ROS_WARN("Nessuna linea trovata!");
    }

    // Debug visivo
    cv::Mat debug;
    cv::addWeighted(red_mask, 1.0, yellow_mask, 1.0, 0.0, debug);
    if (red_cx >= 0)
        cv::circle(warped, cv::Point(red_cx, best_row), 4, cv::Scalar(0, 0, 255), -1);
    if (yellow_cx >= 0)
        cv::circle(warped, cv::Point(yellow_cx, best_row), 4, cv::Scalar(0, 255, 255), -1);
    if (lane_center != -1)
        cv::circle(warped, cv::Point(lane_center, best_row), 5, cv::Scalar(0, 255, 0), -1);
  
    cv::imshow("Warped Image", warped);
    cv::imshow("Masks", debug);
    cv::waitKey(1);
 
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    last_msg = msg;
    new_image = true;
    
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "vision_control");
    ros::NodeHandle nh;
    ros::NodeHandle velocity_nh;
    
    // Carica i parametri
    nh.param("calibration_params/top_x", top_x, 262);  
    nh.param("calibration_params/top_y", top_y, 162);
    nh.param("calibration_params/bottom_x", bottom_x, 551);
    nh.param("calibration_params/bottom_y", bottom_y, 239);
    
    ros::Subscriber image_sub = nh.subscribe("/camera/image", 1, imageCallback);
    //ros::Subscriber factor_sub = velocity_nh.subscribe("/velocity_factor", 1, factorCallback);

    qd_pub = nh.advertise<geometry_msgs::Pose2D>("/qd", 1);
    dqd_pub = nh.advertise<geometry_msgs::Twist>("/dqd", 1);

   ros::Rate rate(30); 

    while (ros::ok()) {
        ros::spinOnce();
        processFrame();
        Traffic_sign();
        Traffic_lights();
        rate.sleep();
    }

    return 0;
}







