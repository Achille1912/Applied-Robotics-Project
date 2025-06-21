#include <ros/ros.h>
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/JointState.h"
#include <stdlib.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"


const double r = 0.033;      // raggio della ruota
const double d = 0.288;      // distanza tra le ruote
const double T = 1.0 / 30.0; // Periodo del loop

double last_theta_r = 0.0, last_theta_l = 0.0; // valori di theta precedenti
bool first = true;        // condizione prima iterazione
double v = 0.0, w = 0.0;  // velocità attuali

double q[3] = {0,0,0};        // Posizione iniziale

void JointCallback(const sensor_msgs::JointState::ConstPtr& msg) {

    double theta_r = msg->position[0];
    double theta_l = msg->position[1];

    if (first) {
        last_theta_r = theta_r;
        last_theta_l = theta_l;
        first = false;
    }

    double d_theta_r = theta_r - last_theta_r;
    double d_theta_l = theta_l - last_theta_l;

    last_theta_r = theta_r;
    last_theta_l = theta_l;

    v = (d_theta_r + d_theta_l)/T * (r / 2.0);
    w = (d_theta_r - d_theta_l)/T*(r / d);

    ROS_INFO("v=%f, w=%f", v, w);
}


void publishOdometry(ros::Publisher odometry_pub) {

     q[0] = q[0] + v * cos(q[2]) * T;
     q[1] = q[1] + v * sin(q[2]) * T;
     q[2] = q[2] + w * T;

     geometry_msgs::Pose2D msg;
     msg.x = q[0];
     msg.y = q[1];
     msg.theta = q[2];

     odometry_pub.publish(msg);
     ROS_INFO_STREAM("Odometry localization: x=" << msg.x << " y=" << msg.y << " theta=" << msg.theta);
}


int main(int argc, char **argv){

     ros::init(argc, argv, "odometry");
     ros::NodeHandle n;
     ros::Publisher odometry_pub = n.advertise<geometry_msgs::Pose2D>("/odometry", 1000); 
     ros::Rate loop_rate(30); 

     ros::Subscriber jointSub; 
     jointSub=n.subscribe<sensor_msgs::JointState>("/joint_states",10,JointCallback);   


     while (ros::ok())
     {
	ros::spinOnce();
	publishOdometry(odometry_pub);
	loop_rate.sleep();
     }
     
     return 0;

}
