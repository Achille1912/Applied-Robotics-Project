#include <ros/ros.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include <stdlib.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

const double T = 1.0 / 30.0;

double q[3];      // stato attuale
double qd[3];     // posizione desiderata
double dqd[3];    // velocità desiderata

double zita = 1;
double alpha = 0.4;
double e[3];
double u1,u2;
double vd, wd;
double k1, k2, k3;


void OdomCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {

    q[0] = msg->x;
    q[1] = msg->y;
    q[2] = msg->theta;

    ROS_INFO("x=%f, y=%f, z=%f", q[0], q[1], q[2]);
}

void QdCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {

    qd[0] = msg->x;  // Posizione desiderata in telecamera (x)
    qd[1] = msg->y;  // Posizione desiderata in telecamera (y)
    qd[2] = msg->theta;  // Orientamento desiderato in telecamera (theta)
    
    ROS_INFO("Vision control qd (camera frame): (%f, %f, %f)", qd[0], qd[1], qd[2]);
    
    // Calcolo della trasformazione
    double x_robot = q[0];  // Posizione attuale del robot
    double y_robot = q[1];
    double theta_robot = q[2];

    double x_camera = qd[0];  // Posizione desiderata in coordinate della telecamera
    double y_camera = qd[1];

    // Trasformazione delle coordinate dalla telecamera al robot
    double x_robot_desired = x_robot + x_camera * cos(theta_robot) - y_camera * sin(theta_robot);
    double y_robot_desired = y_robot + x_camera * sin(theta_robot) + y_camera * cos(theta_robot);

    // Aggiornamento della posizione desiderata nel frame del robot
    qd[0] = x_robot_desired;
    qd[1] = y_robot_desired;
    
    // Calcolo dell'orientamento desiderato (tenendo conto della posizione del robot)
    qd[2] = q[2] + qd[2];  
    qd[2] = atan2(sin(qd[2]), cos(qd[2]));  // normalizza l'angolo
    ROS_INFO("Transformed qd (robot frame): (%f, %f, %f)", qd[0], qd[1], qd[2]);
}


void DqdCallback(const geometry_msgs::Twist::ConstPtr& msg) {

    dqd[0] = msg->linear.x;
    dqd[1] = msg->linear.y;
    dqd[2] = msg->angular.z;
    
    ROS_INFO("dxd=%f, dyd=%f, dzd=%f", dqd[0], dqd[1], dqd[2]);
}

void publishControl(ros::Publisher control_pub){

     vd = sqrt(pow(dqd[0],2) + pow(dqd[1],2));
     wd = dqd[2];


     k1 = 2 * zita * alpha;
     k3 = 2 * zita * alpha;
     k2 = std::min((pow(alpha, 2) - pow(wd, 2)) / std::max(vd, 0.05), 10.0);


     e[0] = cos(q[2]) * (qd[0] - q[0]) + sin(q[2]) * (qd[1] - q[1]);
     e[1] = -sin(q[2]) * (qd[0] - q[0]) + cos(q[2]) * (qd[1] - q[1]);
     e[2] = atan2(sin(qd[2] - q[2]), cos(qd[2] - q[2]));


     u1 = -k1 * e[0];
     u2 = -k2 * e[1] - k3 * e[2];

     geometry_msgs::Twist msg;

     msg.linear.x = vd*cos(e[2])-u1;
     msg.angular.z = wd - u2;	

     control_pub.publish(msg);

     ROS_INFO_STREAM("Control:"<<" v="<<msg.linear<<" w="<<msg.angular);

}

int main(int argc, char **argv){

     ros::init(argc, argv, "control");
     ros::NodeHandle n;

     ros::Publisher control_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000); 

     ros::Subscriber odomSub=n.subscribe<geometry_msgs::Pose2D>("/odometry",10,OdomCallback);
     ros::Subscriber qdSub = n.subscribe("/qd", 10, QdCallback);
     ros::Subscriber dqdSub = n.subscribe("/dqd", 10, DqdCallback);
    
    
     ros::Rate loop_rate(30);


     while (ros::ok())
     {

	ros::spinOnce();
	publishControl(control_pub);
	loop_rate.sleep();
     }
     
     return 0;
}
