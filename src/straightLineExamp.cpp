/**
 *  Created on: September 2019
 *  Author: jguerrero
 *  Example of odometry, laser and command vel topics
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "stdio.h"
#include "assert.h"
#include "math.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_datatypes.h>

#include "nav_msgs/Odometry.h"


#include <string>
#include <sstream>
#include <iostream>

#define CRIT_DIST 1.0

using namespace std;
using std::ostringstream;


ros::Publisher cmd_vel_pub; // publisher for movement commands
ros::Time start;
float codeVelMax;

/**
  Read the the Laser data. If there is an obstacle close r the robot, stop and turn
  otherwise move ahead. 
*/
void callbackLaser(const sensor_msgs::LaserScan& most_intense) {
	  int length = most_intense.ranges.size(); 	
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = codeVelMax;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    for(int i=0; i<length; i++){
    	if(most_intense.ranges[i] < CRIT_DIST) {
    	  cmd_vel.linear.x = 0.0;
    		cmd_vel.angular.z = 1.0;
        break;
    	}
    }
    cmd_vel_pub.publish(cmd_vel);
    ros::spinOnce(); 
}


/***
Print the odometry location.
***/
void callbackOdom(const nav_msgs::Odometry odom) {

	double Xr = odom.pose.pose.position.x;
	double Yr = odom.pose.pose.position.y;
	double current_z=odom.pose.pose.orientation.z;
    double current_w=odom.pose.pose.orientation.w;
	double orientationr=(2.0*atan2(current_z,current_w));
	//ROS_INFO("Odom X=%lf, Y=%lf, ori=%lf\n", Xr, Yr,orientationr);
}


int main(int argc, char **argv) {



	ros::init(argc, argv, "robot_control");
	ros::Time::init();

	ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  nh_private.param<float>("velMax", codeVelMax, 1.0);
  ROS_INFO("VMAX=%lf", codeVelMax);




   //Build a string with the odom topoic
    string odom_topic_name = "robot_";
	odom_topic_name += "0";
	odom_topic_name += "/odom";
	// subscribe to robot's odom topic "robot_X/base_scan"
	ros::Subscriber odom_sub = nh.subscribe(odom_topic_name, 10, callbackOdom);


     // subscribe to robot's laser scan topic "robot_X/base_scan"
	string sonar_scan_topic_name = "robot_";
	sonar_scan_topic_name += "0";
	sonar_scan_topic_name += "/base_scan_1";
	ros::Subscriber sub = nh.subscribe(sonar_scan_topic_name, 1, callbackLaser);

    string cmd_vel_topic_name = "robot_";
	cmd_vel_topic_name += "0";
	cmd_vel_topic_name += "/cmd_vel";
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);
    


	start = ros::Time::now();
	ros::spin();
}

