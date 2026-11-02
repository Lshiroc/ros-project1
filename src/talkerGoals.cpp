#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include "geometry_msgs/Point.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "talkergoals");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	geometry_msgs::Point goal;	
	double T_GOAL;
	nh_private.param("X_GOAL", goal.x, 5.0);
	nh_private.param("Y_GOAL", goal.y, 5.0);
	nh_private.param("T_GOAL", T_GOAL, 5.0);

	ros::Publisher publisher = nh.advertise<geometry_msgs::Point>("/myGoals", 10);
	ros::Rate loop_rate(T_GOAL);

	while (ros::ok()) {
		publisher.publish(goal);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

