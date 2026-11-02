#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <string>
#include <cmath>
#include <tf/transform_listener.h>
#include "first_demo/moveRobot.h"
/*
ros::Publisher cmd_vel_pub;
geometry_msgs::Point goal;
// ros params
int ID_ROBOT;
double CRIT_DIST;
double V_MAX_DES;
double V_MAX_ROT;
double D_OBJ;
double K_ROT_MIN;
double K_ROT_MAX;
double ORI_ERROR;
double T_AVOID_OBS;
double DIST_LEADER;
int ROBOT_ROLE;
int ID_LEADER;
double W_1;
double W_2;
int T_WAIT;
int ALGOR;

// code specific
double ori_error{};
ros::Time time_begin = ros::Time(0);
ros::Time time_end = ros::Time(0);
bool is_intense;
bool have_odom = false;
double dist_goal = 1e9;
bool rot_left_or_right = false; // false->left, true->right
const double CONE_HALF_ANGLE = 0.35;
bool is_goal_set = false;
geometry_msgs::Vector3 V_goal;
geometry_msgs::Vector3 V_obs;
geometry_msgs::Vector3 V_f;
ros::Time last_decision = ros::Time(0);
bool have_decision = false;
double pf_ang = 0;
double pf_lin = 0;

State state = FACE_GOAL;

std::string printStateName(State stateName) {
	std::string str;
	switch(stateName) {
		case FACE_GOAL:
			str = "FACE_GOAL";
			break;
		case GO:
			str = "GO";
			break;
		case ROTATE:
			str = "ROTATE";
			break;
		case FORWARD:
			str = "FORWARD";
			break;
		case STOP:
			str = "STOP";
			break;
		case DONE:
			str = "DONE";
			break;
	}

	return str;
}

double wrapAngleInRadian(double radian)
{
	double twoPi = 2.0 * M_PI;
	return radian - twoPi * floor((radian + M_PI) / twoPi );
}


double rotVel(double e) {
	double k = (std::fabs(e) < ORI_ERROR) ? K_ROT_MIN : K_ROT_MAX;
	return k * V_MAX_ROT * e;
}

void potentialFieldStep(geometry_msgs::Twist &cmd) {
	double stop_dist = (ROBOT_ROLE == 1) ? DIST_LEADER : D_OBJ;
	if (dist_goal < stop_dist) {
		pf_lin = 0.0;
		pf_ang = 0.0;
		cmd.linear.x = 0.0;
		cmd.angular.z = 0.0;
		return;
	}
	ros::Time now = ros::Time::now();
	if (!have_decision || (now.toSec() - last_decision.toSec()) >= T_WAIT / 1000.0) {
		V_f.x = W_1*V_goal.x + W_2*V_obs.x;
		V_f.y = W_1*V_goal.y + W_2*V_obs.y;

		if (std::hypot(V_f.x, V_f.y) > 1e-6) {
			double heading_err = std::atan2(V_f.y, V_f.x);
			pf_ang = rotVel(heading_err);
			pf_lin = (std::fabs(heading_err) < ORI_ERROR) ? V_MAX_DES : 0.0;
		}	

		last_decision = now;
		have_decision = true;
	}

	cmd.linear.x  = pf_lin;
	cmd.angular.z = pf_ang;
}

// Calculate orientation error
double calcOriError(geometry_msgs::Point goal, geometry_msgs::Point robot, double yaw) {
	//return std::atan2(goal.linear.y - robot.linear.y,);
	return wrapAngleInRadian(std::atan2(goal.y - robot.y, goal.x - robot.x) - yaw);
}

// Simple obstacle avoidance
void cmdVel() {
	geometry_msgs::Twist cmd_vel;

	cmd_vel.linear.x = 0.0;
	cmd_vel.linear.y = 0.0;
	cmd_vel.linear.z = 0.0;
	cmd_vel.angular.x = 0.0;
	cmd_vel.angular.y = 0.0;
	cmd_vel.angular.z = 0.0;

	if (ALGOR == 2) {
		potentialFieldStep(cmd_vel);
	} else {
	switch(state) {
		case FACE_GOAL:
			cmd_vel.angular.z = rotVel(ori_error);
			if (std::fabs(ori_error) < ORI_ERROR) state = GO;
			break;
		case GO:
			cmd_vel.angular.z = rotVel(ori_error);
			cmd_vel.linear.x = V_MAX_DES;
			if (dist_goal < D_OBJ) state = DONE;
			break;
		case ROTATE:
			// decide which direction would be faster, and if possible remember what obstacles was on each sides of the robot.
			// for now lets just make it rotate to left
			cmd_vel.angular.z = (rot_left_or_right ? 1 : -1) * V_MAX_ROT;
			if (!is_intense) state = FORWARD;
			break;
		case FORWARD:
			if (time_begin.isZero()) {
				time_begin = ros::Time::now();			
			} else {
				time_end = ros::Time::now();
				if (time_end.toSec() - time_begin.toSec() < T_AVOID_OBS) {
					cmd_vel.linear.x = V_MAX_DES;				
				} else {
					state = FACE_GOAL;
					time_begin = ros::Time(0);
					time_end = ros::Time(0);			
				}

				//ROS_INFO("end: %lf, begin: %lf, sub: %lf, t_avoid: %lf", time_end.toSec(), time_begin.toSec(), time_end.toSec() - time_begin.toSec(), T_AVOID_OBS);
			}
			break;
		case STOP:
			if (dist_goal >= DIST_LEADER) state = GO;
			break;
		case DONE:
			ROS_INFO_ONCE("Goal reached");
			break;
	}
	}
		

	cmd_vel_pub.publish(cmd_vel);
}

void callbackGoal(const geometry_msgs::Point::ConstPtr& p) {
	goal.x = p->x;
	goal.y = p->y;
	is_goal_set = true;
}

double repulsion(double d) {
	return (CRIT_DIST - d) / CRIT_DIST;
}


//  Read the the Laser data. If there is an obstacle close r the robot, stop and turn
// otherwise move ahead. 

void callbackLaser(const sensor_msgs::LaserScan& scan) {
	double lowest = 1e9;
	double lowest_angle = 0;
	double sx = 0;
	double sy = 0;
	for (size_t i = 0; i < scan.ranges.size(); i++) {
		double r = scan.ranges[i];
		if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;
		double angle = scan.angle_min + i * scan.angle_increment;
		if (std::fabs(angle) > CONE_HALF_ANGLE) continue;
		if (r < lowest) { lowest = r; lowest_angle = angle; }

		if (ALGOR == 2 && r < CRIT_DIST) {
			double mag = repulsion(r);
			sx += mag * (-std::cos(angle));
			sy += mag * (-std::sin(angle));
		}
	}
	if (ALGOR=2) {
		V_obs.x = sx;
		V_obs.y = sy;
	}

	if (ALGOR=1) {
		is_intense = (lowest < CRIT_DIST);
		if (is_intense && state != DONE) {
			if (state != ROTATE) {
				rot_left_or_right = (lowest_angle < 0);
				time_begin = ros::Time(0);
			}
			state = ROTATE;
		}	
	}
}


// Print the odometry location.
void callbackOdom(const nav_msgs::Odometry odom) {
	double Xr = odom.pose.pose.position.x;
	double Yr = odom.pose.pose.position.y;
	double current_z=odom.pose.pose.orientation.z;
	double current_w=odom.pose.pose.orientation.w;
	double orientationr=(2.0*atan2(current_z,current_w));
	double ori = tf::getYaw(odom.pose.pose.orientation);
	// print the state as well
	//ROS_INFO("State: %s, Odom X=%lf, Y=%lf, ori=%lf, ori2=%lf\n", printStateName(state).c_str(), Xr, Yr,orientationr, ori);
	
	if (is_goal_set) {
		ori_error = calcOriError(goal, odom.pose.pose.position, tf::getYaw(odom.pose.pose.orientation));
		V_goal.x = std::cos(ori_error);
		V_goal.y = std::sin(ori_error);
		//ROS_INFO("ORI_ERROR: %lf\n", ori_error);
		dist_goal = std::hypot(goal.x - odom.pose.pose.position.x,
		               goal.y - odom.pose.pose.position.y);
		have_odom = true;
		if (ROBOT_ROLE == 1 && dist_goal < DIST_LEADER) {
			state = STOP;
		}
	}
}

void callbackLeadOdom(const nav_msgs::Odometry odom) {
	goal.x = odom.pose.pose.position.x;
	goal.y = odom.pose.pose.position.y;
	is_goal_set = true;
}

void readParams() {
	ros::NodeHandle nh_private("~");
	nh_private.param("ID_ROBOT", ID_ROBOT, 0);
	nh_private.param("CRIT_DIST", CRIT_DIST, 1.0);
	nh_private.param("V_MAX_DES", V_MAX_DES, 1.0);
	nh_private.param("V_MAX_ROT", V_MAX_ROT, 1.0);
	nh_private.param("D_OBJ", D_OBJ, 0.2);
	nh_private.param("K_ROT_MIN", K_ROT_MIN, 0.1);
	nh_private.param("K_ROT_MAX", K_ROT_MAX, 0.25);
	nh_private.param("ORI_ERROR", ORI_ERROR, 0.45);
	nh_private.param("T_AVOID_OBS", T_AVOID_OBS, 2.0);
	nh_private.param("DIST_LEADER", DIST_LEADER, 2.0);
	nh_private.param("ROBOT_ROLE", ROBOT_ROLE, 0);
	nh_private.param("ID_LEADER", ID_LEADER, -1);
	nh_private.param("W_1", W_1, 1.5);
	nh_private.param("W_2", W_2, 3.0);
	nh_private.param("T_WAIT", T_WAIT, 150);
	nh_private.param("ALGOR", ALGOR, 1);
}*/

int main(int argc, char **argv) {
	/*ros::init(argc, argv, "moveRobot");
	ros::NodeHandle nh;
	ros::Rate r(50);
	ros::Subscriber goal_sub;
	ros::Subscriber lead_odom_sub;
	// subscribe to the myGoals topic
	readParams();
	if (ROBOT_ROLE == 0) {
		goal_sub = nh.subscribe("/myGoals", 10, callbackGoal);
	}

	//Build a string with the odom topic
	std::string odom_topic_name = "robot_";
	odom_topic_name += std::to_string(ID_ROBOT);
	odom_topic_name += "/odom";
	// subscribe to robot's odom topic "robot_X/base_scan"
	ros::Subscriber odom_sub = nh.subscribe(odom_topic_name, 10, callbackOdom);

	if (ROBOT_ROLE == 1) {
		std::string lead_odom_topic_name = "robot_";
		lead_odom_topic_name += std::to_string(ID_LEADER);
		lead_odom_topic_name += "/odom";
		lead_odom_sub = nh.subscribe(lead_odom_topic_name, 10, callbackLeadOdom);
	}

	// Subscribe to the robot laser topic
	std::string sonar_scan_topic_name = "robot_";
	sonar_scan_topic_name += std::to_string(ID_ROBOT);
	sonar_scan_topic_name += "/base_scan_1";
	ros::Subscriber sub = nh.subscribe(sonar_scan_topic_name, 1, callbackLaser);

	// publish robot cmd_vel topic
	std::string cmd_vel_topic_name = "robot_";
	cmd_vel_topic_name += std::to_string(ID_ROBOT);
	cmd_vel_topic_name += "/cmd_vel";
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

	while(ros::ok()) {
		ros::spinOnce();

		if (have_odom && is_goal_set) {
			cmdVel();
		}

		r.sleep();
	}*/
	
	ros::init(argc, argv, "moveRobot");
	MoveRobot mvR;
	mvR.readParams();
	mvR.run();


	return 0;
}

