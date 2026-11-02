#include "first_demo/moveRobot.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include <tf/transform_datatypes.h>
#include <cmath>
#include <sstream>
#include <string>

// ---- global state (defined here, once) ----
static Parameters params;
static RobotData robot;
static State state = WAIT_GOAL;
static ros::Publisher cmd_vel_pub, position_pub;
static geometry_msgs::Vector3 V_goal, V_obs, V_f;
static ros::Time start;
static ros::Time last_decision;
static bool have_decision = false;
static double pf_lin = 0.0, pf_ang = 0.0;
static double d_min = 1e9;

// ---- helpers ----
double wrapAngle(double a) {
	return std::atan2(std::sin(a), std::cos(a));
}

double rotVel(double err) {
	double k = (std::fabs(err) > params.ORI_ERROR) ? params.K_ROT_MAX : params.K_ROT_MIN;
	return k * params.V_MAX_ROT * err;
}

double repulsion(double d) {
	return (params.CRIT_DIST - d) / params.CRIT_DIST;
}

bool isObstacle() {
	for (size_t i = 0; i < robot.scan.ranges.size(); i++) {
		double r = robot.scan.ranges[i];
		if (!std::isfinite(r) || r < robot.scan.range_min) continue;
		double angle = robot.scan.angle_min + i * robot.scan.angle_increment;
		if (std::fabs(angle) > CONE_HALF_ANGLE) continue;   // simple algorithm: frontal cone only
		if (r < params.CRIT_DIST) return true;
	}
	return false;
}

void updateVobs() {
	double sx = 0.0, sy = 0.0;
	d_min = 1e9;
	for (size_t i = 0; i < robot.scan.ranges.size(); i++) {
		double d = robot.scan.ranges[i];
		if (!std::isfinite(d) || d < robot.scan.range_min || d > robot.scan.range_max) continue;
		if (d < d_min) d_min = d;
		if (d >= params.CRIT_DIST) continue;
		double phi = robot.scan.angle_min + i * robot.scan.angle_increment;
		double mag = repulsion(d);
		sx += mag * (-std::cos(phi));
		sy += mag * (-std::sin(phi));
	}
	V_obs.x = sx;
	V_obs.y = sy;
}

// ---- callbacks ----
void callbackLaser(const sensor_msgs::LaserScan::ConstPtr& msg) {
	robot.scan = *msg;
	robot.isScan = true;
}

void callbackGoal(const geometry_msgs::Point::ConstPtr& msg) {
	robot.Xgoal = msg->x;
	robot.Ygoal = msg->y;
	robot.isGoal = true;
}

void callbackOdom(const nav_msgs::Odometry::ConstPtr& odom) {
	robot.Xr = odom->pose.pose.position.x;
	robot.Yr = odom->pose.pose.position.y;
	robot.Thetar = tf::getYaw(odom->pose.pose.orientation);
	robot.isOdom = true;

	geometry_msgs::Point p; // publish own position (leader -> follower)
	p.x = robot.Xr;
	p.y = robot.Yr;
	position_pub.publish(p);
}

// ---- parameters ----
void readParams() {
	ros::NodeHandle nh_private("~");
	bool ok = true;
	ok &= nh_private.param("ID_ROBOT", params.ID_ROBOT, 0);
	ok &= nh_private.param("CRIT_DIST", params.CRIT_DIST, 1.0);
	ok &= nh_private.param("V_MAX_DES", params.V_MAX_DES, 1.0);
	ok &= nh_private.param("V_MAX_ROT", params.V_MAX_ROT, 1.0);
	ok &= nh_private.param("D_OBJ", params.D_OBJ, 0.2);
	ok &= nh_private.param("K_ROT_MIN", params.K_ROT_MIN, 0.1);
	ok &= nh_private.param("K_ROT_MAX", params.K_ROT_MAX, 0.25);
	ok &= nh_private.param("ORI_ERROR", params.ORI_ERROR, 0.45);
	ok &= nh_private.param("T_AVOID_OBS", params.T_AVOID_OBS, 2.0);
	ok &= nh_private.param("DIST_LEADER", params.DIST_LEADER, 2.0);
	ok &= nh_private.param("ROBOT_ROLE", params.ROBOT_ROLE, 0);
	ok &= nh_private.param("ID_LEADER", params.ID_LEADER, -1);
	ok &= nh_private.param("W_1", params.W_1, 1.5);
	ok &= nh_private.param("W_2", params.W_2, 3.0);
	ok &= nh_private.param("T_WAIT", params.T_WAIT, 150);
	ok &= nh_private.param("ALGOR", params.ALGOR, 1);

	ROS_INFO("ID=%d ALGOR=%d ROLE=%d CRIT_DIST=%.2f D_OBJ=%.2f V_MAX_DES=%.2f V_MAX_ROT=%.2f",
	         params.ID_ROBOT, params.ALGOR, params.ROBOT_ROLE, params.CRIT_DIST,
	         params.D_OBJ, params.V_MAX_DES, params.V_MAX_ROT);
	ROS_INFO("K_ROT_MIN=%.2f K_ROT_MAX=%.2f ORI_ERROR=%.2f T_AVOID_OBS=%.2f DIST_LEADER=%.2f",
	         params.K_ROT_MIN, params.K_ROT_MAX, params.ORI_ERROR,
	         params.T_AVOID_OBS, params.DIST_LEADER);
	ROS_INFO("W_1=%.2f W_2=%.2f T_WAIT=%d ID_LEADER=%d",
	         params.W_1, params.W_2, params.T_WAIT, params.ID_LEADER);

	if (!ok) ROS_WARN("Some parameters were missing - defaults used, check the launch file");
	if (params.ALGOR != 1 && params.ALGOR != 2)
		ROS_WARN("ALGOR=%d is not 1 or 2, using simple avoidance", params.ALGOR);
	if (params.ROBOT_ROLE != 0 && params.ROBOT_ROLE != 1)
		ROS_WARN("ROBOT_ROLE=%d is not 0 or 1, acting as leader", params.ROBOT_ROLE);
}

// ---- algorithm 1: simple obstacle avoidance ----
void simpleAvoidanceController() {
	double dx = robot.Xgoal - robot.Xr;
	double dy = robot.Ygoal - robot.Yr;
	double distance = std::hypot(dx, dy);
	double ori_error = wrapAngle(std::atan2(dy, dx) - robot.Thetar);
	geometry_msgs::Twist cmd_vel;

	switch (state) {
	case WAIT_GOAL:
		ROS_INFO_THROTTLE(1.0, "%d[WAIT_GOAL] goal=%d odom=%d scan=%d",
		                  params.ID_ROBOT, robot.isGoal, robot.isOdom, robot.isScan);
		if (robot.isGoal && robot.isOdom && robot.isScan) state = FACE_GOAL;
		break;
	case FACE_GOAL:
		cmd_vel.angular.z = rotVel(ori_error);
		if (std::fabs(ori_error) < params.ORI_ERROR) state = GO;
		ROS_INFO_THROTTLE(0.5, "%d[FACE_GOAL] dist=%.2f err=%.1f deg",
		                  params.ID_ROBOT, distance, ori_error * 180 / M_PI);
		break;
	case GO:
		cmd_vel.angular.z = rotVel(ori_error);
		cmd_vel.linear.x = params.V_MAX_DES;
		if (isObstacle()) state = ROTATE;
		if (distance < params.D_OBJ) state = DONE;
		ROS_INFO_THROTTLE(0.5, "%d[GO] dist=%.2f err=%.1f deg",
		                  params.ID_ROBOT, distance, ori_error * 180 / M_PI);
		break;
	case ROTATE:
		cmd_vel.angular.z = params.V_MAX_ROT;
		if (!isObstacle()) {
			state = FORWARD;
			start = ros::Time::now();
		}
		ROS_INFO_THROTTLE(0.5, "%d[ROTATE]", params.ID_ROBOT);
		break;
	case FORWARD:
		cmd_vel.linear.x = params.V_MAX_DES;
		if (isObstacle())
			state = ROTATE;
		else if (ros::Time::now() - start > ros::Duration(params.T_AVOID_OBS))
			state = FACE_GOAL;
		ROS_INFO_THROTTLE(0.5, "%d[FORWARD]", params.ID_ROBOT);
		break;
	case DONE:
		ROS_INFO_THROTTLE(1.0, "%d[DONE] dist=%.2f", params.ID_ROBOT, distance);
		if (distance >= params.D_OBJ) state = FACE_GOAL;  // follower resumes when leader moves away
		break;
	}
	cmd_vel_pub.publish(cmd_vel);
}

// ---- algorithm 2: potential fields ----
void potentialFieldsController() {
	if (!(robot.isGoal && robot.isOdom && robot.isScan)) return;

	double dx = robot.Xgoal - robot.Xr;
	double dy = robot.Ygoal - robot.Yr;
	double distance = std::hypot(dx, dy);
	double stop_dist = (params.ROBOT_ROLE == 1) ? params.DIST_LEADER : params.D_OBJ;
	geometry_msgs::Twist cmd_vel;

	if (distance < stop_dist) {
		pf_lin = 0.0;
		pf_ang = 0.0;
		have_decision = false;
		cmd_vel_pub.publish(cmd_vel); // explicit stop
		return;
	}

	ros::Time now = ros::Time::now();
	if (!have_decision || (now - last_decision).toSec() >= params.T_WAIT / 1000.0) {
		double ori_error = wrapAngle(std::atan2(dy, dx) - robot.Thetar);
		V_goal.x = std::cos(ori_error);// unit vector, robot frame
		V_goal.y = std::sin(ori_error);
		updateVobs();

		V_f.x = params.W_1 * V_goal.x + params.W_2 * V_obs.x;
		V_f.y = params.W_1 * V_goal.y + params.W_2 * V_obs.y;

		if (std::hypot(V_f.x, V_f.y) > 1e-6) {
			double heading_err = std::atan2(V_f.y, V_f.x);	
			pf_ang = rotVel(heading_err);
			const double FLOOR = 0.2; // minimum fraction of speed
			double near  = std::min(1.0, d_min / params.CRIT_DIST); // 1 = clear, 0 = touching
			double align = std::max(0.0, std::cos(heading_err)); // 1 = V ahead, 0 = V sideways/behind
			pf_lin = params.V_MAX_DES * near * (FLOOR + (1.0 - FLOOR) * align);
		}
		last_decision = now;
		have_decision = true;
		ROS_INFO("PF: Vg=(%.2f,%.2f) Vobs=(%.2f,%.2f) err=%.2f lin=%.2f ang=%.2f",
		         V_goal.x, V_goal.y, V_obs.x, V_obs.y,
		         std::atan2(V_f.y, V_f.x), pf_lin, pf_ang);
	}

	cmd_vel.linear.x = pf_lin;                           // last decision, sent every cycle
	cmd_vel.angular.z = pf_ang;
	cmd_vel_pub.publish(cmd_vel);
}

// ---- main ----
int main(int argc, char **argv) {
	ros::init(argc, argv, "moveRobot");
	ros::NodeHandle nh;
	readParams();

	std::ostringstream parent;
	parent << "robot_" << params.ID_ROBOT;
	std::string prefix = parent.str();

	// publishers first, so callbacks can use them as soon as they fire
	cmd_vel_pub  = nh.advertise<geometry_msgs::Twist>(prefix + "/cmd_vel", 10);
	position_pub = nh.advertise<geometry_msgs::Point>(prefix + "/position", 1);

	ros::Subscriber odom_sub  = nh.subscribe(prefix + "/odom", 10, callbackOdom);
	ros::Subscriber laser_sub = nh.subscribe(prefix + "/base_scan_1", 1, callbackLaser);

	ros::Subscriber goal_sub;
	if (params.ROBOT_ROLE == 1) {                        // follower: goal = leader's position
		std::ostringstream leader;
		leader << "robot_" << params.ID_LEADER;
		goal_sub = nh.subscribe(leader.str() + "/position", 1, callbackGoal);
		params.D_OBJ = params.DIST_LEADER;
	} else {                                             // leader: goal from talkerGoals
		goal_sub = nh.subscribe("/myGoals", 10, callbackGoal);
	}

	ros::Rate rate(50);
	while (ros::ok()) {
		ros::spinOnce();
		if (params.ALGOR == 2) potentialFieldsController();
		else                   simpleAvoidanceController();
		rate.sleep();
	}
	return 0;
}
