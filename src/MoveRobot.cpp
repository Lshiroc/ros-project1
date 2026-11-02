#include "first_demo/moveRobot.h"
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>

void MoveRobot::readParams() {
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
	
	ROS_INFO("ID_ROBOT=%d", params.ID_ROBOT);
	ROS_INFO("ALGOR=%d", params.ALGOR);
	ROS_INFO("CRIT_DIST=%lf", params.CRIT_DIST);
	ROS_INFO("D_OBJ=%lf", params.D_OBJ);
	ROS_INFO("V_MAX_DES=%lf", params.V_MAX_DES);
	ROS_INFO("V_MAX_ROT=%lf", params.V_MAX_ROT);
	ROS_INFO("K_ROT_MIN=%lf", params.K_ROT_MIN);
	ROS_INFO("K_ROT_MAX=%lf", params.K_ROT_MAX);
	ROS_INFO("ORI_ERROR=%lf", params.ORI_ERROR);
	ROS_INFO("T_AVOID_OBS=%lf", params.T_AVOID_OBS);
	ROS_INFO("DIST_LEADER=%lf", params.DIST_LEADER);
	ROS_INFO("ROBOT_ROLE=%d", params.ROBOT_ROLE);
	ROS_INFO("ID_LEADER=%d", params.ID_LEADER);
	ROS_INFO("W_1=%lf", params.W_1);
	ROS_INFO("W_2=%lf", params.W_2);
	ROS_INFO("T_WAIT=%d", params.T_WAIT);

	if (!ok)
		ROS_WARN("Some parameters were missing - check the launch file");
	if (param.ALGOR != 1 && param.ALGOR != 2)
		ROS_WARN("ALGOR=%d is not 1 or 2, using simple avoidance", param.ALGOR);
	if (param.ROBOT_ROLE != 0 && param.ROBOT_ROLE != 1)
		ROS_WARN("ROBOT_ROLE=%d is not 0 or 1, acting as leader", param.ROBOT_ROLE);
}

void MoveRobot::callbackLaser(const sensor_msgs::LaserScan& most_intense) {
	robot.scan = most_intense;
	robot.isScan = true;
}

//Callback of the topic myGoals
void MoveRobot::callbackGoal(const geometry_msgs::Point::ConstPtr &msg) {
	robot.Xgoal = msg->x;
	robot.Ygoal = msg->y;
	robot.isGoal = true;
	//ROS_INFO("Recieved  x:[%f], y:[%f]", robot.Xgoal, robot.Ygoal);
}

void MoveRobot::callbackOdom(const nav_msgs::Odometry odom) {
	// read the pose
	robot.Xr = odom.pose.pose.position.x;
	robot.Yr = odom.pose.pose.position.y;
	robot.Thetar = tf::getYaw(odom.pose.pose.orientation);
	// publish your position
	geometry_msgs::Point myPosition;
	myPosition.x = robot.Xr;
	myPosition.y = robot.Yr;
	m_position_pub.publish(myPosition);
	// confirm Odom was received
	robot.isOdom = true;
}

bool MoveRobot::isObstacle() {
	int length = robot.scan.ranges.size();
	for(int i = 0; i<length; i++) {
		double angle = robot.scan.angle_min + i * robot.scan.angle_increment;
		if (params.ALGOR == 1 && std::fabs(angle) > CONE_HALF_ANGLE) continue;
		if(robot.scan.ranges[i] < params.CRIT_DIST) return true;
	}
	return false;
}

double MoveRobot::rotVel(double ori_error) {
	if(std::fabs(ori_error) > params.ORI_ERROR)
		return params.K_ROT_MAX * params.V_MAX_ROT * ori_error;
	else
		return params.K_ROT_MIN * params.V_MAX_ROT * ori_error;
}

double MoveRobot::repulsion(double d) {
	return (params.CRIT_DIST - d) / params.CRIT_DIST;
}

void MoveRobot::updateVobs() {
	double sx = 0.0, sy = 0.0;
	for (size_t i = 0; i < robot.scan.ranges.size(); i++) {
		double d = robot.scan.ranges[i];
		if (!std::isfinite(d) || d < robot.scan.range_min 
			|| d > robot.scan.range_max) continue;
		if (d >= params.CRIT_DIST) continue;
		double phi = robot.scan.angle_min + i * robot.scan.angle_increment;
		double mag = repulsion(d);
		sx += mag * (-std::cos(phi));
		sy += mag * (-std::sin(phi));
	}
	m_V_obs.x = sx;
	m_V_obs.y = sy;
}

void MoveRobot::simpleAvoidanceController() {
	// common calculations
	double dx = robot.Xgoal - robot.Xr;
	double dy = robot.Ygoal - robot.Yr;
	double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
	double thetaGoal = std::atan2(dy, dx);
	double ori_error = std::atan2(std::sin(thetaGoal - robot.Thetar),
                         std::cos(thetaGoal - robot.Thetar));
	geometry_msgs::Twist cmd_vel;
	
	switch(m_state) {
	case WAIT_GOAL:
		if(robot.isGoal && robot.isOdom && robot.isScan) m_state = FACE_GOAL;
		ROS_INFO_THROTTLE(1.0, "%d[WAIT_GOAL] goal=%d odom=%d scan=%d",
		params.ID_ROBOT, robot.isGoal, robot.isOdom, robot.isScan);
		break;
	case FACE_GOAL:
		cmd_vel.angular.z = rotVel(ori_error);
		if(std::fabs(ori_error) < params.ORI_ERROR) m_state = GO;
		ROS_INFO("%d[FACE_GOAL] distance=%lf, angle=%lf\n", 
			params.ID_ROBOT, distance, ori_error * 180 / M_PI);
		break;
	case GO:
		cmd_vel.angular.z = rotVel(ori_error);
		cmd_vel.linear.x = params.V_MAX_DES;
		if(isObstacle()) m_state = ROTATE;
		if(distance < params.D_OBJ) m_state = DONE;
		ROS_INFO("%d[GO] distance=%lf, angle=%lf\n", 
			params.ID_ROBOT, distance, ori_error * 180 / M_PI);
		break;
	case ROTATE:
		cmd_vel.angular.z = params.V_MAX_ROT;
		cmd_vel.linear.x = 0.0;
		ROS_INFO("%d[ROTATE] distance=%lf, angle=%lf\n", 
			params.ID_ROBOT, distance, ori_error * 180 / M_PI);
		if(!isObstacle()) {
			m_state = FORWARD;
			start = ros::Time::now(); // due to clock rate it will trigger 200 not 150
		}
		break;
	case FORWARD:
		cmd_vel.linear.x = params.V_MAX_DES;
		if(ros::Time::now() - start > ros::Duration(params.T_AVOID_OBS)) m_state = FACE_GOAL;
		if(isObstacle()) {
			m_state = ROTATE;
		} else if (ros::Time::now() - start > ros::Duration(params.T_AVOID_OBS)) {
			m_state = FACE_GOAL;
		}
		ROS_INFO("%d[FORWARD] distance=%lf, angle=%lf\n", 
			params.ID_ROBOT, distance, ori_error * 180 / M_PI);
		break;
	case DONE:
		ROS_INFO("%d[DONE] distance=%lf, angle=%lf\n", 
			params.ID_ROBOT, distance, ori_error * 180 / M_PI);
		if(distance >= params.D_OBJ) m_state = FACE_GOAL;
		break;		
	}
	
	m_cmd_vel_pub.publish(cmd_vel);
}

void MoveRobot::potentialFieldsController() {
	double dx = robot.Xgoal - robot.Xr;
	double dy = robot.Ygoal - robot.Yr;
	double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
	double stop_dist = (params.ROBOT_ROLE == 1) ? params.DIST_LEADER : params.D_OBJ;
	geometry_msgs::Twist cmd_vel;

	if (distance < stop_dist) {
		m_pf_lin = 0.0;
		m_pf_ang = 0.0;
		m_have_decision = false; 
		m_cmd_vel_pub.publish(cmd_vel);
		return;
	}
	ros::Time now = ros::Time::now();
	updateVobs();
	if (!m_have_decision || (now.toSec() - m_last_decision.toSec()) >= params.T_WAIT / 1000.0) {
		double thetaGoal = std::atan2(dy, dx);
		double ori_error = std::atan2(std::sin(thetaGoal - robot.Thetar),
		                              std::cos(thetaGoal - robot.Thetar));
		m_V_goal.x = std::cos(ori_error);
		m_V_goal.y = std::sin(ori_error);
		
		// calculate V_f
		m_V_f.x = params.W_1*m_V_goal.x + params.W_2*m_V_obs.x;
		m_V_f.y = params.W_1*m_V_goal.y + params.W_2*m_V_obs.y;

		if (std::hypot(m_V_f.x, m_V_f.y) > 1e-6) {
			double heading_err = std::atan2(m_V_f.y, m_V_f.x);
			m_pf_ang = rotVel(heading_err);
			m_pf_lin = (std::fabs(heading_err) < params.ORI_ERROR) ? params.V_MAX_DES : 0.0;
		}
		m_last_decision = now;
		m_have_decision = true;
		ROS_INFO("PF: Vg=(%.2f,%.2f) Vobs=(%.2f,%.2f) err=%.2f lin=%.2f ang=%.2f",
		         m_V_goal.x, m_V_goal.y, m_V_obs.x, m_V_obs.y,
		         std::atan2(m_V_f.y, m_V_f.x), m_pf_lin, m_pf_ang);	
	}
	
	cmd_vel.angular.z = m_pf_ang;
	cmd_vel.linear.x = m_pf_lin;
	m_cmd_vel_pub.publish(cmd_vel);
}

void MoveRobot::run() {
	ros::Rate rate(50); // make it changable via parameters
	//Build a string with topic name
	start = ros::Time(0);
	std::ostringstream parent;
	parent << "robot_" << params.ID_ROBOT;
	
	// odom subscriber
   	std::string odom_topic_name = parent.str();
    	odom_topic_name += "/odom";
	m_odom_sub = m_nh.subscribe(odom_topic_name, 10, &MoveRobot::callbackOdom, this);
	
	// laser subscriber
	std::string sonar_scan_topic_name = parent.str();
	sonar_scan_topic_name += "/base_scan_1";
	m_laser_sub = m_nh.subscribe(sonar_scan_topic_name, 1, &MoveRobot::callbackLaser, this);
	
	// position publisher
	std::string position_topic_name = parent.str();
	position_topic_name += "/position";
	m_position_pub = m_nh.advertise<geometry_msgs::Point>(position_topic_name, 1);
	
	// velocity publisher
	std::string cmd_vel_topic_name = parent.str();
	cmd_vel_topic_name += "/cmd_vel";
	m_cmd_vel_pub = m_nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);
	
	// goal subscriber FOLLOWER OR LEADER
	std::string goal_topic_name = "";
	// if leader
	if(params.ROBOT_ROLE == 0) {
		// sub to talkerGoals
		goal_topic_name = "/myGoals";
		m_goal_sub = m_nh.subscribe(goal_topic_name, 10, &MoveRobot::callbackGoal, this);
	}
	else if(params.ROBOT_ROLE == 1) {
		// sub to leader position
		std::ostringstream leader;
		leader << "robot_" << params.ID_LEADER;
		goal_topic_name = leader.str() + "/position";
		m_goal_sub = m_nh.subscribe(goal_topic_name, 1, &MoveRobot::callbackGoal, this);
		// if follower D_OBJ = DIST_LEADER instead
		params.D_OBJ = params.DIST_LEADER;
	}

	while (ros::ok()) {
		ros::spinOnce();          // run any waiting callbacks, then return
		// wait for goal and odom
		if(params.ALGOR == 1) {
			simpleAvoidanceController();
		} else if(params.ALGOR == 2) {
			potentialFieldsController();		
		}		
		rate.sleep();
	}
}

