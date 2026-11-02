#ifndef MOVEROBOT_H
#define MOVEROBOT_H

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <string>

enum State { WAIT_GOAL, FACE_GOAL, GO, ROTATE, FORWARD, DONE };

// For Simple Avoidance Algorithm
// The angle to check for obstacles
// that are worth avoiding
const double CONE_HALF_ANGLE = 0.35; // TODO:: add calculation of this angle

// structure for parameters
typedef struct Parameters
{
	int ID_ROBOT{};
	int ALGOR{};
	double CRIT_DIST{};
	double D_OBJ{};
	double V_MAX_DES{};
	double V_MAX_ROT{};
	double K_ROT_MIN{};
	double K_ROT_MAX{};
	double ORI_ERROR{};
	double T_AVOID_OBS{};
	double DIST_LEADER{};
	int ROBOT_ROLE{};
	int ID_LEADER{};
	double W_1{};
	double W_2{};
	int T_WAIT{};
} Parameters;

typedef struct RobotController
{
	double Xr;
	double Yr;
	double Thetar;
	double linear_vel;
	double angular_vel;
	double Xgoal;
	double Ygoal;
	sensor_msgs::LaserScan scan;
	
	bool isGoal = false;
	bool isOdom = false;
	bool isScan = false;

} RobotController;

class MoveRobot {
public:

	void readParams();
	void run();

private:

	ros::NodeHandle m_nh;
	ros::Subscriber m_goal_sub;
	ros::Subscriber m_laser_sub;
	ros::Subscriber m_odom_sub;
	ros::Publisher m_cmd_vel_pub;
	ros::Publisher m_position_pub;
	geometry_msgs::Vector3 m_V_goal;
	geometry_msgs::Vector3 m_V_obs;
	geometry_msgs::Vector3 m_V_f;
	ros::Time start;
	Parameters params;
	RobotController robot;
	State m_state = WAIT_GOAL;
	double m_pf_lin = 0.0;
	double m_pf_ang = 0.0;
	ros::Time m_last_decision;
	bool m_have_decision = false;

	double rotVel(double ori_error);
	double calcOriError(geometry_msgs::Point goal, geometry_msgs::Point robot, double yaw);
	double repulsion(double d);
	bool isObstacle();
	void updateVobs();
	void simpleAvoidanceController();
	void potentialFieldsController();

	// callbacks
	void callbackLaser(const sensor_msgs::LaserScan& most_intense);
	void callbackOdom(const nav_msgs::Odometry odom);
	void callbackGoal(const geometry_msgs::Point::ConstPtr &msg);

};

#endif

