#ifndef MOVEROBOT_H
#define MOVEROBOT_H

#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

// ---- types ----
enum State { WAIT_GOAL, FACE_GOAL, GO, ROTATE, FORWARD, DONE };

struct Parameters {
	int ID_ROBOT{}, ALGOR{}, ROBOT_ROLE{}, ID_LEADER{}, T_WAIT{};
	double CRIT_DIST{}, D_OBJ{}, V_MAX_DES{}, V_MAX_ROT{};
	double K_ROT_MIN{}, K_ROT_MAX{}, ORI_ERROR{}, T_AVOID_OBS{};
	double DIST_LEADER{}, W_1{}, W_2{};
};

struct RobotData {
	double Xr{}, Yr{}, Thetar{}, Xgoal{}, Ygoal{};
	sensor_msgs::LaserScan scan;
	bool isGoal = false, isOdom = false, isScan = false;
};

// ---- constants ----
// half-width of the frontal cone used by the simple algorithm (rad)
const double CONE_HALF_ANGLE = 0.35;

// ---- helpers ----
double wrapAngle(double a);
double rotVel(double err);
double repulsion(double d);
bool isObstacle();
void updateVobs();

// ---- callbacks ----
void callbackLaser(const sensor_msgs::LaserScan::ConstPtr& msg);
void callbackGoal(const geometry_msgs::Point::ConstPtr& msg);
void callbackOdom(const nav_msgs::Odometry::ConstPtr& odom);

// ---- setup and controllers ----
void readParams();
void simpleAvoidanceController();
void potentialFieldsController();

#endif
