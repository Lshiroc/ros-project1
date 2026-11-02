#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){

tf::Quaternion q(
        0,
        0,
        1,
        0);
    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ROS_INFO("roll= %lf, pitch=%lf, yaw=%lf", roll,pitch,yaw);

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 1, 0, 0), tf::Vector3(0.1, 0.0, 0.2)),
        ros::Time::now(),"base_link", "base_laser"));
    
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 1, 0), tf::Vector3(1.4, 2.8, 0.2)),
        ros::Time::now(),"base_laser", "base_robot"));
    r.sleep();
  }
}
