#pragma once

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <robotinfo_msgs/RobotInfo10Fields.h>
#include <ros/ros.h>

class RobotGUI {

public:
  RobotGUI();
  ~RobotGUI();

  void run();

private:
  ros::NodeHandle nh_;

  ros::Subscriber robot_info_sub_;
  ros::Subscriber odom_sub_;

  ros::Publisher gui_command_pub_;

  ros::ServiceClient distance_service_;
  std::string distance_service_response_;

  ros::ServiceClient reset_distance_service_;
  std::string reset_distance_service_response_;

  robotinfo_msgs::RobotInfo10Fields robot_info_msg_;
  nav_msgs::Odometry odom_msg_;
  geometry_msgs::Twist twist_msg_;

  int x_, y_, z_;

  double linear_velocity_ = 0, angular_velocity_ = 0;

  void robot_info_cb(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);
  void odom_callback(const nav_msgs::OdometryConstPtr &msg);
};
