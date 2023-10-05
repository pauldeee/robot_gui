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

  //   ros::Publisher gui_command_pub_;

  robotinfo_msgs::RobotInfo10Fields robot_info_msg_;
  nav_msgs::Odometry odom_msg_;
  geometry_msgs::Twist twist_msg_;

  int x_, y_, z_;

  void robot_info_cb(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);
  void odom_callback(const nav_msgs::OdometryConstPtr &msg);
};
