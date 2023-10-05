#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "ros/init.h"
#include "std_srvs/Trigger.h"
#include <cmath>
#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

#include "robot_gui/robot_gui.hpp"

#define WINDOW_NAME "Robot GUI"

RobotGUI::RobotGUI() {

  robot_info_sub_ = nh_.subscribe("/robot_info", 10, &RobotGUI::robot_info_cb,
                                  this); // sub to robot info

  odom_sub_ = nh_.subscribe("/odom", 10, &RobotGUI::odom_callback, this);

  gui_command_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  distance_service_ = nh_.serviceClient<std_srvs::Trigger>("/get_distance");
  reset_distance_service_ =
      nh_.serviceClient<std_srvs::Trigger>("/reset_distance");
}

RobotGUI::~RobotGUI() {}

void RobotGUI::odom_callback(const nav_msgs::OdometryConstPtr &msg) {
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  z_ = msg->pose.pose.position.z;
}

void RobotGUI::robot_info_cb(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  robot_info_msg_.data_field_01 = msg->data_field_01;
  robot_info_msg_.data_field_02 = msg->data_field_02;
  robot_info_msg_.data_field_03 = msg->data_field_03;
  robot_info_msg_.data_field_04 = msg->data_field_04;
  robot_info_msg_.data_field_05 = msg->data_field_05;
  robot_info_msg_.data_field_06 = msg->data_field_06;
  robot_info_msg_.data_field_07 = msg->data_field_07;
}

void RobotGUI::run() {

  cv::Mat frame = cv::Mat(650, 300, CV_8UC3); // create frame

  cv::namedWindow("Robot GUI", cv::WINDOW_NORMAL);

  cvui::init(WINDOW_NAME);

  while (ros::ok()) {

    frame = cv::Scalar(50, 50, 50);

    // info
    cvui::text(frame, 10, 10, "Robot Information:");
    cvui::text(frame, 10, 30, robot_info_msg_.data_field_01);
    cvui::text(frame, 10, 50, robot_info_msg_.data_field_02);
    cvui::text(frame, 10, 70, robot_info_msg_.data_field_03);
    cvui::text(frame, 10, 90, robot_info_msg_.data_field_04);
    cvui::text(frame, 10, 110, robot_info_msg_.data_field_05);
    cvui::text(frame, 10, 130, robot_info_msg_.data_field_06);
    cvui::text(frame, 10, 150, robot_info_msg_.data_field_07);

    // control
    cvui::text(frame, 10, 200, "Robot Control:");

    if (cvui::button(frame, 100, 230, " FORWARD ")) {
      twist_msg_.linear.x += 0.1;
    }

    if (cvui::button(frame, 100, 260, "   STOP  ")) {
      twist_msg_.linear.x = 0;
      twist_msg_.angular.z = 0;
    }
    if (cvui::button(frame, 100, 290, "BACKWARDS")) {
      twist_msg_.linear.x += -0.1;
    }
    if (cvui::button(frame, 0, 260, "  LEFT   ")) {
      twist_msg_.angular.z += 0.1;
    }
    if (cvui::button(frame, 200, 260, "   RIGHT ")) {
      twist_msg_.angular.z += -0.1;
    }

    // velocities
    cvui::text(frame, 10, 330, "Velocities:");
    // linear
    linear_velocity_ = twist_msg_.linear.x;
    std::ostringstream lv;
    lv << std::fixed << std::setprecision(2) << linear_velocity_;
    cvui::text(frame, 10, 350, "Linear Velocity");
    cvui::text(frame, 10, 370, lv.str() + " m/s", 0.5, 0xFF0000);
    // angular
    angular_velocity_ = twist_msg_.angular.z;
    std::ostringstream av;
    av << std::fixed << std::setprecision(2) << angular_velocity_;
    cvui::text(frame, 150, 350, "Angular Velocity");
    cvui::text(frame, 150, 370, av.str() + " rad/s", 0.5, 0xFF0000);

    // robot position
    cvui::text(frame, 10, 420, "Estimate Robot Position via Odometry:");
    cvui::text(frame, 10, 440, "X: " + std::to_string(static_cast<int>(x_)));
    cvui::text(frame, 70, 440, "Y: " + std::to_string(static_cast<int>(y_)));
    cvui::text(frame, 130, 440, "Z: " + std::to_string(static_cast<int>(z_)));

    // distance travelled
    cvui::text(frame, 10, 490,
               "Distance Travelled: " + distance_service_response_);
    // get_distance service call
    if (cvui::button(frame, 10, 510, "Get Distance")) {
      std_srvs::Trigger srv;

      if (distance_service_.call(srv)) {
        distance_service_response_ = srv.response.message + " meters";

      } else {
        distance_service_response_ = "GET FAILED";
      }
    }
    // rest_distance service call
    if (cvui::button(frame, 10, 540, "Reset Distance")) {
      std_srvs::Trigger srv;
      if (reset_distance_service_.call(srv)) {
        distance_service_response_ = srv.response.message + " meters";

      } else {
        distance_service_response_ = "RESET FAILED";
      }
    }

    gui_command_pub_.publish(twist_msg_); // publish gui commands

    cvui::update();
    cvui::imshow(WINDOW_NAME, frame);

    if (cv::waitKey(20) == 27) { // Press 'Esc' to exit.
      break;
    }

  ros:
    ros::spinOnce();
  }
}