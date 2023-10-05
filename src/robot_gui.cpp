#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "ros/init.h"
#include <cmath>
#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

#include "robot_gui/robot_gui.hpp"

#define WINDOW_NAME "Robot GUI"

RobotGUI::RobotGUI() {

  robot_info_sub_ = nh_.subscribe("/robot_info", 10, &RobotGUI::robot_info_cb,
                                  this); // sub to robot info
}

RobotGUI::~RobotGUI() {}

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
    cvui::button(frame, 100, 230, " FORWARD ");
    cvui::button(frame, 100, 260, "   STOP  ");
    cvui::button(frame, 100, 290, "BACKWARDS");
    cvui::button(frame, 0, 260, "  LEFT   ");
    cvui::button(frame, 200, 260, "   RIGHT ");

    // velocities
    cvui::text(frame, 10, 330, "Velocities:");

    // linear
    cvui::text(frame, 10, 350, "Linear Velocity");
    cvui::text(frame, 10, 370, " m/s", 0.5, 0xFF0000);
    // angular
    cvui::text(frame, 150, 350, "Angular Velocity");
    cvui::text(frame, 150, 370, " rad/s", 0.5, 0xFF0000);

    // robot position
    cvui::text(frame, 10, 420, "Estimate Robot Position via Odometry:");
    cvui::text(frame, 10, 440, "X: ");
    cvui::text(frame, 70, 440, "Y: ");
    cvui::text(frame, 130, 440, "Z: ");

    // distance travelled
    cvui::text(frame, 10, 490, "Distance Travelled:");

    cvui::button(frame, 10, 510, "Call");
    cvui::text(frame, 80, 510, "Distance Travelled: x m");

    cvui::imshow(WINDOW_NAME, frame);

    if (cv::waitKey(20) == 27) { // Press 'Esc' to exit.
      break;
    }

  ros:
    ros::spinOnce();
  }
}

// void initializeUI() { cvui::init("Simple cvui Application"); }

// void updateUI(cv::Mat &frame) {
//   cv::namedWindow("Simple cvui Application", cv::WINDOW_NORMAL);

//   bool buttonClicked = cvui::button(frame, 50, 50, "Click me!");
//   if (buttonClicked) {
//     cv::putText(frame, "Button clicked!", cv::Point(50, 150),
//                 cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
//   }

//   static int trackbarValue = 50;
//   cvui::trackbar(frame, 50, 200, 165, &trackbarValue, 0, 100);

//   cvui::imshow("Simple cvui Application", frame);
// }