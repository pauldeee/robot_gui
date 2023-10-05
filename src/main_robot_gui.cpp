#include "robot_gui/robot_gui.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_gui_node");

  RobotGUI rgui;
  rgui.run();

  return 0;
}
