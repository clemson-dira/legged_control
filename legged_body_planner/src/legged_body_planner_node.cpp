#include "legged_body_planner/legged_body_planner.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "legged_body_planner_node");
  ros::NodeHandle nh;
  std::string robot_name = "legged_robot";

  TrajectoriesPublisher trajectories_publisher(nh, robot_name);
  trajectories_publisher.spin();
  return 0;
}