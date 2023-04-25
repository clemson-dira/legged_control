#include "legged_body_planner/legged_body_planner.h"
// #include "legged_body_planner/trajectories_publisher.h"

int main(int argc, char **argv) {
  std::cout << "Load legged body planner node" << std::endl;
  ros::init(argc, argv, "legged_body_planner_node");
  ros::NodeHandle nh;
  std::string robot_name = "legged_robot";

  LeggedBodyPlanner legged_body_planner(nh, robot_name);
  legged_body_planner.spin();

  // TrajectoriesPublisher trajectories_publisher(nh, robot_name);
  // trajectories_publisher.spin();
  // TODO (AZ): Figure out why not spinning
  // when
  //                                 // putting two together
  return 0;
}