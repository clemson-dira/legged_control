#include <gtest/gtest.h>
#include <ros/ros.h>

#include "legged_body_planner/legged_body_planner.h"
#include "legged_body_planner/target_trajectories_publisher.h"

// Declaring a test
TEST(TestTrajectoriesPublisher, InitTargetTrajectoriesPublisher) {
  // TargetTrajectories
  // TargetTrajectoriesPublisher target_trajectories_publisher(nh, "test", )
  ros::NodeHandle nh;
  const std::string robot_name = "legged_robot";
  TrajectoriesPublisher trajectories_publisher(nh, robot_name);
  EXPECT_EQ(1 + 1, 2) << "1 + 1 is not equal to 2";
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);  // &argc is the memory address of argc
  ros::init(argc, argv, "legged_body_planner_tester");
  return RUN_ALL_TESTS();
}