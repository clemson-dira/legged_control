#include "legged_body_planner/legged_body_planner.h"

LeggedBodyPlanner::LeggedBodyPlanner(ros::NodeHandle& nh,
                                     std::string topic_prefix) {
  std::cout << "Initialized LeggedBodyPlanner class\n";

  nh_ = nh;

  // Get parameters
  std::string reference_file, task_file;
  std::vector<double> goal_state_vector(12);
  loadROSParam(nh_, "/referenceFile", reference_file);
  loadROSParam(nh_, "/taskFile", task_file);

  ocs2::loadData::loadCppDataType(reference_file, "comHeight", COM_HEIGHT_);
  ocs2::loadData::loadCppDataType(task_file, "mpc.timeHorizon", PLAN_HORIZON_);

  // Load rosparams
  loadROSParam(nh_, "/legged_body_planner/update_rate", update_rate_);
  loadROSParam(nh_, "legged_body_planner/dt", dt_);
  loadROSParam(nh_, "legged_body_planner/goal_state", goal_state_vector);

  // Setup publisher and subscribers

  // Fill in goal state information
  // TODO (AZ) Get start, goal states, terrain info, call planning algorithm
  // (prob a ROS action server), and then publish to something subscribed by
  // trajectories_publisher
}

void LeggedBodyPlanner::observerCallback(
    const ocs2_msgs::mpc_observation::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(latest_observation_mutex_);
  latest_observation_ = ocs2::ros_msg_conversions::readObservationMsg(*msg);
}

void LeggedBodyPlanner::vectorToRigidBodyState(const std::vector<double>& v,
                                               legged_body_msgs::State& s) {
  if (v.size() != 12) {
    ROS_ERROR("std::vector<double> is incorrect size");
  }
  s.state[0] = v[0];    // x_dot
  s.state[1] = v[1];    // y_dot
  s.state[2] = v[2];    // z_dot
  s.state[3] = v[3];    // yaw_dot
  s.state[4] = v[4];    // pitch_dot
  s.state[5] = v[5];    // roll_dot
  s.state[6] = v[6];    // x
  s.state[7] = v[7];    // y
  s.state[8] = v[8];    // z
  s.state[9] = v[9];    // yaw
  s.state[10] = v[10];  // pitch
  s.state[11] = v[11];  // roll
}
