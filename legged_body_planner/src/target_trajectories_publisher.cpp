#include "legged_body_planner/target_trajectories_publisher.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

// namespace {
scalar_t TARGET_DISPLACEMENT_VELOCITY;
scalar_t TARGET_ROTATION_VELOCITY;
scalar_t COM_HEIGHT;
scalar_t TIME_TO_TARGET;
vector_t DEFAULT_JOINT_STATES(
    12);  // joint and time derivatives of hip, upper leg, lower leg
// }  // namespace

//! A copy code to understand the original code
/*!
  Summary of cpp notes learned:
  1) Pointers point to an object and do not necessarily call construct of
  object. Pointers simply point to memory location
  2) std::function is a way to define any callable object with the following
  params
  3) Typedef declarations can be used as initialization statements where
  alias declarations (e.g. using) cannot
  4) Lambda functions can have captures
  in [] where it can capture within the scope declared the type of data to
  'capture'
  5) Lambda functions can be declared, defined, and called in one
  statement

  OCS2 Notes:
  1) Read files from the .info files and using their member functions in
  LoadData.h 2) Their TargetTrajectories class simply consists of two
  structures, current state and target states (e.g. states have time, state, and
  control)
  2) Currently no reference in pitch or roll
  3) Time Horizon: 1s
  4) State Trajectory for quadruped dynamic is order prob in the following
  manner
    - v_x, v_y, v_z, yaw_dot, pitch_dot, roll_dot, x, y, z, yaw, pitch, roll,
  joint angles
*/

/**
 * @brief The time taken from position 1 to position 2
 * @param curr_pose Position 1
 * @param pos2 Position 2
 * @return scalar_t Return a scalar_t (type double) duration
 */
scalar_t estimateTimeToTarget(const vector_t& pos1, const vector_t& pos2) {
  const vector_t difference_pose = pos2 - pos1;
  const scalar_t& dx = difference_pose(0);
  const scalar_t& dy = difference_pose(1);
  const scalar_t& dyaw = difference_pose(3);
  const scalar_t rotation_time = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacement_time =
      displacement / TARGET_DISPLACEMENT_VELOCITY;

  return std::max(rotation_time, displacement_time);
}

/**
 * @brief Form the trajectory and time at t0 and delta t_target given target
 * pose and target time to reach pose
 * @param target_pose Target position
 * @param observation Observor for the current state
 * @param target_reaching_time Duration time to reach target pose
 * @return TargetTrajectories Trajectory of {time, state, and input}
 */
TargetTrajectories targetPoseToTargetTrajectories(
    const vector_t& target_pose, const SystemObservation& observation,
    const scalar_t& target_reaching_time) {
  // Desired time trajectory
  const scalar_array_t time_trajectory{
      observation.time,
      target_reaching_time};  // Initialize and declare std::vector at t0 and
                              // t_target

  // Desired state trajectory
  vector_t current_pose = observation.state.segment<6>(6);  // x,y,z,y,p,r
  current_pose(2) = COM_HEIGHT;
  current_pose(4) = 0;
  current_pose(5) = 0;

  vector_array_t state_trajectory(
      2, vector_t::Zero(
             observation.state
                 .size()));  // Create a std::vector of size 2 with each element
                             // initialized as eigen matrix initialize to 0 of
                             // observation state size
  // Trajectory x_dot, x, and q's
  state_trajectory[0] << vector_t::Zero(6), current_pose, DEFAULT_JOINT_STATES;
  state_trajectory[1] << vector_t::Zero(6), target_pose, DEFAULT_JOINT_STATES;

  // Desired input trajectory (just right dimensions, they are not used here)
  const vector_array_t input_trajectory(
      2, vector_t::Zero(observation.input.size()));
  return {time_trajectory, state_trajectory,
          input_trajectory};  // Implicit casting here?
}

/**
 * @brief Given target states, return the target states as a TargetTrajectories
 * @param target_state Target state(s)
 * @param observation Observer
 * @param target_reaching_time Time to reach target(s)
 * @return TargetTrajectories of state trajectories at different waypoints
 */
TargetTrajectories targetStatesToTargetTrajectories(
    const vector_t& target_state, const SystemObservation& observation,
    const scalar_t& target_reaching_time) {
  // Desired time trajectory
  const scalar_array_t time_trajectory{
      observation.time, target_reaching_time};  // std::vector<double>

  // Desired state trajectory
  // Get rigid body states (x_dot, euler rates, x, ypr)
  vector_t current_pose =
      observation.state.segment<12>(0);  // Operation for vectors

  // TODO (AZ) : Generalize to multiple target states
  vector_array_t state_trajectory(
      2, vector_t::Zero(
             observation.state.size()));  // std::vector<Eigen::Matrix<double,
                                          // Eigen::Dynamic, 1>;
  // Trajectory x_dot, x, and q's for initial and final states
  state_trajectory[0] << current_pose, DEFAULT_JOINT_STATES;
  state_trajectory[1] << target_state, DEFAULT_JOINT_STATES;

  // Desired input trajectory (they are not used here, so just for dimensions)
  const vector_array_t input_trajectory(
      2, vector_t::Zero(observation.input.size()));
  return {time_trajectory, state_trajectory, input_trajectory};
}

/**
 * @brief Converts desired goal to a target trajectory from current state to
 * goal state
 *
 * @param goal 3-dimensional goal state (x,y,yaw)
 * @param observation Observer
 * @return TargetTrajectories State, time, and control trajectory of current and
 * final goals
 */
TargetTrajectories goalToTargetTrajectories(
    const vector_t& goal, const SystemObservation& observation) {
  const vector_t current_pose = observation.state.segment<6>(6);
  auto target_pose = [&]() {
    vector_t target(6);
    target(0) = goal[0];     // x goal
    target(1) = goal[1];     // y goal
    target(2) = COM_HEIGHT;  // z goal
    target(3) = goal[3];     // yaw goal
    target(4) = 0;           // pitch goal
    target(5) = 0;           // roll goal
    return target;
  }();  // Declare, define, and call lambda function
  const scalar_t target_reaching_time =
      observation.time + estimateTimeToTarget(current_pose, target_pose);
  // std::cout << "Observation time: " << observation.time << std::endl;
  // std::cout << "target reaching time: " << target_reaching_time << std::endl;
  return targetPoseToTargetTrajectories(target_pose, observation,
                                        target_reaching_time);
}

/**
 * @brief Convert published command /cmd_vel to target trajectories
 * @param cmd_vel Local command velocity for fixed duration
 * @param observation Observer
 * @return TargetTrajectories trajectories consisting of initialy and final
 * trajectory
 */
TargetTrajectories cmdVelToTargetTrajectories(
    const vector_t& cmd_vel, const SystemObservation& observation) {
  const vector_t current_pose =
      observation.state.segment<6>(6);  // At position 6, segment 6 out
  const Eigen::Matrix<scalar_t, 3, 1> zyx =
      current_pose.tail(3);  // YPR current euler angles
  vector_t cmd_vel_rot = getRotationMatrixFromZyxEulerAngles(zyx) *
                         cmd_vel.head(3);  // Cmd vel w.r.t. body frame

  const scalar_t time_to_target = TIME_TO_TARGET;
  const vector_t target_pose = [&]() {
    vector_t target(6);
    target(0) = current_pose(0) + cmd_vel_rot(0) * time_to_target;
    target(1) = current_pose(1) + cmd_vel_rot(1) * time_to_target;
    target(2) = COM_HEIGHT;
    target(3) = current_pose(3) + cmd_vel(3) * time_to_target;  // yaw
    target(4) = 0;
    target(5) = 0;
    return target;
  }();

  // Target reaching duration
  const scalar_t target_reaching_time =
      observation.time + time_to_target;  // Reach target in a fixed time
  auto trajectories = targetPoseToTargetTrajectories(target_pose, observation,
                                                     target_reaching_time);
  trajectories.stateTrajectory[0].head(3) = cmd_vel_rot;
  trajectories.stateTrajectory[1].head(3) = cmd_vel_rot;
  return trajectories;
}

/**
 * @brief Hardcode to try multiple target trajectories
 *
 * @param goal
 * @param observation
 * @return TargetTrajectories
 */
TargetTrajectories test_goalToTargetTrajectories(
    const vector_t& goal, const SystemObservation& observation) {
  const vector_t current_pose = observation.state.segment<6>(6);
  auto target_pose = [&]() {
    vector_t target(6);
    target(0) = goal[0];     // x goal
    target(1) = goal[1];     // y goal
    target(2) = COM_HEIGHT;  // z goal
    target(3) = goal[3];     // yaw goal
    target(4) = 0;           // pitch goal
    target(5) = 0;           // roll goal
    return target;
  }();  // Declare, define, and call lambda function
  vector_t target_pose_2 = [&]() {
    vector_t target2(6);
    target2(0) = target_pose[0] + 1;
    target2(1) = target_pose[1] + 1;
    target2(2) = COM_HEIGHT;
    target2(3) = target_pose[3] + 0.5;
    target2(4) = 0;
    target2(5) = 0;
    return target2;
  }();
  const scalar_t target_reaching_time =
      observation.time + estimateTimeToTarget(current_pose, target_pose);
  const scalar_t target_reaching_time_2 =
      target_reaching_time + estimateTimeToTarget(target_pose, target_pose_2);

  // std::cout << "Observation time: " << observation.time << std::endl;
  // std::cout << "target reaching time: " << target_reaching_time <<
  // std::endl;

  // Define time trajectory
  const scalar_array_t time_trajectories{observation.time, target_reaching_time,
                                         target_reaching_time_2};

  // Define state trajectories
  vector_array_t state_trajectories{3,
                                    vector_t::Zero(observation.state.size())};
  state_trajectories[0] << vector_t::Zero(6), current_pose,
      DEFAULT_JOINT_STATES;
  state_trajectories[1] << vector_t::Zero(6), target_pose, DEFAULT_JOINT_STATES;

  // // Uncomment & comment above to enable velocity targets
  // vector_t vel_target(6);
  // vel_target << 1.0, 0.0, 0.0, -0.5, -0.5, 0.0;
  // state_trajectories[1] << vel_target, target_pose, DEFAULT_JOINT_STATES;

  state_trajectories[2] << vector_t::Zero(6), target_pose_2,
      DEFAULT_JOINT_STATES;

  // Define control trajectories
  const vector_array_t control_trajectories{
      3, vector_t::Zero(observation.input.size())};
  return {time_trajectories, state_trajectories,
          control_trajectories};  // Initializer list
}

int main(int argc, char** argv) {
  ROS_INFO("Main");
  const std::string robot_name = "legged_robot";

  // Initialize ros node
  ::ros::init(argc, argv, robot_name + "_target");
  ::ros::NodeHandle node_handle;

  // Declare variables
  std::string reference_file;
  std::string task_file;
  // Get node parameters
  node_handle.getParam("/referenceFile", reference_file);
  node_handle.getParam("/taskFile", task_file);

  loadData::loadCppDataType(reference_file, "comHeight", COM_HEIGHT);
  loadData::loadEigenMatrix(reference_file, "defaultJointState",
                            DEFAULT_JOINT_STATES);
  loadData::loadCppDataType(reference_file, "targetRotationVelocity",
                            TARGET_ROTATION_VELOCITY);
  loadData::loadCppDataType(reference_file, "targetDisplacementVelocity",
                            TARGET_DISPLACEMENT_VELOCITY);
  loadData::loadCppDataType(task_file, "mpc.timeHorizon", TIME_TO_TARGET);

  // ROS_INFO("Settings: ");
  // ROS_INFO("targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);

  TargetTrajectoriesPublisher target_pose_command(node_handle, robot_name,
                                                  &goalToTargetTrajectories,
                                                  &cmdVelToTargetTrajectories);

  // // Test multiple trajectories
  // TargetTrajectoriesPublisher target_pose_command(
  //     node_handle, robot_name, &test_goalToTargetTrajectories,
  //     &cmdVelToTargetTrajectories);
  ros::spin();
  // Successful exit
  return 0;
}