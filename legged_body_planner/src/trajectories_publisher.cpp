#include "legged_body_planner/trajectories_publisher.h"

// TODO (AZ): Make a abstract class
TrajectoriesPublisher::TrajectoriesPublisher(
    ros::NodeHandle nh, const std::string topic_prefix,
    CmdToTargetTrajectories_t goalToTargetTrajectories,
    CmdToTargetTrajectories_t cmdVelToTargetTrajectories)
    : goalToTargetTrajectories_(goalToTargetTrajectories),
      cmdVelToTargeTrajectories_(cmdVelToTargetTrajectories),
      tf2_(buffer_) {
  nh_ = nh;
  // Load parameters
  nh_.getParam("/trajectories_publisher/update_rate", update_rate_);

  // Trajectories publisher
  target_trajectories_publisher_.reset(
      new ocs2::TargetTrajectoriesRosPublisher(nh, topic_prefix));

  // Observation subscriber
  auto observationCallback =
      [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(latest_observation_mutex_);
        latest_observation_ =
            ocs2::ros_msg_conversions::readObservationMsg(*msg);
      };
  observation_sub_ = nh.subscribe<ocs2_msgs::mpc_observation>(
      topic_prefix + "_mpc_observation", 1, observationCallback);

  // Goal Subscriber
  // Subscribes to goal, converts goal msg (e.g. rviz) to vector_t data type,
  // create target trajectory, and publish target trajectory
  auto goalCallback = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (latest_observation_.time == 0) {  // What is this?
      return;
    }
    geometry_msgs::PoseStamped pose = *msg;  // Dereference the msg

    try {
      buffer_.transform(
          pose, pose, "odom",
          ros::Duration(0.2));  // Simply putting pose msg into odom?
    } catch (tf2::TransformException& ex) {
      ROS_WARN("Failure %s\n", ex.what());
      return;
    }

    //  cmd_goal is x,y,z, yaw, pitch, roll
    ocs2::vector_t cmd_goal = ocs2::vector_t::Zero(6);
    cmd_goal[0] = pose.pose.position.x;
    cmd_goal[1] = pose.pose.position.y;
    cmd_goal[2] = pose.pose.position.z;

    // Use Eigen to convert to euler angles
    Eigen::Quaternion<ocs2::scalar_t> q(
        pose.pose.orientation.w, pose.pose.orientation.x,
        pose.pose.orientation.y, pose.pose.orientation.z);

    cmd_goal[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z();
    cmd_goal[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y();
    cmd_goal[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x();

    const auto trajectories =
        goalToTargetTrajectories_(cmd_goal, latest_observation_);
    target_trajectories_publisher_->publishTargetTrajectories(trajectories);
  };

  // cmd_vel subscriber
  // Subscribe to topic (e.g. cmd_vel), converts geometry_msgs to vector_t,
  // convert to TargetTrajectories, and publish new message
  auto cmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
    if (latest_observation_.time == 0.0) {
      // std::cout << "Latest observation is: " << latest_observation_.time
      //           << std::endl;
      return;
    }

    // std::cout << "Past observation 0" << std::endl;
    ocs2::vector_t cmd_vel = ocs2::vector_t::Zero(4);
    cmd_vel[0] = msg->linear.x;
    cmd_vel[1] = msg->linear.y;
    cmd_vel[2] = msg->linear.z;
    cmd_vel[3] = msg->angular.z;

    const auto trajectories =
        cmdVelToTargeTrajectories_(cmd_vel, latest_observation_);
    target_trajectories_publisher_->publishTargetTrajectories(trajectories);
  };

  goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",
                                                       1, goalCallback);
  cmd_vel_sub_ =
      nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelCallback);

  // TODO (AZ): Subscribe to body_plan & process ROS msg to TargetTrajectories
};

TrajectoriesPublisher::TrajectoriesPublisher(ros::NodeHandle& nh,
                                             const std::string& topic_prefix)
    : tf2_(buffer_) {
  // Define the default function reference for basic navigation

  // Get robot params
  std::string reference_file;
  std::string task_file;
  ocs2::scalar_t COM_HEIGHT;
  ocs2::vector_t DEFAULT_JOINT_STATE(12);
  ocs2::scalar_t TARGET_ROTATION_VELOCITY;
  ocs2::scalar_t TARGET_DISPLACEMENT_VELOCITY;
  ocs2::scalar_t TIME_TO_TARGET;

  // Get params
  nh.getParam("/referenceFile", reference_file);
  nh.getParam("/taskFile", task_file);
  // ROS_INFO("reference file %s", reference_file.c_str());
  // ROS_INFO("Taskfile %s", task_file.c_str());

  // Load params
  DEFAULT_JOINT_STATE(12);

  ocs2::loadData::loadCppDataType(reference_file, "comHeight", COM_HEIGHT);
  ocs2::loadData::loadEigenMatrix(reference_file, "defaultJointState",
                                  DEFAULT_JOINT_STATE);
  ocs2::loadData::loadCppDataType(reference_file, "targetRotationVelocity",
                                  TARGET_ROTATION_VELOCITY);
  ocs2::loadData::loadCppDataType(reference_file, "targetDisplacementVelocity",
                                  TARGET_DISPLACEMENT_VELOCITY);
  ocs2::loadData::loadCppDataType(task_file, "mpc.timeHorizon", TIME_TO_TARGET);

  // By default, define goalToTargetTrajectories and cmdVelToTargetTrajectories
  auto estimateTimeToTarget =
      [&TARGET_ROTATION_VELOCITY, &TARGET_DISPLACEMENT_VELOCITY](
          const ocs2::vector_t& pos1, const ocs2::vector_t& pos2) {
        const ocs2::vector_t difference_pos = pos2 - pos1;
        const ocs2::scalar_t& dx = difference_pos(0);
        const ocs2::scalar_t& dy = difference_pos(1);
        const ocs2::scalar_t& dyaw = difference_pos(3);
        const ocs2::scalar_t& rotation_time =
            std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
        const ocs2::scalar_t& displacement = std::sqrt(dx * dx + dy * dy);
        const ocs2::scalar_t& displacement_time =
            displacement / TARGET_DISPLACEMENT_VELOCITY;
        return std::max(rotation_time, displacement_time);
      };

  auto targetPoseToTargetTrajectories =
      [&COM_HEIGHT, &DEFAULT_JOINT_STATE](
          const ocs2::vector_t& target_pose,
          const ocs2::SystemObservation& observation,
          const ocs2::scalar_t target_reaching_time) {
        // Desired time trajectory
        const ocs2::scalar_array_t time_trajectory{
            observation.time,
            target_reaching_time};  // Initiaize & declare std::vector @ t0 and
                                    // t_target
        // Desired state trajecotry
        ocs2::vector_t current_pose =
            observation.state.segment<6>(6);  // x,y,z,y,p,r
        current_pose(2) = COM_HEIGHT;
        current_pose(4) = 0;
        current_pose(5) = 0;

        // Create std::vector to contain initial and final state trajectory
        ocs2::vector_array_t state_trajectory(
            2, ocs2::vector_t::Zero(observation.state.size()));

        // State trajectory: x_dot, x, and q's
        state_trajectory[0] << ocs2::vector_t::Zero(6), current_pose,
            DEFAULT_JOINT_STATE;
        state_trajectory[1] << ocs2::vector_t::Zero(6), target_pose,
            DEFAULT_JOINT_STATE;

        // Desired input traj (just get right dim, not used here)
        const ocs2::vector_array_t input_trajectory(
            2, ocs2::vector_t::Zero(observation.input.size()));
        return ocs2::TargetTrajectories(time_trajectory, state_trajectory,
                                        input_trajectory);
      };

  auto targetStatesToTargetTrajectories =
      [&DEFAULT_JOINT_STATE](const ocs2::vector_t& target_state,
                             const ocs2::SystemObservation& observation,
                             const ocs2::scalar_t target_reaching_time) {
        // TODO (AZ): Check this works and increment target state
        // Desired time trajectory
        const ocs2::scalar_array_t time_trajectory{observation.time,
                                                   target_reaching_time};

        // Desired state trajectory
        // Rigid body states (x_dot, euler_rates, x, ypr)
        ocs2::vector_t current_pose = observation.state.segment<12>(0);

        ocs2::vector_array_t state_trajectory(
            2, ocs2::vector_t::Zero(
                   observation.state
                       .size()));  // std::vector<Eigen::Matrix<double,
                                   // Eigen::Dynamic,1>;
        // State traj: x_dot, q_dot, x, and q's
        state_trajectory[0] << current_pose, DEFAULT_JOINT_STATE;
        state_trajectory[1] << target_state, DEFAULT_JOINT_STATE;

        // Desired input trajectory (used for dim)
        const ocs2::vector_array_t input_trajectory(
            2, ocs2::vector_t::Zero(observation.input.size()));

        return ocs2::TargetTrajectories(time_trajectory, state_trajectory,
                                        input_trajectory);
      };

  auto goalToTargetTrajectories =
      [&, COM_HEIGHT](const ocs2::vector_t& goal,
                      const ocs2::SystemObservation& observation) {
        // Current position (x,ypr) x in R^3
        const ocs2::vector_t current_pose = observation.state.segment<6>(6);
        ocs2::vector_t target_pose(6);
        target_pose[0] = goal[0];     // x goal
        target_pose[1] = goal[1];     // y goal
        target_pose[2] = COM_HEIGHT;  // z goal
        target_pose[3] = goal[3];     // yaw goal
        target_pose[4] = 0;           // pitch goal
        target_pose[5] = 0;           // roll goal

        const ocs2::scalar_t target_reaching_time =
            observation.time + estimateTimeToTarget(current_pose, target_pose);

        return targetPoseToTargetTrajectories(target_pose, observation,
                                              target_reaching_time);
      };

  auto cmdVelToTargetTrajectories =
      [&, COM_HEIGHT](const ocs2::vector_t& cmd_vel,
                      const ocs2::SystemObservation observation) {
        // Get current state
        const ocs2::vector_t current_pose = observation.state.segment<6>(6);

        // Get rotate cmd_vel
        const Eigen::Matrix<ocs2::scalar_t, 3, 1> ypr =
            current_pose.tail(3);  // YPR of euler angles
        ocs2::vector_t cmd_vel_rot =
            ocs2::getRotationMatrixFromZyxEulerAngles(ypr) *
            cmd_vel.head(3);  // cmd vel w.r.t. body frame
        const ocs2::scalar_t time_to_target = TIME_TO_TARGET;

        // Define target pose
        ocs2::vector_t target(6);
        target(0) = current_pose(0) + cmd_vel_rot(0) * time_to_target;
        target(1) = current_pose(1) + cmd_vel_rot(1) * time_to_target;
        target(2) = COM_HEIGHT;
        target(3) = current_pose(3) + cmd_vel(3) * time_to_target;  // yaw
        target(4) = 0;
        target(5) = 0;

        const ocs2::vector_t target_pose = target;

        // Target reaching time
        const ocs2::scalar_t target_reaching_time =
            observation.time + time_to_target;  // Fixed time
        auto trajectories = targetPoseToTargetTrajectories(
            target_pose, observation, target_reaching_time);

        // Set initial and final velocity to be the following
        trajectories.stateTrajectory[0].head(3) = cmd_vel_rot;
        trajectories.stateTrajectory[1].head(3) = cmd_vel_rot;
        return trajectories;
      };

  TrajectoriesPublisher(nh, topic_prefix, goalToTargetTrajectories,
                        cmdVelToTargetTrajectories);
};

void TrajectoriesPublisher::spin() {
  update_rate_ = 10;  // TODO(AZ): Rate to update
  ros::Rate r(update_rate_);
  while (ros::ok()) {
    // Process callbacks
    ros::spinOnce();

    // Set start state

    // Set goal state

    // Call planner (TODO(AZ): This should process message)

    // Publish if results are valid
    r.sleep();
  }
}
