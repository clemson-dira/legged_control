#ifndef TARGET_TRAJECTORIES_PUBLISHER_H
#define TARGET_TRAJECTORIES_PUBLISHER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <mutex>

using namespace ocs2;

//! A copy of the Target Trajectories class w/o implementation of muloc, etc.
/*!
    TargetTrajectoriesPublisher is a class to understand the underlying
   mechanisms between publishing a reference 'plan' to the OCS2 framework

    Notes:
    1) Create std::function as a way to return a reference to a generic callable
   object with the following input parameters and output datatype
   2)
*/
class TargetTrajectoriesPublisher final {
 public:
  /// @brief Callable object to return target trajectories given vector_t& and
  /// SystemObservations&
  using CmdToTargetTrajectories_t = std::function<TargetTrajectories(
      const vector_t& cmd,
      const SystemObservation& observation)>;  // Alias declaration not used for
                                               // initialization statement, no
                                               // need for typedef

  /**
   * @brief Construct a new Target Trajectories Publisher object
   *
   * @param nh nodehandle
   * @param topic_prefix topic prefix
   * @param goalToTargetTrajectories Goal
   * @param cmd_vel_to_target_trajectories Vel
   */
  TargetTrajectoriesPublisher(
      ::ros::NodeHandle& nh, const std::string& topic_prefix,
      CmdToTargetTrajectories_t goalToTargetTrajectories,
      CmdToTargetTrajectories_t cmd_vel_to_target_trajectories)
      : goal_to_target_trajectories_(std::move(goalToTargetTrajectories)),
        cmd_vel_to_target_trajectories_(
            std::move(cmd_vel_to_target_trajectories)),
        tf2_(buffer_) {
    std::cout << "Initialize Target Trajectories Subscriber & Publisher\n";
    // Trajectories Publisher
    target_trajectories_publisher_.reset(
        new TargetTrajectoriesRosPublisher(nh, topic_prefix));

    // Observation subscriber
    // Gets MPC observation and updates into variable
    auto observationCallback =
        [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
          // Adding lock guard does the following:
          // Ensures it follows RAII
          // atm, no visible difference...

          std::lock_guard<std::mutex> lock(latest_observation_mutex_);
          latest_observation_ = ros_msg_conversions::readObservationMsg(*msg);
        };
    observation_sub_ = nh.subscribe<ocs2_msgs::mpc_observation>(
        topic_prefix + "_mpc_observation", 1, observationCallback);

    // Goal Subscriber
    // Subscribes to goal, converts goal msg (e.g. rviz) to vector_t data type,
    // create target trajectory, and publish target trajectory
    auto goalCallback = [this](
                            const geometry_msgs::PoseStamped::ConstPtr& msg) {
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
      vector_t cmd_goal = vector_t::Zero(6);
      cmd_goal[0] = pose.pose.position.x;
      cmd_goal[1] = pose.pose.position.y;
      cmd_goal[2] = pose.pose.position.z;

      // Use Eigen to convert to euler angles
      Eigen::Quaternion<scalar_t> q(
          pose.pose.orientation.w, pose.pose.orientation.x,
          pose.pose.orientation.y, pose.pose.orientation.z);

      cmd_goal[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z();
      cmd_goal[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y();
      cmd_goal[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x();

      const auto trajectories =
          goal_to_target_trajectories_(cmd_goal, latest_observation_);
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
      vector_t cmd_vel = vector_t::Zero(4);
      cmd_vel[0] = msg->linear.x;
      cmd_vel[1] = msg->linear.y;
      cmd_vel[2] = msg->linear.z;
      cmd_vel[3] = msg->angular.z;

      const auto trajectories =
          cmd_vel_to_target_trajectories_(cmd_vel, latest_observation_);
      target_trajectories_publisher_->publishTargetTrajectories(trajectories);
    };

    goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
        "/move_base_simple/goal", 1, goalCallback);
    cmd_vel_sub_ =
        nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelCallback);
  };

 private:
  /// Callable objects that translate goals or cmdVel to TargetTrajectories
  /// objects
  CmdToTargetTrajectories_t goal_to_target_trajectories_,
      cmd_vel_to_target_trajectories_;  // Class on its own

  // Subscribers
  ::ros::Subscriber observation_sub_, goal_sub_, cmd_vel_sub_;

  /// Publisher
  std::unique_ptr<TargetTrajectoriesRosPublisher>
      target_trajectories_publisher_;  // Pointer to an object to not initialize
                                       // constructor | Note: unique s.t. only
                                       // this has ownership

  /// @brief Buffer
  tf2_ros::Buffer buffer_;

  /// @brief Transformation listener
  tf2_ros::TransformListener tf2_;  // Object needs a buffer argument

  /// Mutex Observation (More descriptive when I know the inner workings of
  /// mutex)
  mutable std::mutex
      latest_observation_mutex_;  // Somehow protects latest_observation_

  /// @brief An observer for states
  SystemObservation latest_observation_;
};

#endif  // TARGET_TRAJECTORIES_PUBLISHER_H
