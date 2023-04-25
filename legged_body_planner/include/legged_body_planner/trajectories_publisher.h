#ifndef TRAJECTORIES_PUBLISHER_H
#define TRAJECTORIES_PUBLSIHER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <mutex>

#include "legged_body_msgs/Control.h"
#include "legged_body_msgs/Plan.h"
#include "legged_body_msgs/State.h"
#include "legged_body_planner/planning_utils.h"

//! Class that subscribes to topics and publishes the topics as
//! TargetTrajectories
/*!
  TrajectoriesPublisher is a class that subscribes to topics and transforms the
  following ROS msgs to TargetTrajectories. The class is capable of converting
  body_plan msgs to TargetTrajectories

  Topics subscribed:
  - /cmd_vel
  - /move_base/simple/goal
  ROS msgs conversion:
  - Twist -> TargetTrajectories
  - PoseStamped -> TargetTrajectories
  - BodyPlan -> TargetTrajectories
*/
class TrajectoriesPublisher {
 public:
  using CmdToTargetTrajectories_t = std::function<ocs2::TargetTrajectories(
      const ocs2::vector_t& cmd, const ocs2::SystemObservation& observation)>;
  using PlanToTargetTrajectories_t = std::function<ocs2::TargetTrajectories(
      const legged_body_msgs::Plan::ConstPtr& plan,
      const ocs2::SystemObservation& observation)>;

  /**
   * @brief Construct a new Trajectories Publisher object
   * @param nh ros nodehandle
   * @param topicPrefix prefix topic
   */
  TrajectoriesPublisher(ros::NodeHandle& nh, const std::string& topic_prefix);

  /**
   * @brief Construct a new Trajectories Publisher object
   * @param nh ros nodehandle
   * @param topic_prefix prefix topic
   * @param goal_to_target_trajectories Function reference from goal to target
   * trajectories
   * @param cmd_vel_to_target_trajectories Function reference from cmd_vel to
   * target trajectories
   */
  TrajectoriesPublisher(ros::NodeHandle nh, const std::string topic_prefix,
                        CmdToTargetTrajectories_t goalToTargetTrajectories,
                        CmdToTargetTrajectories_t cmdVelToTargetTrajectories,
                        PlanToTargetTrajectories_t planToTargetTrajectories);

  // /**
  //  * @brief Converts plan to a target trajectories
  //  * @param plan Legged Body Msg plan
  //  * @return ocs2::TargetTrajectories TargetTrajectory
  //  */
  // ocs2::TargetTrajectories planToTargetTrajectories(
  //     const legged_body_msgs::Plan::ConstPtr& plan);

  /**
   * @brief Primary work function in class. Called in node file for this
   * component
   */
  void spin();

 private:
  /// @brief Callable objects w/ inputs const vector_t& and const
  /// SystemObservation& w/ return TargetTrajectories
  CmdToTargetTrajectories_t goalToTargetTrajectories_,
      cmdVelToTargeTrajectories_;

  /// @brief Callable object
  PlanToTargetTrajectories_t planToTargetTrajectories_;

  //   /// @brief Callable object to convert plan to target trajectories
  //   PlanToTargetTrajectories_t planToTargetTrajectories_;

  /// TF2 Buffer
  tf2_ros::Buffer buffer_;

  /// Transform listener
  tf2_ros::TransformListener tf2_;

  /// Nodehandle
  ros::NodeHandle nh_;

  /// Subscribers
  ros::Subscriber observation_sub_, goal_sub_, cmd_vel_sub_, body_plan_sub_;

  /// Publisher
  std::unique_ptr<ocs2::TargetTrajectoriesRosPublisher>
      target_trajectories_publisher_;

  /// Mutex on Observer
  mutable std::mutex latest_observation_mutex_;

  /// @brief Observer for states
  ocs2::SystemObservation latest_observation_;

  /// @brief Update rate for sending and receiving data
  double update_rate_;
};

#endif  // TRAJECTORIES_PUBLISHER_H
