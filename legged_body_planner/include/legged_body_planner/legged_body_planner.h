#ifndef LEGGED_BODY_PLANNER_H
#define LEGGED_BODY_PLANNER_H

#include <legged_body_msgs/Control.h>
#include <legged_body_msgs/Plan.h>
#include <legged_body_msgs/State.h>
#include <legged_body_planner/RobotConfiguration.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ros/ros.h>

#include <mutex>

//! Body Planning class for legged robots
/*!
  LeggedBodyPlanner is a container for logic utilized in legged body planner
  node. This algorithm requires terrain information (future work as GridMap
  message type). Accordingly, the class will call a planning algorithm class,
  which will be converted to legged rigid body model plan. The plan will publish
  these state trajectories in the form of discretized states.
*/
class LeggedBodyPlanner {
 public:
  /**
   * @brief Construct a new Legged Body Planner object
   *
   */
  LeggedBodyPlanner(ros::NodeHandle& nh, std::string topic_prefix);

  /**
   * @brief Primary work function in class. Called in node file for this
   * component
   */
  void spin();

 private:
  template <typename ParamType>
  inline bool loadROSParam(ros::NodeHandle& nh, std::string param_name,
                           ParamType& var_name) {
    if (!nh.getParam(param_name, var_name)) {
      ROS_ERROR("Can't find param %s from parameter server",
                param_name.c_str());
      return false;
    }
    return true;
  }

  /**
   * @brief Function to read state trajectory from a file
   */
  void readStateTrajectories();

  /**
   * @brief Get current observation
   * @param msg Observer message
   */
  void observerCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg);

  /**
   * @brief Terrain callback (TODO (AZ): probably a service)
   */
  void terrainCallback();

  /**
   * @brief Converts a vector of rigid body state to RobotConfiguration msg
   * @param v STL vector
   * @param s RobotConfiguration obtained from STL vector
   */
  void vectorToRigidBodyState(const std::vector<double>& v,
                              legged_body_msgs::State& s);

  /// @brief Nodehandle
  ros::NodeHandle nh_;

  /// Subscriber for planning algorithm (TBD later).. rn just use planning
  /// function
  ros::Subscriber planning_algorithm_sub_;

  /// @brief Subscriber for goal state
  ros::Subscriber goal_configuration_sub_;

  /// @brief Publisher for body plan messages
  ros::Publisher body_plan_pub_;

  /// @brief Mutex on observer
  mutable std::mutex latest_observation_mutex_;

  /// @brief Observer of the system
  ocs2::SystemObservation latest_observation_;

  /// @brief ID for status of planner
  int planner_status_;

  /// @brief Time of planner
  ros::Time plan_time_;  // TODO (AZ) : Do I want ros time?

  /// @brief Update rate for the legged body planner
  double update_rate_;

  /// @brief Timestep for planner
  double dt_;

  // Robot parameters

  /// @brief Center of mass height of robot
  ocs2::scalar_t COM_HEIGHT_;

  /// @brief Horizon for planner
  ocs2::scalar_t PLAN_HORIZON_;

  /// @brief Current robot configuration (x_dot, euler rate, x, euler)
  legged_body_msgs::State rigid_body_states_;

  /// @brief Goal robot configuration (2d?)
  legged_body_msgs::State goal_state_;

  /// @brief Control of rigid body
  legged_body_msgs::Control control_;

  /// @brief Plan of rigid body
  legged_body_msgs::Plan body_plan_;
};
#endif  // LEGGED_BODY_PLANNER_H
