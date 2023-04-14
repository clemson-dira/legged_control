#ifndef TRAJECTORIES_PUBLISHER_H
#define TRAJECTORIES_PUBLSIHER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

class TrajectoriesPublisher final {  // Ensure this class has no derived class
 public:
  /**
   * @brief Construct a new Trajectories Publisher object
   */
  TrajectoriesPublisher(::ros::NodeHandle& nh, const std::string& topicPrefix);
};

#endif  // TRAJECTORIES_PUBLISHER_H
