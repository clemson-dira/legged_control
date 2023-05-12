# Installing Legged Control on ROS Melodic

## Install ros melodic desktop ONLY
Then follow rest of instructions
Install catkin tools
	- https://stackoverflow.com/questions/41234957/catkin-command-not-found
	- Use melodic version

update Cmake greater than 3.14 usinf following link:
	- https://askubuntu.com/questions/829310/how-to-upgrade-cmake-in-ubuntu
	- Install Linux x86_64 shell version



### Build files
Install dependencies
```
sudo apt-get install doxygen
sudo apt-get install ros-melodic-octomap
sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev
sudo apt-get install ros-melodic-hardware-interface
sudo apt-get install ros-melodic-realtime-tools
sudo apt-get install ros-melodic-geometry2
sudo apt-get install ros-melodic-controller-interface
sudo apt install ros-melodic-ros-control* ros-melodic-control*
```
IF WANT TO INSTALL GAZEBO on HW, DO THE FOLLOWING:
```
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
```
Change publishTransforms, etc. to ros melodic syntax
- https://github.com/leggedrobotics/ocs2/issues/28
- Or look up ros melodic documentation of the member functions

Error in REST request
- address by the following
- https://answers.gazebosim.org//question/25030/gazebo-error-restcc205-error-in-rest-request/

### LCM Dependency
Will run into -llcm error, either clone or download lcm, put into src/ of ws, build as instructed in github
- Follow Ubuntu/Debian section
- https://lcm-proj.github.io/lcm/content/build-instructions.html#ubuntu-and-debian

### Building Gazebo 
Building legged_gazebo
- Change the following legged hw
```
...
struct HybridJointData {
  HybridJointData(hardware_interface::JointHandle joint) : joint_(joint){} // ros-melodic changes | add initializer list to constructor
  hardware_interface::JointHandle joint_;
  double posDes_{}, velDes_{}, kp_{}, kd_{}, ff_{};
};


struct HybridJointCommand {
  HybridJointCommand(ros::Time stamp, double posDes, double velDes, double kp, double kd, 
    double ff) : posDes_(posDes), velDes_(velDes), kp_(kp), kd_(kd), ff_(ff) {} // ros-melodic changes | add initializer list to constructor
  ros::Time stamp_;
  double posDes_{}, velDes_{}, kp_{}, kd_{}, ff_{};
};
...
```

### Build Legged Body Planner w/ mapping
Install elevation mapping / grid map dependencies

```
# Install dependencies
sudo apt-get install ros-$ROS_DISTRO-grid-map
sudo apt-get install libeigen3-dev
sudo apt install libpcl-dev
sudo apt install ros-melodic-pcl-ros
sudo apt install pcl-tools
sudo apt-get install python-rospkg
# Clone repos
cd ~/legged_robot_ws/src
git clone https://github.com/anybotics/elevation_mapping.git
git clone git@github.com:ANYbotics/kindr.git
git clone git@github.com:ANYbotics/kindr_ros.git
git clone git@github.com:ANYbotics/message_logger.git
```
## Code Adjustment for melodic
Change C++ stl function
- In ocs2 core Numerics.h
```
const auto prec = std::numeric_limits<typename std::remove_reference<T1>::type>::epsilon();
```
Change python version
- In `legged_mapping/src/tf_to_pose_publisher.py`
```
#!/usr/bin/env python
# Change shebang from #!/usr/bin/env python -> #!/usr/bin/env python
...
```

Change elevation map accordingly
Source: https://github.com/ANYbotics/elevation_mapping/issues/151

```
cd ~/legged_robot_ws/src/elevation_mapping
```
Change the `PLC_MAKE_ALIGNED_OPERATOR_NEW` to `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` in a file... (just build elevation_mapping to find the file location)

Change in `PostProcessingPipelineFunctor.hpp` the following
```
# Change #include <filters/filter_chain.hpp> to
...
#include <filters/filter_chain.h>
```


Build legged body planner, etc.
```
catkin build legged_body_planner legged_body_msgs legged_mapping
```


