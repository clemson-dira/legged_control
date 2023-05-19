# Installing Legged Control on ROS Melodic
Rough notes of converting legged control to be ros melodic compatible

***Note:*** Notes are rough and assumes that the workspace is named legged_robot_ws


## Install ROS Melodic Desktop ONLY
This is so that it can be more lightweight

Install ros melodic through ros website, then follow the instructions sequentially

Install catkin tools
- https://stackoverflow.com/questions/41234957/catkin-command-not-found
- Use melodic version

Upgrade CMake s.t. greater than 3.14 using the following link:
- https://askubuntu.com/questions/829310/how-to-upgrade-cmake-in-ubuntu




## STEPS IN CONVERTING NOETIC TO MELODIC COMPATIBILITY for OCS2 & Legged Control
### Install Dependencies
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

### Install Legged Control
Install legged control dependencies / clone this repo
```
git clone git@github.com:clemson-dira/legged_control.git
```
Install rest of dependencies listed in legged control `README.md`

### Syntactical Edits
Change publishTransforms, etc. to ros melodic syntax (in LeggedRobotVisualizer.cpp)
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

## Build Legged Body Planner w/ Mapping on ROS Melodic
Install elevation mapping / grid map dependencies

```
# Install dependencies
sudo apt-get install ros-$ROS_DISTRO-grid-map
sudo apt-get install libeigen3-dev
sudo apt install libpcl-dev
sudo apt install ros-melodic-pcl-ros
sudo apt install pcl-tools
sudo apt-get install python-rospkg
sudo apt install ros-$ROS_DISTRO-plotjuggler-ros
# Clone repos
cd ~/legged_robot_ws/src
git clone https://github.com/anybotics/elevation_mapping.git
git clone git@github.com:ANYbotics/kindr.git
git clone git@github.com:ANYbotics/kindr_ros.git
git clone git@github.com:ANYbotics/message_logger.git
```
### Elevation Mapping Code Adjustments
Change C++ stl function
- In ocs2 core Numerics.h
  - Change std::remove_reference to C++11 version
```
const auto prec = std::numeric_limits<typename std::remove_reference<T1>::type>::epsilon();
```
  - Additionally, all the lines w/ `std::remove_reference_t<T1>` --> `std::remove_reference<T1>::type 
Change python version
- In `legged_mapping/src/tf_to_pose_publisher.py`
```
#!/usr/bin/env python
# Change shebang from #!/usr/bin/env python -> #!/usr/bin/env python
...
```

Change elevation map accordingly

Source: https://github.com/ANYbotics/elevation_mapping/issues/151 (Don't think I changed to source branch... kept with master.. TODO: double check)

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

## Build
Build from legged control instructions
Build legged body planner, etc.
```
catkin build elevation_mapping kindr kindr_ros message_logger
catkin build legged_body_planner legged_body_msgs legged_mapping legged_body_utils
source ~/legged_robot_ws/devel/setup.bash
```
Hopefully, everything is built successfully!

## ARM SPECIFICS
This section describes some of the detail of arm specific changes that was done

### HPIPM / Blasfeo
In `ocs2/ocs2_sqp`, need to change the CMakeLists.txt file of `hpipm_catkin` and `blasfeo_catkin`

FetchContent_Declare git repo and git tag needs to be changed such that it is of the following
```
# For hpipm_catkin
FetchContent_Declare(hpipmDownload
  GIT_REPOSITORY https://github.com/clemson-dira/hpipm
  GIT_TAG 0ca7dfcaebf36969bd9b0ee34063f07817566605
  UPDATE_COMMAND ""
  SOURCE_DIR ${HPIPM_DOWNLOAD_DIR}
  BINARY_DIR ${HPIPM_BUILD_DIR}
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND "$(MAKE) install"
)
```

```
# For blasfeo_catkin
FetchContent_Declare(hpipmDownload
  GIT_REPOSITORY https://github.com/clemson-dira/hblasfeopipm
  GIT_TAG 3adca0b940bf8332cd8f0efb34bd13cfa56154d1
  UPDATE_COMMAND ""
  SOURCE_DIR ${HPIPM_DOWNLOAD_DIR}
  BINARY_DIR ${HPIPM_BUILD_DIR}
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND "$(MAKE) install"
)
```

### Opencv
Could  not get grid_map_cv to be specified in /usr/include/opencv
- https://github.com/ANYbotics/grid_map/issues/291
- Changed for grid_map_cv and and grid_map_filters to have conditional and set condition to include opencv4



