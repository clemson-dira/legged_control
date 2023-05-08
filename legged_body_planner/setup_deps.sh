sudo apt install -y liburdfdom-dev liboctomap-dev libassimp-dev
# Python dependencies
pip3 install sympy # Symbolic python package
pip3 install numpy # Numpy
# ROS Dependencies for catkin tools
sudo apt install -y ros-$ROS_DISTRO-catkin python3-catkin-tools
sudo apt install -y ros-$ROS_DISTRO-grid-map
# Eigen library
sudo apt install -y libeigen3-dev
# Doxygen
sudo apt install -y doxygen doxygen-doc doxygen-gui graphviz

# Install graphing utils
sudo apt install -y ros-noetic-plotjuggler-ros