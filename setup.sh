### Install all dependencies

# ROS Dependencies for catkin tools
echo
echo "###############################"
echo "Install base dependencies"
echo "###############################"
echo

sudo apt install -y ros-$ROS_DISTRO-catkin python3-catkin-tools
sudo apt install -y ros-$ROS_DISTRO-grid-map
# Eigen library
sudo apt install -y libeigen3-dev
# Doxygen
sudo apt install -y doxygen doxygen-doc doxygen-gui graphviz

# Install graphing utils
sudo apt install -y ros-noetic-plotjuggler-ros

# Install all dependencies from package setup files
echo
echo "###########################################"
echo "Install dependencies in package setup files"
echo "###########################################"
echo

CWD="$(pwd)"
for f in $(find -name 'setup_deps.sh'); do # Find files w/ name setup_deps.sh
    echo "Found package setup file $f"
    BASE=$(dirname $f) # Get directory name of file with setup_deps.sh
    cd "$BASE" # Quotation to ensure changing to corresponding dir
    chmod +x setup_deps.sh
    ./setup_deps.sh
    cd "$CWD"
done

# TODO (AZ): Install rosdeps