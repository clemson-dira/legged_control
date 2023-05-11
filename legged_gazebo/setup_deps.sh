
# Ensure gazebo model path is found
GAZEBO_MODEL_PATH_UPDATE="export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$(pwd)/worlds"

if grep "$GAZEBO_MODEL_PATH_UPDATE" ~/.bashrc > /dev/null
then
    echo "bashrc contains gazebo model path update"
else
    echo "Adding gazebo model path update to bashrc"
    echo "$GAZEBO_MODEL_PATH_UPDATE" >> ~/.bashrc
fi