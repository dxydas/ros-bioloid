# ROS config

# Single machine configuration
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311

# Path vars
export ROS_DIR=/mnt/STORAGE/Dropbox/Personal/Programming/Bioloid/Linux/ROS
export CATKIN_WS=$ROS_DIR/catkin_ws

# Source environment setup file
#source /opt/ros/indigo/setup.bash
source $CATKIN_WS/devel/setup.bash

# Custom rosconsole.config file (to avoid logging all INFO messages to file)
export ROSCONSOLE_CONFIG_FILE=$CATKIN_WS/my_rosconsole.config
