#!/bin/bash

# Using ROS Indigo

# Single machine configuration (added to ~/.bashrc)
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311

# Path vars (also added to ~/.bashrc)
export ROS_DIR=/mnt/STORAGE/Dropbox/Personal/Programming/Bioloid/Linux/ROS
export CATKIN_WS=$ROS_DIR/catkin_ws

# Source the environment setup file
source /opt/ros/indigo/setup.bash


# Create a ROS workspace
# ----------------------
cd $ROS_DIR
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS/src
catkin_init_workspace
# Build
cd ..
catkin_make
# Source the new setup.*sh file (also added to ~/.bashrc)
source devel/setup.bash


# Create a ROS package (beginner tutorials)
# -----------------------------------------
cd $CATKIN_WS/src
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
# Build
cd $CATKIN_WS
catkin_make
# Also see: http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
#           http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29


# Create a ROS package (USB2AX Controller)
# ----------------------------------------
cd $CATKIN_WS/src
catkin_create_pkg usb2ax_controller std_msgs rospy roscpp
# Copy the USB2AX library to src
# Note: USB2AX uses a different dxl_hal.c file than the Robotis' Dynamixel SDK for Linux
# See link: http://www.xevelabs.com/doku.php?id=product:usb2ax:faq#qdynamixel_sdkhow_do_i_use_it_with_the_usb2ax
cp -r /mnt/STORAGE/Dropbox/Personal/Programming/Bioloid/Linux/Program_Files/USB2AX $CATKIN_WS/src/usb2ax_controller/src/usb2ax
# Build
cd $CATKIN_WS
catkin_make


# Custom rosconsole.config file (to avoid logging all INFO messages to file) (also added to ~/.bashrc)
export ROSCONSOLE_CONFIG_FILE=$ROS_DIR/myrosconsole.config


# My roslaunch/rosrun files

## Simulated robot ##
# Load robot description and start state publishers
roslaunch bioloid_master bioloid_pubs.launch dummy_imu:=true
# Visualise robot using URDF, no IMU
roslaunch bioloid_master bioloid_pubs.launch dummy_imu:=true
roslaunch bioloid_master bioloid_rviz.launch                                    # Start RViz, publish robot/joint state, publish dummy IMU frame
#####################

## Real robot ##
# Visualise real sensor data
roslaunch bioloid_sensors_interface avr_sensors_serial_plot.launch              # Start sensors interface, rqt_plot
# Visualise robot using URDF
roslaunch usb2ax_controller controller.launch                                   # Initialise servo controller with position control disabled
roslaunch bioloid_sensors_interface avr_sensors_serial_process.launch           # Start sensors interface, publish IMU frame
roslaunch bioloid_master bioloid_rviz.launch                                    # Start RViz, publish robot/joint state
# Test servo motion
roslaunch usb2ax_controller controller.launch
roslaunch bioloid_sensors_interface avr_sensors_serial_process.launch
rosrun usb2ax_controller test_interface                                         # Run test program: home position and arm wave
# Test PID balancer
roslaunch usb2ax_controller controller.launch
roslaunch bioloid_sensors_interface avr_sensors_serial_process.launch
rosrun usb2ax_controller test_balancer                                          # Run test program: home position and balancing
################

## MoveIt! ##
# Configure the MoveIt SRDF
roslaunch bioloid_moveit_config setup_assistant.launch                          # Start MoveIt Setup Assistant
# Run the MoveIt RViz plugin
roslaunch bioloid_moveit_config demo.launch                                     # Start RViz with MoveIt MotionPlanning plugin
# Run the MoveIt RViz plugin with default database
roslaunch bioloid_moveit_config demo.launch db:=true                            # Start RViz with MoveIt MotionPlanning plugin and default database
# Run the MoveIt RViz plugin with custom database
roslaunch bioloid_moveit_config warehouse.launch moveit_warehouse_database_path:=~/bioloid_warehouse_mongo_db
roslaunch bioloid_moveit_config demo.launch
# Test MoveIt Move Group API
rosrun bioloid_master moveit_api_test                                           # Run test program: MoveIt Move Group Interface/C++ API
# Run MoveIt with ros_control controller interface to hardware
roslaunch usb2ax_controller controller.launch pos_control:=true                 # Position control enabled. Use arg dummy_imu:=true if IMU is not running.
roslaunch bioloid_master bioloid_controllers.launch                             # Start ros_control controller spawner
roslaunch bioloid_sensors_interface avr_sensors_serial_process.launch
roslaunch bioloid_moveit_config demo_custom.launch                              # Loads controller configuration instead of fake controllers
#############

# GUIs ##
# Bioloid Sensors Visualiser (Python, for Raspberry Pi 2)
python /mnt/STORAGE/Dropbox/Personal/Programming/Bioloid/Linux/Raspberry_Pi_2/Pi.py
# ROSoloid Control GUI (Qt, for PC)
cd $CATKIN_WS && rosrun bioloid_master rosoloid_gui                             # Run from catkin workspace in order to correctly locate relative assets path
#########


# Notes

# Getting MongoDB to work with NTFS: Add "permissions" flag to /etc/fstab for drive to be mounted
# See link: http://askubuntu.com/questions/251901/configuring-mongo-db-to-work-with-ntfs-drive
