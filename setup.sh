#!/bin/bash

echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/ros-latest.list 
sudo apt update
sudo apt install ros-melodic-fetch-calibration ros-melodic-fetch-open-auto-dock \
ros-melodic-fetch-navigation ros-melodic-fetch-tools ros-melodic-turtlebot3 ros-melodic-fetch-simulation -y
sudo apt install ros-sensor-msgs
sudo apt install ros-geometry-msgs
sudo apt install ros-melodic-map-server
sudo apt install ros-melodic-fetch-gazebo
sudo apt install ros-melodic-ros-numpy
sudo apt install python-opencv
sudo apt install python-numpy


sudo sed -i 's/<limit effort="0.68" lower="-0.76" upper="1.45"/<limit effort="0.68" lower="0.7" upper="1"/' /opt/ros/melodic/share/fetch_description/robots/fetch.urdf

sudo sed -i 's/DarkGrey/Orange/' /opt/ros/melodic/share/turtlebot3_description/urdf/turtlebot3_burger.gazebo.xacro

echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc