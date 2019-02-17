#!/usr/bin/env bash

echo "/*====================================*/"
echo "Adding ROS Kinetic additional packages "
echo "/*====================================*/"
sudo apt-get install ros-kinetic-ackermann-msgs -y
sudo apt-get install ros-kinetic-ackermann-controller -y
sudo apt-get install ros-kinetic-gmapping -y
sudo apt-get install ros-kinetic-teb-local-planner -y

