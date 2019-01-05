#!/usr/bin/env bash

(roslaunch ${PWD}/src/robot1_chassis_gazebo/launch/robot1_chassis.launch &)
(./view_rviz.sh &)
