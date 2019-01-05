#! /usr/bin/env bash

rosparam set joy_node/dev "/dev/input/js0"
rosrun joy joy_node

rostopic echo joy
