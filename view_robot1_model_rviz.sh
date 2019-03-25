#!/usr/bin/env bash

# If not already running, run `roscore` to start master node
currdir=${PWD}
echo ${currdir}
echo ${PWD}
turrpath="${currdir}/src/robot1_chassis_description/urdf/em_3905.urdf.xacro"
roslaunch urdf_tutorial display.launch model:=${turrpath}
