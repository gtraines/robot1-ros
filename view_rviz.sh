#!/usr/bin/env bash
currdir=${PWD}
echo $currdir
echo ${PWD}
turrpath="$currdir/src/turret_urdf/turret_test1.urdf"
roslaunch urdf_tutorial display.launch model:=$turrpath
