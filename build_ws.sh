#!/usr/bin/env sh

#build current workspace
catkin_make && $PWD/devel/setup.bash
