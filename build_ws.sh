#!/usr/bin/env bash

#build current workspace
catkin_make && chmod +x $PWD/devel/setup.bash && source $PWD/devel/setup.bash
