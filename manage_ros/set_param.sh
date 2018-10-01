#!/usr/bin/env bash

rosparam set ${1} ${2} # parameter value
rosparam get ${1} # shows the value`

rosparam list
