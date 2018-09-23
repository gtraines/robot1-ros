#!/usr/bin/env bash
# catkin_create_pkg --rosdistro Kinetic {1} dependencies rospy

args=("$@")
dependency_arr=${args[*]:2}

echo "$(tput setab 2)$(tput setaf 0) Creating package ${2} $(tput sgr0)"
cd $PWD/src/
catkin_create_pkg --rosdistro Kinetic -m graham.traines -l MIT $2 std_msgs message_generation message_runtime $1 ${dependency_arr}

echo "$(tput setaf 2) Created package ${2} $(tput sgr0)"
echo "$(tput bold) Current dependencies: $(tput sgr0)"

#rospack depends1 $2
cd .. 

# this is a hack block comment
if [ ]; then
catkin_create_pkg [-h] [--meta] [-s [SYS_DEPS [SYS_DEPS ...]]]
                         [-b [BOOST_COMPS [BOOST_COMPS ...]]] [-V PKG_VERSION]
                         [-D DESCRIPTION] [-l LICENSE] [-a AUTHOR]
                         [-m MAINTAINER] [--rosdistro ROSDISTRO]
                         name [dependencies [dependencies ...]]

	Direct
	Call tput as part of a sequence of commands:
		tput setab [1-7] # Set the background colour using ANSI escape
		tput setaf [1-7] # Set the foreground colour using ANSI escape

	tput setaf 1; echo "this is red text"
	Use ; instead of && so if tput errors the text still shows.

	Colours are as follows:

	Num  Colour    #define         R G B

	0    black     COLOR_BLACK     0,0,0
	1    red       COLOR_RED       1,0,0
	2    green     COLOR_GREEN     0,1,0
	3    yellow    COLOR_YELLOW    1,1,0
	4    blue      COLOR_BLUE      0,0,1
	5    magenta   COLOR_MAGENTA   1,0,1
	6    cyan      COLOR_CYAN      0,1,1
	7    white     COLOR_WHITE     1,1,1
	
	Shell variables
	Another option is to use shell variables:

	red=`tput setaf 1`
	green=`tput setaf 2`
	reset=`tput sgr0`
	echo "${red}red text ${green}green text${reset}"
	
	Text mode commands
	tput bold    # Select bold mode
	tput dim     # Select dim (half-bright) mode
	tput smul    # Enable underline mode
	tput rmul    # Disable underline mode
	tput rev     # Turn on reverse video mode
	tput smso    # Enter standout (bold) mode
	tput rmso    # Exit standout mode


	#╔═════════════════╦════════════════════════════════════════╗
	#║ Syntax          ║ Result                                 ║
	#╠═════════════════╬════════════════════════════════════════╣
	#║ arr=()          ║ Create empty array                     ║
	#║ arr=(1 2 3)     ║ Initialize array                       ║
	#║ ${arr[2]}       ║ Retrieve third element                 ║
	#║ ${arr[@]}       ║ Retrieve all elements                  ║
	#║ ${!arr[@]}      ║ Retrieve array indices                 ║
	#║ ${#arr[@]}      ║ Calculate array size                   ║
	#║ arr[0]=3        ║ Overwrite 1st element                  ║
	#║ arr+=(4)        ║ Append value(s)                        ║
	#║ str=$(ls)       ║ Save ls output as string               ║
	##║ arr=( $(ls) )   ║ Save ls output as array of files       ║
	#║ ${arr[@]:s:n}   ║ Elements at indices n to s+n           ║
	#║ ${str//ab/c}    ║ For a given string, replace ab with c  ║
	#║ ${arr[@]//ab/c} ║ For each array item, replace ab with c ║
	#╚═════════════════╩════════════════════════════════════════╝

fi

