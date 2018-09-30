# catkin_create_pkg --rosdistro Kinetic {1} dependencies rospy
echo "Creating package ${1}"

catkin_create_pkg --rosdistro Kinetic $1 dependencies rospy
