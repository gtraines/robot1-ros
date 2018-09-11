-# add repos to sources.list

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list' \
&& wget http://packages.ros.org/ros.key -O - | sudo apt-key add - \
&& apt-get update \
&& apt-get install ros-kinetic-desktop

# -y = assume yes for prompts for additional dependency packages
sudo apt-get install ros-kinetic-rosbridge-suite -y 


