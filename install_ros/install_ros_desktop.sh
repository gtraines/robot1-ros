sudo apt-get update -y
sudo apt-get install ros-kinetic-desktop-full -y
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "export PYTHONPATH=/usr/lib/python2.7/dist-packages:$PATH" >> ~/.bashrc
source ~/.bashrc
