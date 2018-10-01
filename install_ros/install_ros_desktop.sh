sudo apt-get update && \
sudo apt-get install ros-kinetic-desktop-full && \
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "export PYTHONPATH=/usr/lib/python2.7/dist-packages:$PATH"
source ~/.bashrc
