sudo apt install ros-humble-realsense2-* -y

cd ~/ros2_ws
sudo apt-get install python3-rosdep -y
sudo rosdep init # "sudo rosdep init --include-eol-distros" for Foxy and earlier
rosdep update # "sudo rosdep update --include-eol-distros" for Foxy and earlier
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y

colcon build

ROS_DISTRO=humble  # set your ROS_DISTRO: iron, humble, foxy
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/ros2_ws
. install/local_setup.bash

