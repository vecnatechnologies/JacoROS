#NEED TO INSTALL ROS FUERTE
#TESTED ON UBUNTU 12.04

#ADD this to your .bashrc file

#ROS
source /opt/ros/fuerte/setup.bash

#KINOVA ROS PACKAGES
export ROS_PACKAGE_PATH=<trunk directory where the Kinova files are>:$ROS_PACKAGE_PATH

#COMPILE & enjoy!
rosdep install kinova_jaco
rosmake kinova_jaco

#MAKE SURE YOU HAVE THE PERMISSIONS ON THE USB PORT!

#RUN TEST
roslaunch jaco_interactive_markers standalone_markers.launch

