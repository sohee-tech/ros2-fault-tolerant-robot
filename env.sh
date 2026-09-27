# source ~/capstone_fault_ws/env.sh
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH
source ~/capstone_fault_ws/install/setup.bash
# Fast DDS: UDP only (see src/fault_bringup/config/fastdds_udp.xml)
export FASTRTPS_DEFAULT_PROFILES_FILE=~/capstone_fault_ws/src/fault_bringup/config/fastdds_udp.xml
