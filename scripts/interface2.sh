#1/bin/sh
source ../../../devel/setup.bash
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_IP=192.168.1.102
rosrun autonomous_pkg camera_interface &
rosrun interface_pkg gamepad_interface &
wait
echo "Interface done"
