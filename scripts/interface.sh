#1/bin/sh
source ../../../devel/setup.bash
/bin/sh -eci "rosrun autonomous_pkg camera_interface" &
/bin/sh -eci "rosrun interface_pkg gamepad_interface"
wait
echo "Interface done"
