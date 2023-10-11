#!/usr/bin/env python
import rospy
from rover_pkg.msg import CameraControlData
from rover_pkg.msg import UserInput

tilt = 1500
pan = 1500
maxA = 2000
minA = 1000

def keyboard_controller(user_data):
	keys_pressed = user_data.keysPressed
	# use keys_pressed data to update rover data
	speed = 50
	global pan
	global tilt
	print(tilt)
	if 'f' in keys_pressed and tilt < maxA:
		tilt = tilt +speed;
	if 'c' in keys_pressed and tilt > minA:
		tilt = tilt -speed;
	if 'v' in keys_pressed and pan < maxA:
		pan = pan + speed;		
	if 'b' in keys_pressed and pan > minA:
		pan = pan - speed;
		
	# publish the new data
	data_out = CameraControlData()	
	data_out.ctt = tilt
	data_out.ctp = pan
	data_out.tool_select = 1
	
	if 'z' in keys_pressed:
		# set target
		data_out.QDC_SIG1 = -50
	if 'x' in keys_pressed:
		data_out.QDC_SIG1 = 50
		
	if len(keys_pressed)==0:
		data_out.QDC_SIG1 = 0
		
	CameraDataPublisher = rospy.Publisher('camera_control_stream', CameraControlData, queue_size=10)
	CameraDataPublisher.publish(data_out)
		
			

def inputCallback(user_data):
	if (user_data.controlMode == "basic_controller"):
		basic_controller(user_data)
		
	if (user_data.controlMode == "keyboard_controller"):
		keyboard_controller(user_data)

	
def initialize():	
	rospy.init_node('camera_control_control_interface_node', anonymous=True)	
	rospy.Subscriber('user_input_stream', UserInput, inputCallback)
	rospy.loginfo("Camera Control Controller node started")
	rospy.spin()

if __name__ == '__main__':
	try:
		initialize()
	except rospy.ROSInterruptException:
		pass
		
		
		
		
		
		
		
		
		

