#!/usr/bin/env python3.6
import rospy

from rover_pkg.msg import UserInput
from rover_pkg.msg import SensorData
from rover_pkg.msg import ArmData

still = 1450
speed2 = 250
speed = 300

def userCallback(user_data):

	keys_pressed = user_data.keysPressed

	
	if 'i' in keys_pressed:
		arm_zm = still + speed2
	elif 'j' in keys_pressed:
		arm_zm = still - speed2
	else:
		arm_zm = still		

	if 'k' in keys_pressed:
		arm_bm = still + speed
	elif 'o' in keys_pressed:
		arm_bm = still - speed
	else:
		arm_bm = still
		
	if 'p' in keys_pressed:
		arm_am = still + speed
	elif 'l' in keys_pressed:
		arm_am = still - speed
	else:
		arm_am = still
		
	if 'h' in keys_pressed:
		arm_wp = still + speed2
	elif 'u' in keys_pressed:
		arm_wp = still - speed2
	else:
		arm_wp = still

	if 'g' in keys_pressed:
		arm_wr = still + speed2
	elif 'y' in keys_pressed:
		arm_wr = still - speed2
	else:
		arm_wr = still		
		
	armData = ArmData()
	armData.arm_zm = int(arm_zm)
	armData.arm_bm = int(arm_bm)
	armData.arm_am = int(arm_am)
	armData.arm_wp = int(arm_wp)
	armData.arm_wr = int(arm_wr)
	
	ArmPosPublisher = rospy.Publisher('basic_arm_control_stream', ArmData, queue_size=10)
	ArmPosPublisher.publish(armData)
	

def init():
	
	rospy.init_node('basic_arm_control_interface', anonymous=True)
	rospy.Subscriber('user_input_stream', UserInput, userCallback)
	
	rospy.loginfo("Basic Arm Control node started")
	rospy.spin()

if __name__ == '__main__':
	try:
		init()
	except rospy.ROSInterruptException:
		pass
		
		
		
		
		
		
