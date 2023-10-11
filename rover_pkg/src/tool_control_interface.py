#!/usr/bin/env python
import rospy
from rover_pkg.msg import ToolData
from rover_pkg.msg import UserInput		

def inputCallback(user_data):
	keys_pressed = user_data.keysPressed
	data_out = ToolData()	
	if 'z' in keys_pressed:
		# set target
		data_out.QDC_SIG1 = -50
	if 'x' in keys_pressed:
		data_out.QDC_SIG1 = 50
	if len(keys_pressed)==0:
		data_out.QDC_SIG1 = 0
	data_out.tool_select = 1

	ToolDataPublisher = rospy.Publisher('tool_control_stream', ToolData, queue_size=10)
	ToolDataPublisher.publish(data_out)	

	
def initialize():	
	rospy.init_node('tool_control_interface_node', anonymous=True)	
	rospy.Subscriber('user_input_stream', UserInput, inputCallback)
	rospy.loginfo("Tool Controller node started")
	rospy.spin()

if __name__ == '__main__':
	try:
		initialize()
	except rospy.ROSInterruptException:
		pass
		
		
		
		
		
		
		
		
		

