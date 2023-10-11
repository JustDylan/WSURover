#!/usr/bin/env python3

import rospy
import numpy as np
import pygame
from rover_pkg.msg import UserInput

pygame.init()
pygame.joystick.init()

P = [ 0 , 0 , 0 , 0 ]

def init():
	while True:
		rospy.init_node('user_input_stream')
		pygame.init()
		print(P)
		x1 = pygame.joystick.Joystick(0).get_axis(0)
		y1 = pygame.joystick.Joystick(0).get_axis(1)
		x2 = pygame.joystick.Joystick(1).get_axis(0)
		y2 = pygame.joystick.Joystick(1).get_axis(1)
		if x1 > 0.1 or x1 < -0.1 :
			P[0]=x1
		else:
			P[0]=0
		if y1 > 0.1 or y1 < -0.1 :	
			P[1]=y1
		else:
			P[1]=0
		if x2 > 0.1 or x2 < -0.1 :
			P[2]=x2
		else:
			P[2]=0
		if y2 > 0.1 or y2 < -0.1 :	
			P[3]=y2
		else:
			P[3]=0
		control_state = UserInput()
		control_state.controlMode = 'basic_controller'
		control_state.ljx = y2
		control_state.rjx = x1

		ControlPublisher = rospy.Publisher('user_input_stream', UserInput, queue_size=10)
		ControlPublisher.publish(control_state)

		
if __name__ == '__main__':
	try:
		init()
	except rospy.ROSInterruptException:
		pass
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
