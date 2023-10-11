#!/usr/bin/env python3.6

import rospy
import numpy as np
import pygame

pygame.init()
pygame.joystick.init()

P = [ 0 , 0 ]

def init():
	while True:
		pygame.init()
		print(P)
		x1 = pygame.joystick.Joystick(0).get_axis(0)
		x2 = pygame.joystick.Joystick(0).get_axis(1)
		if x1 > 0.3 or x1 < -0.1 :
			P[0]=P[0]+x1
		if x2 > 0.3 or x2 < -0.1 :	
			P[1]=P[1]+x2
		
		
if __name__ == '__main__':
	try:
		init()
	except rospy.ROSInterruptException:
		pass
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
