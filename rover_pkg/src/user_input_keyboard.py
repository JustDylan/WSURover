#!/usr/bin/env python3.8

import rospy
import keyboard
import numpy as np
#import pygame

from rover_pkg.msg import UserInput

global P
P = [0,0,0,0]

def getInputs():
	input_stream_publisher = rospy.Publisher('user_input_stream', UserInput, queue_size=10)
	rospy.init_node('user_input_node')
	rate = rospy.Rate(50)
	rospy.loginfo("User Input lnode started")
	#pygame.init()
	
	while not rospy.is_shutdown():
		keyList = ""
		charsToLookFor = ['a','b','c','d' ,'e' ,'f' ,'g' ,'h' ,'i' ,'j' ,
		'k' ,'l' ,'m' ,'n' ,'o' ,'p' ,'q' ,'r' ,'s' ,'t' ,'u' ,'v' ,'w' ,
		'x', 'y', 'z']
		
		for char in charsToLookFor:
			if keyboard.is_pressed(char):
				keyList = keyList + char

		#x1 = pygame.joystick.Joystick(0).get_axis(1)
		#y1 = pygame.joystick.Joystick(0).get_axis(0)
		#x2 = pygame.joystick.Joystick(1).get_axis(0)
		#y2 = pygame.joystick.Joystick(1).get_axis(1)
		x1 = 0
		y1 = 0
		x2 = 0
		y2 = 0
		
		
		if x1 > 0.2 or x1 < -0.2 :
			P[0]=x1
		else:
			P[0]=0
		if y1 > 0.2 or y1 < -0.2 :	
			P[1]=y1
		else:
			P[1]=0
		if x2 > 0.2 or x2 < -0.2 :
			P[2]=x2
		else:
			P[2]=0
		if y2 > 0.2 or y2 < -0.2 :	
			P[3]=y2
		else:
			P[3]=0
			
		user_input = UserInput()
		user_input.controlMode = "keyboard_controller"
		user_input.keysPressed = keyList
		user_input.ljx = P[2]
		user_input.rjx = P[0]
		user_input.ljy = P[3]
		user_input.rjy = P[1]
		
		input_stream_publisher.publish(user_input)		
		rate.sleep()

	
if __name__=="__main__":
	try:
		getInputs()
	except rospy.ROSInterruptException:
		pass		

		




  

