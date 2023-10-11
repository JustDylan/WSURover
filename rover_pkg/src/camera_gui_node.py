#!/usr/bin/env python3.6

import rospy
import numpy as np
import pygame
import cv2
import time
import os
from cv2 import *
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rover_pkg.msg import CameraControlData
from rover_pkg.msg import UserInput

width = 1900
height = 1000
backgroundColor = (100,100,100)
primaryColor = (255,0,0)
primaryColorDark = (153,0,0)
primaryColorLight = (225,51,51)
borderColor = (150,150,150)
textColor = (200,200,200)
fontName = "Times New Roman"

captureImageMode = 0

imageRatio = 16/9
capture_path = "/home/wsurover/rover_pkg/src/rover_pkg/src/image_captures/"

pygame.init()

image1 = pygame.image.load("/home/wsurover/rover_pkg/src/rover_pkg/src/gui_recources/no_file_loaded.jpeg")
image1raw = pygame.image.load("/home/wsurover/rover_pkg/src/rover_pkg/src/gui_recources/no_file_loaded.jpeg")

image2 = pygame.image.load("/home/wsurover/rover_pkg/src/rover_pkg/src/gui_recources/filler.jpeg")
image2raw = pygame.image.load("/home/wsurover/rover_pkg/src/rover_pkg/src/gui_recources/filler.jpeg")

rawImages = [image1raw,image2raw]
	
def drawTextBox(screen, font, width, x, y, txt1, txt2):
	text_surface = font.render(txt1, True, textColor)
	screen.blit(text_surface, (x,y))
	
	text_width, text_height = font.size(txt2)
	text_surface = font.render(txt2, True, textColor)
	screen.blit(text_surface, (x+width-text_width,y))	

def imageCallback(frame):
	if not captureImageMode:
		br = CvBridge()
		frame = br.compressed_imgmsg_to_cv2(frame)
		frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		global image1
		rawImages[0] = pygame.image.frombuffer(frame.tostring(), frame.shape[1::-1],"RGB")
		image1 = rawImages[0]
		
def imageCallback2(frame):
	if not captureImageMode:
		br = CvBridge()
		frame = br.compressed_imgmsg_to_cv2(frame)
		frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		global image2
		rawImages[1] = pygame.image.frombuffer(frame.tostring(), frame.shape[1::-1],"RGB")
		image2 = rawImages[1]
	
def readCaptureFolder():
	res = []
	for item in os.listdir(capture_path):
		# check if current path is a file
		if os.path.isfile(os.path.join(capture_path, item)):
			res.append(item)    
	return res
	
class Button():
	activated = 0
	def __init__(self, xpos, ypos, width, height, text):
		self.xpos  = xpos
		self.ypos = ypos
		self.width = width
		self.height = height
		self.text = text
		
	def handle(self,screen, screenWidth, screenHeight, font, mpos, click):
		x = screenWidth * self.xpos
		y = screenHeight * self.ypos
		w = screenWidth * self.width
		h = screenHeight * self.height
		text_width, text_height = font.size(self.text)
		
		if (x<mpos[0] and mpos[0]<x+w and y<mpos[1] and mpos[1]<y+h):		
			pygame.draw.rect(screen, primaryColorLight, pygame.Rect(x, y, w, h))
			pygame.draw.rect(screen, primaryColor, pygame.Rect(x, y, w, h),5)
			if clicked:				
				self.activated = not self.activated
		else:
			pygame.draw.rect(screen, primaryColor, pygame.Rect(x, y, w, h))
			pygame.draw.rect(screen, primaryColorDark, pygame.Rect(x, y, w, h),5)
					
		text_surface = font.render(self.text, True, textColor)
		screen.blit(text_surface, (x+w/2-text_width/2,y+h/2-text_height/2))
		
		return self.activated
		
	def deactivate(self):
		self.activated = 0
	
if __name__ == '__main__':

	rospy.init_node("gui_node", anonymous=True)
	rospy.Subscriber('videostream',CompressedImage,imageCallback)
	
	rospy.Subscriber('videostream2',CompressedImage,imageCallback2)
	# Set up the drawing window
	screen = pygame.display.set_mode([width, height], pygame.RESIZABLE)
	pygame.display.set_caption('WSU Rover Team')
	

	# Run until the user asks to quit
	running = True
	userTextInput = []
	
	captureImageButton = Button(0.5, 0.5, 0.1, 0.05, "Capture Image")
	saveImageButton = Button(0.58, 0.68, 0.1, 0.05, "Save")
	cancelImageButton = Button(0.32, 0.68, 0.1, 0.05, "Cancel")	
	selectImageButton = Button(0.45, 0.68, 0.1, 0.05, "Select")
	
	
	cameraHomeButton = Button(0.7, 0.5, 0.1, 0.05, "Home")
	
	tilt_home = 3888
	pan_home = 3888
	pan = pan_home
	tilt = tilt_home
	camera_max = 8600
	camera_min = 1200
	tilt_pos = 0
	pan_pos = 0
	pressed = 0
	pressedPos = [-1,-1]
	drag = 0
	
	while running:

		clicked = 0
		mousePos = pygame.mouse.get_pos()
		userText = ""
		
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				running = False
				
			if event.type == pygame.VIDEORESIZE:
				width, height = event.size
				
			if event.type == pygame.MOUSEBUTTONUP:
				clicked = 1
				pressed = 0
				
			if event.type == pygame.MOUSEBUTTONDOWN:
				pressed = 1
				pressedPos = mousePos
				
			if event.type == pygame.KEYDOWN:
				if event.key == pygame.K_BACKSPACE:  
					userText = userText[:-1]	  
				else:
					userText += event.unicode
			
		point = int(height/42)
		font = pygame.font.SysFont(fontName, point)
		xalignl = width/40
		stroke = 5
		
		# RESET SCREEN
		screen.fill(backgroundColor)
		
		# CAMERA FEED				
		screen.blit(image1,(width/10,height/10))			
		screen.blit(image2,(width/2,height/10))
		
		if captureImageButton.handle(screen, width, height, font, mousePos, clicked):
			captureImageMode = 1
			captureImageButton.deactivate()
		
			
		# CAMERA CONTROL LOGIC
		# publish the new data
		
		maxA = 2000
		minA = 1000
		
		
		center_x = width*0.75
		center_y = height*0.6
		radius = width*0.075
		
		pygame.draw.circle(screen, primaryColor, (center_x+radius,center_y+radius), radius)
		pygame.draw.circle(screen, borderColor, (center_x+radius + tilt_pos*radius,center_y+radius + pan_pos*radius), radius*0.26)
		pygame.draw.circle(screen, primaryColorDark, (center_x+radius + tilt_pos*radius,center_y+radius + pan_pos*radius), radius*0.25)
		
		# detect if mouse is in the joystick
		if ((mousePos[0]-(center_x+radius))**2 + (mousePos[1]-(center_y+radius))**2) < (radius*0.75)**2 and pressed:
			drag = 1
		if drag:
			tilt_pos = (mousePos[0] - pressedPos[0])/radius
			pan_pos = (mousePos[1] - pressedPos[1])/radius
		if not pressed:
			tilt_pos = 0
			pan_pos = 0
			drag = 0
		
		
		data_out = CameraControlData()
		camera_speed = 21
		
		tilt = tilt - tilt_pos*camera_speed
		pan = pan - pan_pos*camera_speed
		
		if tilt < camera_min:
			tilt = camera_min
		if tilt > camera_max:
			tilt = camera_max
		if pan < camera_min:
			pan = camera_min
		if pan > camera_max:
			pan = camera_max
		
		if cameraHomeButton.handle(screen, width, height, font, mousePos, clicked):
			cameraHomeButton.deactivate()			
			pan = pan_home
			tilt = tilt_home
		
		data_out.ctt = int(tilt)
		data_out.ctp = int(pan)
				
		CameraDataPublisher = rospy.Publisher('camera_control_stream', CameraControlData, queue_size=10)
		CameraDataPublisher.publish(data_out)
		
		selectedImage = 0
		fileName = 'file_name'
		hide = 1
		
		while captureImageMode:		
			if hide:		
				hideRect = pygame.Surface((width,height))
				hideRect.fill((250,250,250))
				hideRect.set_alpha(80)
				screen.blit(hideRect, (0,0))
				hide = 0
				
			clicked = 0
			mousePos = pygame.mouse.get_pos()
						
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					pygame.quit()
					
				if event.type == pygame.VIDEORESIZE:
					width, height = event.size
					
				if event.type == pygame.MOUSEBUTTONUP:
					clicked = 1
					
				if event.type == pygame.KEYDOWN:
					if event.key == pygame.K_BACKSPACE:  
						fileName = fileName[:-1]	  
					else:
						fileName += event.unicode
						
			point = int(height/42)
			font = pygame.font.SysFont(fontName, point)
						
			menuWidth = width*0.4
			menuHeight = height*0.5
			
			pygame.draw.rect(screen, backgroundColor, pygame.Rect(width/2-menuWidth/2,height/2-menuHeight/2,menuWidth,menuHeight))	
			pygame.draw.rect(screen, borderColor, pygame.Rect(width/2-menuWidth/2,height/2-menuHeight/2,menuWidth,menuHeight),stroke)
			screen.blit(rawImages[selectedImage],(width/2-image1.get_width()/2,height/2-image1.get_height()/2))
			
			text_width, text_height = font.size("NAME: %s.png"%fileName)
			text_surface = font.render("NAME: %s.png"%fileName, True, textColor)
			screen.blit(text_surface, (width/2-text_width/2,height*0.28))
			
			if saveImageButton.handle(screen, width, height, font, mousePos, clicked):
				saveImageButton.deactivate()
				pygame.image.save(rawImages[selectedImage],'/home/wsurover/rover_pkg/src/rover_pkg/src/image_captures/%s.JPEG'%fileName)
				captureImageMode = 0
			if cancelImageButton.handle(screen, width, height, font, mousePos, clicked):
				cancelImageButton.deactivate()
				captureImageMode = 0
			if selectImageButton.handle(screen, width, height, font, mousePos, clicked):
				selectedImage = selectedImage+1
				if selectedImage > len(rawImages)-1:
					selectedImage = 0
				selectImageButton.deactivate()
				
			
			pygame.display.update()
		
		
		pygame.display.update()
		

	
	pygame.quit()
	
	
	 
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
