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

from rover_pkg.msg import RoverStateData

width = 1900
height = 1000
backgroundColor = (20,20,20)
primaryColor = (255,0,0)
primaryColorDark = (153,0,0)
primaryColorLight = (225,51,51)
borderColor = (150,150,150)
textColor = (200,200,200)
fontName = "Times New Roman"
messageList = ["RB: MOTOR TEMP APPROACHING MAX","ARM:POSITION REACHED", "ARM: QDC UNLOCKED"]

captureImageMode = 0

imageRatio = 16/9
capture_path = "/home/wsurover/rover_pkg/src/rover_pkg/src/image_captures/"

pygame.init()

image1 = pygame.image.load("/home/wsurover/rover_pkg/src/rover_pkg/src/gui_recources/no_file_loaded.jpeg")
image1raw = pygame.image.load("/home/wsurover/rover_pkg/src/rover_pkg/src/gui_recources/no_file_loaded.jpeg")

image2 = pygame.image.load("/home/wsurover/rover_pkg/src/rover_pkg/src/gui_recources/filler.jpeg")
image2raw = pygame.image.load("/home/wsurover/rover_pkg/src/rover_pkg/src/gui_recources/filler.jpeg")

rawImages = [image1raw,image2raw]


def drawThrottleBar(screen, font, width, height, x, y, motor, vel):
	maxVel = 20
	throttle = abs(float(vel)/20)
	thickness = int(height/5)
	pygame.draw.rect(screen, borderColor, pygame.Rect(x,y,width,height), thickness)
	gapThickness = thickness*1.5
	if vel>0:	
		pygame.draw.rect(screen, (0,255,0), pygame.Rect(x+width/2, y+gapThickness, int(throttle*width/2), height-2*gapThickness))
	else:		
		pygame.draw.rect(screen, (255,0,0), pygame.Rect(x+width/2-int(throttle*width/2), y+gapThickness, int(throttle*width/2), height-2*gapThickness))
	
	text_surface = font.render("%s %s rad/s"%(motor, vel), True, textColor)
	screen.blit(text_surface, (x+width+thickness,y+thickness/2))
	
def drawFillBar(screen, font, width, height, x, y, label, val, maxVal, unit, color):
	throttle = float(val)/maxVal
	thickness = int(height/5)
	pygame.draw.rect(screen, borderColor, pygame.Rect(x,y,width,height), thickness)	
	
	gapThickness = thickness*1.5
	pygame.draw.rect(screen, color, pygame.Rect(x+gapThickness, y+gapThickness, int(throttle*(width-2*gapThickness)), height-2*gapThickness))
	
	text_width, text_height = font.size(label)
	text_surface = font.render(label, True, textColor)
	screen.blit(text_surface, (x+thickness,y-text_height))	
	
	text_surface = font.render("%s %s "%(val, unit), True, textColor)
	screen.blit(text_surface, (x+width+thickness,y+thickness/2))
	
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
	
	armStateButton = Button(0.7, 0.2, 0.15, 0.05, "Inverse K Control")
	
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
				
		# MESSAGE BOARD
		pygame.draw.rect(screen, borderColor, pygame.Rect(xalignl, height*0.04, width*0.28, height/7),stroke)
		spacing = height*0.022
		for i in range(len(messageList)):
			text_surface = font.render(messageList[i], True, (255,0,0))
			screen.blit(text_surface, (xalignl+width/100,height*0.05+spacing*i))
		
		# ROCKER BOGIE THROTTLE BARS
		barHeight = height/33
		barWidth = width/7
		spacing = height/25
		yalign = height*0.20
		
		rbr_temp = 42.8
		rbl_temp = 43.12
		drawThrottleBar(screen,font,barWidth,barHeight,xalignl,yalign+0*spacing,"RB_FR",-10)
		drawThrottleBar(screen,font,barWidth,barHeight,xalignl,yalign+1*spacing,"RB_MR",-11)
		text_surface = font.render("%s C"%rbr_temp, True, textColor)
		screen.blit(text_surface, (width*0.27,yalign+1*spacing))
		drawThrottleBar(screen,font,barWidth,barHeight,xalignl,yalign+2*spacing,"RB_RR",-9.5)
		drawThrottleBar(screen,font,barWidth,barHeight,xalignl,yalign+3*spacing,"RB_FL",15.2)
		drawThrottleBar(screen,font,barWidth,barHeight,xalignl,yalign+4*spacing,"RB_ML",16.1)
		text_surface = font.render("%s C"%rbl_temp, True, textColor)
		screen.blit(text_surface, (width*0.27,yalign+4*spacing))
		drawThrottleBar(screen,font,barWidth,barHeight,xalignl,yalign+5*spacing,"RB_RL",15.7)
		drawFillBar(screen, font, width/4, barHeight, xalignl, height*0.47, "THROTTLE", 35.4, 100, "%", (0,255,0))	
			
		# SCIENCE STUFF	
		barWidth = width/7
		drawFillBar(screen, font, barWidth, barHeight, xalignl, height/5*3, "BENEDICT SOL'N", 2.51, 3, "mL", (0,102,204))	
		drawFillBar(screen, font, barWidth, barHeight, xalignl, height/5*3+height/16, "BIURET REAGENT", 1.2, 3, "mL", (102,0,204))
		pygame.draw.rect(screen, borderColor, pygame.Rect(xalignl, height*3/4, width*0.28, height*0.15),5)

		# BOTTOM RIGHT DATA
		xalign = width*0.75
		yalign = height*0.8
		spacing = height*0.025
		boxWidth = width*0.2
		
		qdc_status = "LOCKED"
		voltage = 12.33
		current = 1.2
		ping = 12.2
		pressure = 16.3
		RSSI = 10.2		
		
		drawTextBox(screen, font, boxWidth, xalign, yalign+spacing*0, "QUICK DISCONNECT:", "%s"%qdc_status)
		drawTextBox(screen, font, boxWidth, xalign, yalign+spacing*1, "BATTERY VOLTAGE:", "%s V"%voltage)
		drawTextBox(screen, font, boxWidth, xalign, yalign+spacing*2, "CURRENT DRAW:", "%s A"%current)
		drawTextBox(screen, font, boxWidth, xalign, yalign+spacing*3, "PING:", "%s"%ping)
		drawTextBox(screen, font, boxWidth, xalign, yalign+spacing*4, "AMBIENT PRESSURE:", "%s kPa"%pressure)
		drawTextBox(screen, font, boxWidth, xalign, yalign+spacing*5, "RSSI:", "%s"%RSSI)
		
		# Rover State Button
		rover_state_data = RoverStateData()
		rover_state_data.arm_state = "basic"
		
		if armStateButton.handle(screen, width, height, font, mousePos, clicked):
				#armStateButton.deactivate()				
				rover_state_data.arm_state = "advanced"
		
		
		
		RoverStatePublisher = rospy.Publisher('rover_state_stream', RoverStateData, queue_size=1)
		RoverStatePublisher.publish(rover_state_data)
		
		selectedImage = 0
		fileName = '123'
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
	
	
	 
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
