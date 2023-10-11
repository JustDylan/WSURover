#!/usr/bin/env python3.6

import rospy
from kivymd.app import MDApp
from kivy.lang import Builder
from std_msgs.msg import Bool
from kivy.core.window import Window
from kivy.config import Config

class RoverControlApp(MDApp):
	def __init__(self,**kwargs):
		super().__init__(**kwargs)
		Window.size = (1600, 800)
		#Config.set('graphics', 'resizable', False)
		self.theme_cls.theme_style = "Dark"
		self.theme_cls.primary_palette = "Red"
		self.theme_cls.accent_palette = "Gray"		
		self.screen = Builder.load_file('/home/ds332a/rover_code/src/rover_pkg/gui_recources/rover_gui.kv')
		
	def build(self):		
		return self.screen
		
	def my_function(self, *args):
		print("button Pressed")
		self.screen.ids.messages_label.text="button pressed........................."

if __name__ == '__main__':

	pub=rospy.Publisher('/button',Bool,queue_size=1)
	rospy.init_node("gui_node", anonymous=True)
	
	RoverControlApp().run()
	
	
	
