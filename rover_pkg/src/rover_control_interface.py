#!/usr/bin/env python3

import rospy
import Jetson.GPIO as GPIO
from rover_pkg.msg import ArmData
from std_msgs.msg import Int32MultiArray
from rover_pkg.msg import RockerBogieData
from rover_pkg.msg import CameraControlData
from rover_pkg.msg import ToolData
from rover_pkg.msg import RoverStateData
from gortmcp import GortMCP23017
import smbus
import time
import rospy

GPIOExpander = GortMCP23017.MCP23017(1,0x20)


INPUT = 0xFF
OUTPUT = 0x00
HIGH = 0xFF
LOW = 0x00
Signal_LED = False

SIG_LED_PIN = 37
RB_LOCK_PIN = 38
ARM_LOCK_PIN = 40

global arm_state 
arm_state = "basic"

motor_pwm_driver = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1];
general_pwm_driver = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1];

# rb_fr, rb_mr, rb_rr, rb_fl,rb_ml, rb_rl
# 6 arm_zm, arm_bm, arm_am, arm_wr, arm_wp, qdcl
# 12 QDC_SIG1, QDC_SIG2, QDC_SIG3, QDC_SIG4, QDC_SIG5, QDC_SIG6, QDC_SIG7
# 19 ctp, ctt, tool_select

pwmConstant = 6.5535

def jetsonSignalDriver(event):

	global motor_driver_pwm
	global Signal_LED
	if (Signal_LED):
		GPIO.output(SIG_LED_PIN,HIGH)
		Signal_LED = False
	else:
		GPIO.output(SIG_LED_PIN,LOW)
		Signal_LED = True

	motor_pwm_driver[12] = 65535*Signal_LED
	motor_pwm_driver[13] = 65535*Signal_LED
	
def roverStateStatus(data):
	global arm_state
	arm_state = data.arm_state

def setAdvancedArmStatus(data):
	global arm_state
	if arm_state == "advanced":
		motor_pwm_driver[6] = int(data.arm_bm*pwmConstant)
		motor_pwm_driver[7] = int(data.arm_am*pwmConstant)
		motor_pwm_driver[8] = int(data.arm_zm*pwmConstant)
		motor_pwm_driver[9] = int(data.arm_wr*pwmConstant)
		motor_pwm_driver[10] = int(data.arm_wp*pwmConstant)
	
def setBasicArmStatus(data):
	global arm_state
	if arm_state == "basic":
		motor_pwm_driver[6] = int(data.arm_bm*pwmConstant)
		motor_pwm_driver[7] = int(data.arm_am*pwmConstant)
		motor_pwm_driver[8] = int(data.arm_zm*pwmConstant)
		motor_pwm_driver[9] = int(data.arm_wr*pwmConstant)
		motor_pwm_driver[10] = int(data.arm_wp*pwmConstant)
	
def setRockerBogieStatus(data):
	motor_pwm_driver[0] = int(data.rb_fr*pwmConstant)
	motor_pwm_driver[1] = int(data.rb_mr*pwmConstant)
	motor_pwm_driver[2] = int(data.rb_rr*pwmConstant)
	motor_pwm_driver[3] = int(data.rb_fl*pwmConstant)
	motor_pwm_driver[4] = int(data.rb_ml*pwmConstant)
	motor_pwm_driver[5] = int(data.rb_rl*pwmConstant)
	
	
def setCameraStatus(data):
	general_pwm_driver[11] = data.ctp
	general_pwm_driver[12] = data.ctt
	
	
def setToolStatus(data):
	pass
	#signals[21] = data.tool_select
	#signals[12] = data.QDC_SIG1
	#signals[13] = data.QDC_SIG2
	#signals[14] = data.QDC_SIG3
	#signals[15] = data.QDC_SIG4
	#signals[16] = data.QDC_SIG5	
	#signals[17] = data.QDC_SIG6	
	#signals[18] = data.QDC_SIG7

def setScienceTableStatus(data):
	pass
	
def setToolStatus(data):
	pass

def updatePWMDrivers(event):	
			
	# publish the new data
	motor_data_out = Int32MultiArray(data = motor_pwm_driver)
	general_data_out = Int32MultiArray(data = general_pwm_driver)	

	MotorPWMPublisher = rospy.Publisher('motor_pwm_driver_stream', Int32MultiArray, queue_size=10)
	MotorPWMPublisher.publish(motor_data_out)
	
		
	GeneralPWMPublisher = rospy.Publisher('general_pwm_driver_stream', Int32MultiArray, queue_size=10)
	GeneralPWMPublisher.publish(general_data_out)	

	
def initialize():	
	rospy.init_node('rover_control_interface_node', anonymous=True)	
	
	rospy.Subscriber('tool_control_stream', ToolData, setToolStatus)
	rospy.Subscriber('rocker_bogie_control_stream', RockerBogieData, setRockerBogieStatus)
	rospy.Subscriber('advanced_arm_control_stream', ArmData, setAdvancedArmStatus)	
	rospy.Subscriber('basic_arm_control_stream', ArmData, setBasicArmStatus)
	rospy.Subscriber('camera_control_stream', CameraControlData, setCameraStatus)
	rospy.Subscriber('rover_state_stream', RoverStateData, roverStateStatus)
	
	rospy.loginfo("Rover Controller node started")

	# update every signal to the pwm drivers
	rospy.Timer(rospy.Duration(0.1), updatePWMDrivers)	
	# update the LED on the PCB for signal
	rospy.Timer(rospy.Duration(0.5), jetsonSignalDriver)
	
	# set pinmodes
	GPIO.setmode(GPIO.BOARD)
	GPIO.setwarnings(False)
	GPIO.setup(SIG_LED_PIN, GPIO.OUT, initial=HIGH)
	GPIO.setup(RB_LOCK_PIN, GPIO.OUT, initial=HIGH) # set rb pwm high
	GPIO.setup(ARM_LOCK_PIN, GPIO.OUT, initial=HIGH) # set arm pwm high

	rospy.spin()

if __name__ == '__main__':
	try:
		initialize()
		if rospy.is_shutdown():
			#disable motors	
			GPIO.output(RB_LOCK_PIN,LOW)
			GPIO.output(ARM_LOCK_PIN,LOW)
		
	except rospy.ROSInterruptException:
		pass
		
		
		
		
		
		
		
		
		

