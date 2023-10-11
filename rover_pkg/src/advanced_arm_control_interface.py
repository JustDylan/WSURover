#!/usr/bin/env python3.6
import rospy
import math
import time

from rover_pkg.msg import UserInput
from rover_pkg.msg import FastAnalogData
from rover_pkg.msg import ArmData
from rover_pkg.msg import RoverStateData

class PIDMotorController:
	# PID controller handles moving a motor to a specified position with PID
	# the PID loop inputs arm positions in terms of degrees
	# the PID loop outputs a pulse length to be sent to a motor controller
	# PID controller includes a value to set the zero value of the motor
	def __init__(self, Kp, Ki, Kd, zeroBalance, maxVel, minError):
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd
		self.error = 0.0
		self.prev_error = 0.0
		self.integral = 0.0
		self.target = 0
		self.maxVel = maxVel
		self.zeroBalance = zeroBalance
		self.minError = minError
		
	def calculate(self, current_value, dt):
		self.error = self.target - current_value # find the current error
		#print(self.error)
		self.integral += self.error*dt # calculate the integral
		derivative = (self.error-self.prev_error)/dt # calculate the derivative
		output = self.Kp*self.error + self.Ki*self.integral + self.Kd*derivative # calculate PID output
		
		# limit the motor speed
		if output > self.maxVel:
			output = self.maxVel
		if output < -1*self.maxVel:
			output = -1*self.maxVel
			
		# add the output to the zero balance
		output = self.zeroBalance + output
		
		return  int(output)
		
	def setTarget(self, newTarget):
		self.target = newTarget
	
	def isSettled(self):
		return abs(self.error)<self.minError

# Kp, Ki, Kd, zeroBalance, maxVel
ZMotor = PIDMotorController(0.4, 0.0, 0.0, 1450,250, 100) # slew motor
BMotor = PIDMotorController(-2.2, 0, 0, 1450,400, 201) # boom motor
AMotor = PIDMotorController(2.2, 0, 0, 1450,400,205) # arm motor
WPMotor = PIDMotorController(0.01, 0, 0, 1450,100,300) # wrist pitch motor

L_0 = 24 # length of the arm segments

boom_a_min = 29312 # min analog value for boom, calibrated at 90*
boom_a_max = 26976
boom_a_calibrated_angle = 90

arm_a_min = 19110 # calibrate at 180* to 270
arm_a_max = 21360
arm_a_calibrated_angle = 270

slew_a_min = 16064
slew_a_max = 23776 # max analog signal for slew, calibrated at 90*
slew_a_calibrated_angle = 90

pitch_a_min = 500
pitch_a_max = 20000

knowsPosition = False
motorUpdateRate = 0.01

boom_analog = -1
arm_analog = -1
wpd_analog = -1
table_analog = -1

target_x = 5
target_y = 0
target_z = 10

global arm_state
arm_state = "basic"

def map(value, from_min, from_max, to_min, to_max):
	proportion = (to_max-to_min)/(from_max-from_min)
	return proportion*(value-from_min) + to_min

def sensorCallback(FastAnalogData):
	# get all arm positions read as analog values
	global boom_analog
	global arm_analog
	global wpd_analog
	global table_analog
	
	boom_analog = FastAnalogData.bd_sig
	arm_analog = FastAnalogData.ad_sig
	wpd_analog = FastAnalogData.wpd_sig
	table_analog = FastAnalogData.table_sig
	global knowsPosition
	knowsPosition = True

def userCallback(user_data):
	global arm_state
	if knowsPosition and arm_state == "advanced":
		scalingFactor = 0.5
		keyboardScale = 0.2
		global target_x
		global target_y
		global target_z
		if 'i' in user_data.keysPressed:
			target_x += scalingFactor*keyboardScale
		if 'k' in user_data.keysPressed:
			target_x -= scalingFactor*keyboardScale
		if 'j' in user_data.keysPressed:
			target_y -= scalingFactor*keyboardScale
		if 'l' in user_data.keysPressed:
			target_y += scalingFactor*keyboardScale
		if 'u' in user_data.keysPressed:
			target_z -= scalingFactor*keyboardScale
		if 'o' in user_data.keysPressed:
			target_z += scalingFactor*keyboardScale
		
		#target_x += scalingFactor*user_data.ljx
		#target_y += scalingFactor*user_data.rjx
		#target_z += scalingFactor*user_data.ljy

		print(target_x,target_y,target_z)
		# update the degree targets based off the user input
		if target_x !=0:
			phi_deg_target = math.atan(target_y/target_x)*(180/math.pi)
		else:
			phi_deg_target = 0
		
			
		
			
		alpha_deg_target = 2*math.asin(math.sqrt((target_x**2)+(target_y**2)+(target_z**2))/(2*L_0))*(180/math.pi)		
		theta_deg_target = (abs(target_z)/target_z)*(math.acos(math.sqrt((target_x**2)+(target_y**2))/math.sqrt((target_x**2)+(target_y**2)+(target_z**2)))*(180/math.pi))+(90-(alpha_deg_target/2))
		
		# correct negative angle values
		if target_x < 0 :
			if target_y<0:
				phi_deg_target = -180 + phi_deg_target 
			else: 
				phi_deg_target = 180 + phi_deg_target
		
		# update the analog targets based on the angles to approach target position		
		theta_deg_encoder = theta_deg_target
		theta_a_target = map(theta_deg_encoder, 0, boom_a_calibrated_angle, boom_a_min, boom_a_max)
		
		alpha_deg_corrected = alpha_deg_target + map(theta_deg_target, 0, 45, 0, 11.7)		
		alpha_a_target =  map(alpha_deg_corrected, 180, arm_a_calibrated_angle, arm_a_min, arm_a_max) # convert analog signal to alpha in deg, angle between boom and arm
		
		phi_a_target = map(phi_deg_target, 0, slew_a_calibrated_angle, slew_a_min, slew_a_max) # convert analog signal to phi, angle of slew
		# set motor analog targets
		ZMotor.setTarget(phi_a_target)
		BMotor.setTarget(theta_a_target)
		AMotor.setTarget(alpha_a_target)
		
	
def updateMotors(event):
	armData = ArmData()
	if knowsPosition:
		armData.arm_zm = ZMotor.calculate(table_analog,motorUpdateRate)
		armData.arm_bm = BMotor.calculate(boom_analog,motorUpdateRate)
		armData.arm_am = AMotor.calculate(arm_analog,motorUpdateRate)
		armData.arm_wp = 1450
		armData.arm_wr = 1450
			
	ArmPosPublisher = rospy.Publisher('advanced_arm_control_stream', ArmData, queue_size=10)
	ArmPosPublisher.publish(armData)
	
def roverStateStatus(data):
	global arm_state
	arm_state = data.arm_state

def init():
	rospy.init_node('basic_arm_control_interface', anonymous=True)
	rospy.Subscriber('fast_sensor_data_stream', FastAnalogData, sensorCallback)
	rospy.Subscriber('user_input_stream', UserInput, userCallback)
	rospy.Subscriber('rover_state_stream', RoverStateData, roverStateStatus)
	
	# update every motor velocity to approach target every 0.1 seconds
	rospy.Timer(rospy.Duration(motorUpdateRate), updateMotors)

	
	rospy.loginfo("Advanced Arm Control node started")
	rospy.spin()
	
	
		

if __name__ == '__main__':
	try:
		init()
		print("hello")
			
	except rospy.ROSInterruptException:
		pass
		
		
		
		
