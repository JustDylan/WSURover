#!/usr/bin/env python3.6
import rospy
import math
import time

from rover_pkg.msg import UserInput
from rover_pkg.msg import FastAnalogData
from rover_pkg.msg import ArmData

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
ZMotor = PIDMotorController(0.5, 0.0, 0.0, 1450,250, 100) # slew motor
BMotor = PIDMotorController(-2.2, 0, 0, 1450,400, 201) # boom motor
AMotor = PIDMotorController(2.2, 0, 0, 1450,400,205) # arm motor
WPMotor = PIDMotorController(0.01, 0, 0, 1450,100,300) # wrist pitch motor

L_0 = 24 # length of the arm segments

boom_a_min = 26304 # min analog value for boom, calibrated at 90*
boom_a_max = 24096
boom_a_calibrated_angle = 90

arm_a_min = 19168 # calibrate at 180* to 270
arm_a_max = 21360
arm_a_calibrated_angle = 270

slew_a_min = 14832
slew_a_max = 21100 # max analog signal for slew, calibrated at 60*
slew_a_calibrated_angle = 90

pitch_a_min = 500
pitch_a_max = 20000

knowsPosition = False
motorUpdateRate = 0.01


phi_a_target = -10000
alpha_a_target = -10000
theta_a_target = -10000

boom_analog = -1
arm_analog = -1
wpd_analog = -1
table_analog = -1

home = [1,0,10]

super_lit_demo_targets_frfr_no_cap_lets_impress_everyone = [home, [13,-31,22], [20,-31,22], [32,25,20] , [34,27,3], home]

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
	global printout
	printout = [boom_analog,arm_analog,table_analog]
	print(printout)
	
	global knowsPosition
	if not knowsPosition:
		print("set current pos to target")
		theta_a_target = boom_analog
		alpha_a_target = arm_analog
		phi_a_target = table_analog
		ZMotor.setTarget(phi_a_target)
		BMotor.setTarget(theta_a_target)
		AMotor.setTarget(alpha_a_target)
	
	knowsPosition = True
	
	
	

index_of_super_lit_demo_targets_frfr_no_cap_lets_impress_everyone = 0
begin = False

def userCallback(user_data):	
	keys_pressed = user_data.keysPressed	
	global begin
	if 'x' in keys_pressed and not begin: # execute the super_lit_demo_targets_frfr_no_cap_lets_impress_everyone
		begin = True
		print("Cycle Start");
		
def updateScheduler(event):
	new_target = False
	global index_of_super_lit_demo_targets_frfr_no_cap_lets_impress_everyone
	if begin:
		print(index_of_super_lit_demo_targets_frfr_no_cap_lets_impress_everyone)
		target_x = super_lit_demo_targets_frfr_no_cap_lets_impress_everyone[index_of_super_lit_demo_targets_frfr_no_cap_lets_impress_everyone][0]
		target_y = super_lit_demo_targets_frfr_no_cap_lets_impress_everyone[index_of_super_lit_demo_targets_frfr_no_cap_lets_impress_everyone][1]
		target_z = super_lit_demo_targets_frfr_no_cap_lets_impress_everyone[index_of_super_lit_demo_targets_frfr_no_cap_lets_impress_everyone][2]
		print(ZMotor.isSettled());
		
		if (ZMotor.isSettled() and AMotor.isSettled() and BMotor.isSettled()): # motors error < somehting
			index_of_super_lit_demo_targets_frfr_no_cap_lets_impress_everyone += 1
			new_target = True			
			print("has new target")
	
	if new_target:
		global theta_a_target 
		global alpha_a_target
		global phi_a_target	
		# update the degree targets based off the user input
		new_target = False
		if target_z!=0:
			phi_deg_target = math.atan(target_y/target_x)*(180/math.pi)
		else:
			phi_deg_target = 0
		alpha_deg_target = 2*math.asin(math.sqrt((target_x**2)+(target_y**2)+(target_z**2))/(2*L_0))*(180/math.pi)		
		theta_deg_target = (math.acos(math.sqrt((target_x**2)+(target_y**2))/math.sqrt((target_x**2)+(target_y**2)+(target_z**2)))*(180/math.pi)+90-(alpha_deg_target/2))
		
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
			
	ArmPosPublisher = rospy.Publisher('arm_control_stream', ArmData, queue_size=10)
	ArmPosPublisher.publish(armData)
	

def init():
	rospy.init_node('basic_arm_control_interface', anonymous=True)
	rospy.Subscriber('fast_sensor_data_stream', FastAnalogData, sensorCallback)
	rospy.Subscriber('user_input_stream', UserInput, userCallback)
	
	# update every motor velocity to approach target every 0.1 seconds
	rospy.Timer(rospy.Duration(motorUpdateRate), updateMotors)
	
	# update scheduler every 1 seconds
	rospy.Timer(rospy.Duration(3), updateScheduler)
	
	#rospy.Timer(rospy.Duration(1), updateActiveCoordinates)
	
	rospy.loginfo("Advanced Arm Control node started")
	rospy.spin()
	
	
		

if __name__ == '__main__':
	try:
		init()
		print("hello")
			
	except rospy.ROSInterruptException:
		pass
		
		
		
		
