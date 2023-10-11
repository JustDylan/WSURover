#!/usr/bin/env python
import rospy
from rover_pkg.msg import RockerBogieData
from rover_pkg.msg import UserInput


rb_fr_inv = -1
rb_mr_inv = -1
rb_rr_inv = 1
rb_fl_inv = 1
rb_ml_inv = -1
rb_rl_inv = 1

motorPWMzero = 1450 # PWM pulse width associated with zero velocity
motorMinPWMchange = 50
motorMaxPWMchange = 500
maxChange = 2

global motorVels
global motorTargetVels
			#rb_fr, rb_mr, rb_rr, rb_fl, rb_ml, rb_rl
motorVels = [motorPWMzero,motorPWMzero,motorPWMzero,motorPWMzero,motorPWMzero,motorPWMzero] # PWM values associated with each motor
motorTargetVels = [motorPWMzero,motorPWMzero,motorPWMzero,motorPWMzero,motorPWMzero,motorPWMzero]

def pwmMap(value):
	#input value from -1 to 1 and map to pwm range
	return motorPWMzero + value*motorMaxPWMchange	

def basic_controller(user_data):	
	motorTargetVels[0] = pwmMap(user_data.ljx*rb_fr_inv)
	motorTargetVels[1] = pwmMap(user_data.ljx*rb_rr_inv)
	motorTargetVels[3] = pwmMap(user_data.rjx*rb_fl_inv)
	motorTargetVels[4] = pwmMap(user_data.rjx*rb_ml_inv)
	motorTargetVels[5] = pwmMap(user_data.rjx*rb_rl_inv)	

def keyboard_controller(user_data):
	keys_pressed = user_data.keysPressed
	# use keys_pressed data to update rover data
	speed = 0.8
	correction = 1.414
	if 'w' in keys_pressed:
		motorTargetVels[0] = pwmMap(speed*rb_fr_inv)
		motorTargetVels[1] = pwmMap(speed*rb_mr_inv)
		motorTargetVels[2] = pwmMap(speed*rb_rr_inv)
		motorTargetVels[3] = pwmMap(speed*rb_fl_inv)
		motorTargetVels[4] = pwmMap(speed*rb_ml_inv)
		motorTargetVels[5] = pwmMap(speed*rb_rl_inv)
	if 's' in keys_pressed:
		motorTargetVels[0] = pwmMap(-speed*rb_fr_inv)
		motorTargetVels[1] = pwmMap(-speed*rb_mr_inv)
		motorTargetVels[2] = pwmMap(-speed*rb_rr_inv)
		motorTargetVels[3] = pwmMap(-speed*rb_fl_inv)
		motorTargetVels[4] = pwmMap(-speed*rb_ml_inv)
		motorTargetVels[5] = pwmMap(-speed*rb_rl_inv)
	if 'a' in keys_pressed:
		motorTargetVels[0] = pwmMap(speed*rb_fr_inv)
		motorTargetVels[1] = pwmMap(speed*rb_mr_inv)
		motorTargetVels[2] = pwmMap(-speed*rb_rr_inv)
		motorTargetVels[3] = pwmMap(-speed*rb_fl_inv)
		motorTargetVels[4] = pwmMap(speed*rb_ml_inv)
		motorTargetVels[5] = pwmMap(-speed*rb_rl_inv)
	if 'd' in keys_pressed:
		motorTargetVels[0] = pwmMap(-speed*rb_fr_inv)
		motorTargetVels[1] = pwmMap(-speed*rb_mr_inv/correction)
		motorTargetVels[2] = pwmMap(speed*rb_rr_inv)
		motorTargetVels[3] = pwmMap(speed*rb_fl_inv)
		motorTargetVels[4] = pwmMap(-speed*rb_ml_inv/correction)
		motorTargetVels[5] = pwmMap(speed*rb_rl_inv)	
	if len(keys_pressed)==0:
		motorTargetVels[0] = pwmMap(0)
		motorTargetVels[1] = pwmMap(0)
		motorTargetVels[2] = pwmMap(0)
		motorTargetVels[3] = pwmMap(0)
		motorTargetVels[4] = pwmMap(0)
		motorTargetVels[5] = pwmMap(0)
		

def inputCallback(user_data):
	if (user_data.controlMode == "basic_controller"):
		basic_controller(user_data)
		
	if (user_data.controlMode == "keyboard_controller"):
		keyboard_controller(user_data)

def updateMotorData(event):	
	
	# go through each motor and update the current velocity to approach target
	for i in range(len(motorVels)):
		targetVel = motorTargetVels[i]
		currentVel = motorVels[i]		
		
		# if current velocity is not target, do something about it
		if (currentVel != targetVel):			
			
			difference = targetVel - currentVel
			
			# new velocity = current velocity + direction*speed
			newVel = currentVel + min(maxChange,abs(difference))* (abs(difference)/difference)
			
			# if within dead zone
			if (newVel > motorPWMzero-motorMinPWMchange and newVel < motorPWMzero+ motorMinPWMchange):				
				# if going from pos to neg
				if (targetVel < motorPWMzero-motorMinPWMchange):
					newVel = motorPWMzero-motorMinPWMchange
				# if going from neg to pos
				if (targetVel > motorPWMzero+motorMinPWMchange):
					newVel = motorPWMzero+motorMinPWMchange
				
			# write new velocity to array			
			motorVels[i] = newVel
			
	# publish the new data
	data_out = RockerBogieData()	
	data_out.rb_fr = int(motorVels[0])
	data_out.rb_mr = int(motorVels[1])
	data_out.rb_rr = int(motorVels[2])
	data_out.rb_fl = int(motorVels[3])
	data_out.rb_ml = int(motorVels[4])
	data_out.rb_rl = int(motorVels[5])	

	RockerBogieDataPublisher = rospy.Publisher('rocker_bogie_control_stream', RockerBogieData, queue_size=10)
	RockerBogieDataPublisher.publish(data_out)	

	
def initialize():	
	rospy.init_node('rocker_bogie__control_interface_node', anonymous=True)	
	rospy.Subscriber('user_input_stream', UserInput, inputCallback)
	rospy.loginfo("Rocker Bogie Controller node started")

	# update every motor velocity to approach target every 0.1 seconds
	rospy.Timer(rospy.Duration(0.01), updateMotorData)
	rospy.spin()

if __name__ == '__main__':
	try:
		initialize()
	except rospy.ROSInterruptException:
		pass
		
		
		
		
		
		
		
		
		

