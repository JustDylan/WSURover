#!/usr/bin/env python
import rospy
from rover_pkg.msg import RoverData
from std_msgs.msg import Int32MultiArray

qdclLocked = 4000 # PWM pulse width associated with locked quick disconnect
qdclOpen = 2600 # PWM pulse width associated with open quick disconnect
motorPWMzero = 1500 # PWM pulse width associated with zero velocity
			#rb_fr, rb_mr, rb_rr, rb_fl, rb_ml, rb_rl
motorVels = [motorPWMzero,motorPWMzero,motorPWMzero,motorPWMzero,motorPWMzero,motorPWMzero] # PWM values associated with each motor
motorTargetVels = [motorPWMzero,motorPWMzero,motorPWMzero,motorPWMzero,motorPWMzero,motorPWMzero]
motorMinPWMchange = 50
motorMaxPWMchange = 500

def pwmMap(value):
	#input value from -1 to 1 and map to pwm range
	return motorPWMzero + value*motorMaxPWMchange
	
def PCAMap(value):
		
	stupid_number =6800
	# inpout pulse length (ranges from 1000us-2000us) and map to PCA9685 duty (0-65536)
	return stupid_number + (value-1000)*(2*stupid_number-stupid_number)/(2000-1000);

def updatePWMstream(data):
	# update quick disconnect servo
	qdclValue = qdclOpen
	if (data.qdcl):
		qdclValue = qdclLocked
	
	# set new targets for PWM motors
	
	global motorTargetVels
	motorTargetVels = [pwmMap(data.rb_fr),pwmMap(data.rb_mr),pwmMap(data.rb_rr),pwmMap(data.rb_fl),pwmMap(data.rb_ml),pwmMap(data.rb_rl)]
	
	
	# prep PWM data
	pwm_data_out = Int32MultiArray()
	# fill PWM data with current motor data
	pwm_data_out.data = [PCAMap(motorVels[0]), PCAMap(motorVels[1]), PCAMap(motorVels[2]), PCAMap(motorVels[3]), PCAMap(motorVels[4]), PCAMap(motorVels[5]), 
						 qdclValue,-1,-1,-1,-1,-1,-1,-1,-1,-1]
		
	RoverDataPublisher = rospy.Publisher('PWM_stream', Int32MultiArray, queue_size=10)
	RoverDataPublisher.publish(pwm_data_out)

def updateMotorData(event):	
	
	# go through each motor and update the current velocity to approach target
	for i in range(len(motorVels)):
		targetVel = motorTargetVels[i]
		currentVel = motorVels[i]
		
		
		# if current velocity is not target, do something about it
		if (currentVel != targetVel):
			
			maxChange = 50
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
			global motorVels		
			motorVels[i] = newVel

def listener():
	rospy.init_node("PWM_interface", anonymous = True)	
	rospy.loginfo("PWM interface node started")
	
	# subscribe to PWM stream
	rospy.Subscriber('rover_control_stream', RoverData, updatePWMstream)
	
	# update every motor velocity to approach target every 0.1 seconds
	rospy.Timer(rospy.Duration(0.1), updateMotorData)
	
	rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass

