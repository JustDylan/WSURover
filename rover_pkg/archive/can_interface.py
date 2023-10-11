#!/usr/bin/env python
import rospy
import can
import can.interfaces.slcan
from rover_pkg.msg import RoverData

global rb_fr
global rb_mr
global rb_rr
global rb_fl
global rb_ml
global rb_rl

bus = can.interface.Bus(bustype='slcan', channel='/dev/ttyS0', bitrate=500000)

class TalonSRX:
	def __init__(canID):
		self.DemandH = 0
		self.DemandM = 0
		self.DemandL = 0
		self.canID = canID
		
	def set_demand(demand):
		self.Demand = int(demand)
		self.DemandH = (self.Demand & 0x00FF0000) >> 16
		self.DemandM = (self.Demand & 0x0000FF00) >> 8
		self.DemandL = self.Demand & 0x000000FF

	def getFrame():
		return bytearray([0,0,self.DemandH,self.DemandM,self.DemandL,0,0,0])

def callback(data):
	rb_fr = data.rb_fr
	rb_mr = data.rb_mr
	rb_rr = data.rb_rr
	rb_fl = data.rb_fl
	rb_ml = data.rb_ml
	rb_rl = data.rb_rl
	rospy.loginfo("Drivetrain motor data updated: %f", rb_fr)

	msg = can.Message(arbitration_id=0x00, data=[0,0,0,1,225,0,0,0])
	try:
		bus.send(msg)
		rospy.loginfo("msg sent")
	except can.CanError:
		rospy.loginfo("msg failed to send")

def listener():
	rospy.init_node("can_interface", anonymous = True)
	rospy.Subscriber('rover_control_stream', RoverData, callback)
	rate = rospy.Rate(10)
	rospy.spin()
	rate.sleep()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass

