#!/usr/bin/env python3

import rospy
import math
from std_msgs import int
from rover_pkg.msg import UserInput
from rover_pkg.msg import TEndCord
from rover_pkg.msg import ArmData

#pre-allocating coords and previous coords
global xprev
global yprev
global zprev
global x
global y
global z
global xt
global yt
global zt
global theta
global alpha
global epsilon
global ttheta
global talpha
global tepsilon
global turnstate

#setting rmax, Rmax, x and y maxes
global rmax
Rmax = int(44)
xmax = int(10)
ymaxp = int(5)
ymaxn = int(20)
spedres = int(0.5)
stationary = int(1500)

zm = int(4)

def approach_check():
    xp = xt
    yp = yt
    zp = zt
    if x = xt and y = yt and z = zt
        targetcheck = 1
    else
        targetcheck = 0    
    xs = xp**2
    lim = math.sqrt((xs+zs))
    xprev = x - 0.1
    zprev = z - 0.1
    if y < ymaxp and z <= 0
        yp = yprev
        print("colliding with front of rover")
    if y > ymaxn and z < 0
        yp = yprev
        print("colliding with rear of rover")
    if lim <= rmax
        xp = xprev
        zp = zprev
        print("inside rmax limit")
    if abs(x) <= xmax and zm < 0
        zp = int(2)
        print("colliding with side of rover")
    if z <= int(2)
        if y <= ymaxp
            yp = yprev
        if yt >= ymaxn
            yp = yprev
        if abs(xt) <= xmax
            xt = xprev
        print("colliding with rover")
        
    return(xp,yp,zp,targetcheck)

def angles(xp,zp):
    xs = xp**2
    zs = zp**2
    mag = math.sqrt((xs+zs))
    alpha = 2*((180/3.14)*math.asin(mag/48))
    if zp > 0
        thedev = 48*math.sin(alpha/2)
        theta = ((180/3.14)*math.acos(mag/thedev)) + (90 - (alpha/2))
    else 
        thedev = (-1)*((180/3.14)*math.atan(abs(zp/xp)))
        theta = thedev + (90 - (alpha/2))      
    
#update tracking co-ordinates
def tracking(SensorData):
    t = SensorData.arm_bm
    a = SensorData.arm_am 
    ttheta = CONVERSION PLES
    talpha = CONVERSION PLES
	gam = 2*48*math.sin(talpha/2)
	tan = math.tan(ttheta-90+(talpha/2))
    pz = sqrt(gam/(1+(tan**2)))
    px = 
   
    TEndCord = TEndCord()
    TEndCord.x = px
    TEndCord.z = pz
    CoordPublisher = rospy.Publisher('tend_coord', TEndCord, queue_size=10)
	CoordPublisher.publish(TEndCord)
        
def getter():
    etheta = ttheta - theta
    ealpha = talpha - alpha

    arm_zm = int(1490+(turnstate*100))
    arm_bm = int(1490+((etheta/abs(etheta)*40)+((etheta/ttheta)*80))
    arm_am = int(1490+((ealpha/abs(ealpha))*40)+((ealpha/talpha)*80))
         
    ArmData = ArmData()
    ArmData.arm_zm = arm_zm
    ArmData.arm_bm = arm_bm
    ArmData.arm_am = arm_am
    ArmPosPublisher = rospy.Publisher('arm_control_stream', ArmData, queue_size=10)
	ArmPosPublisher.publish(ArmData)

#update coordinates
def scheduler():
    TargetList = approach_check()
    if TargetList[3] = 0 
        angles(TargetList[0],TargetList[1])
        getter()        

#use user input commands to change coordinates
def controller(UserInput):
    turnstate = UserInput.t
    xs = (UserInput.x)**2
    zs = (UserInput.z)**2
    lim2 = math.sqrt((xs+zs))
    if zt > 0 and zt < zm
        rmax = int(6)
    else
        rmax = int(1)
    if abs(x) <= xmax and zm < 0
        if x < 0
            xt = xmax - 0.5
        else 
            xt = xmax + 0.5
        print("colliding with side of rover")
    if lim2 <= rmax
        xt = xprev
        zt = zprev
        print("inside rmax limit")

        
        
#initializes the the function chain
def init():
#if statement sets all coordinates to zero on startup

    rospy.Subscriber('sensor_data', SensorData, tracking)
    rospy.Subscriber('user_input_stream', UserInput, controller)
    rospy.Timer(rospy.Duration(0.01), scheduler)
    rospy.Timer(rospy.Duration(0.01), getter)
    rospy.init_node('arm_control', anonymous=True)	

	rospy.spin()

if __name__ == '__main__':
	try:
		init()
	except rospy.ROSInterruptException:
		pass
