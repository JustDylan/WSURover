4#!/usr/bin/env python3

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

#setting rmax, Rmax, x and y maxes
global rmax
Rmax = int(44)
xmax = int(10)
ymaxp = int(5)
ymaxn = int(20)
spedres = int(0.5)
stationary = int(1500)
startx = int(0)
starty = int(10)
startz = int(10)
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
    ys = yp**2
    zs = zp**2
    lim2 = math.sqrt((xs+ys))
    lim3 = math.sqrt((xs+ys+zs))
    xprev = x - 0.1
    yprev = y - 0.1
    zprev = z - 0.1
    if y < ymaxp and z <= 0
        yp = yprev
        print("colliding with front of rover")
    if y > ymaxn and z < 0
        yp = yprev
        print("colliding with rear of rover")
    
    if lim2 <= rmax
        xp = xprev
        yp = yprev
        print("inside rmax limit")
    if lim3 >= Rmax
        xp = xprev
        yp = yprev
        zp = zprev
        print("outside Rmax limit")
    if abs(x) <= xmax and zm < 0
        zp = int(2)
        print("colliding with side of rover")
    if y < ymaxp and z < 0
        zp = int(2)
        print("colliding with front of rover")
    if y > ymaxn and z < 0
        zp = int(2)
        print("colliding with rear of rover")
    if z <= int(2)
        if y <= ymaxp
            yp = yprev
        if yt >= ymaxn
            yp = yprev
        if abs(xt) <= xmax
            xt = xprev
        print("colliding with rover")
        
    return(xp,yp,zp,targetcheck)

def angles(xp,yp,zp):
    xs = xp**2
    ys = yp**2
    zs = zp**2
    mag = math.sqrt((xs+ys+zs))
    tmag = math.sqrt((ys+zs))
    alpha = 2*((180/3.14)*math.asin(tmag/48))
    if xp < 0
        epsilon = (-1)*(180/3.14)*math.atan(abs(xp/yp))
        if yp < 0 
            epsilon = epsilon - 90
    else 
        epsilon = (180/3.14)*math.atan(abs(xp/yp))
        if yp < 0 
            epsilon = epsilon + 90
    if yp > 0
        thedev = 48*math.sin(alpha/2)
        theta = ((180/3.14)*math.acos(tmag/thedev)) + (90 - (alpha/2))
    else 
        thedev = 180 - ((180/3.14)*math.atan(abs(zp/yp)))
        theta = thedev + (90 - (alpha/2))      
    
#update tracking co-ordinates
def tracking(SensorData):
    e = SensorData.arm_zm
    t = SensorData.arm_bm
    a = SensorData.arm_am
    tepsilon = CONVERSION PLES 
    ttheta = CONVERSION PLES
    talpha = CONVERSION PLES
    omega = 2*48*math.sin(talpha/2)
    sig = ((math.cos(ttheta+(talpha/2)-90))*omega)**2
    py = sqrt(sig/((math.tan(tepsilon)**2)+1)
    px = py*math.tan(epsilon)
    pz = sqrt((omega**2)-(py**2)-(px**2))
   
    TEndCord = TEndCord()
    TEndCord.x = px
    TEndCord.y = py
    TEndCord.z = pz
    CoordPublisher = rospy.Publisher('tend_coord', TEndCord, queue_size=10)
	CoordPublisher.publish(TEndCord)
        
def getter():
    eepsilon = tepsilon - epsilon
    etheta = ttheta - theta
    ealpha = talpha - alpha
    
    arm_zm = int(1500+((eepsilon/abs(eepsilon))*50)+((eepsilon/tepsilon)*450))
    arm_bm = int(1500+((etheta/abs(etheta)*50)+((etheta/ttheta)*450))
    arm_am = int(1500+((ealpha/abs(ealpha))*50)+((ealpha/talpha)*450))
         
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
        angles(TargetList[0],TargetList[1],TargetList[2])
        getter()        

#use user input commands to change coordinates
def controller(UserInput):
    xs = (UserInput.x)**2
    ys = (UserInput.y)**2
    zs = (UserInput.z)**2
    lim2 = math.sqrt((xs+ys))
    lim3 = math.sqrt((xs+ys+zs))
    if zt > 0 and zt < zm
        rmax = int(6)
    else
        rmax = int(1)
    if y < ymaxp and z <= 0
        yt = tmaxp + 0.5
        print("colliding with front of rover")
    if y > ymaxn and z <= 0
        yt = tmaxn - 0.5
        print("colliding with rear of rover")
    if abs(x) <= xmax and zm < 0
        if x < 0
            xt = xmax - 0.5
        else 
            xt = xmax + 0.5
        print("colliding with side of rover")
    if lim2 <= rmax
        xt = xprev
        yt = yprev
        print("inside rmax limit")
    if lim3 >= Rmax
        xt = xprev
        yt = yprev
        zt = zprev
        print("outside Rmax limit")
        
        
#initializes the the function chain
def init():
#if statement sets all coordinates to zero on startup
    if len(x) == 0:
        startup()
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