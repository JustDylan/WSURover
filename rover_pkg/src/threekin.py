import rospy
import math
from std_msgs import int
from rover_pkg.msg import UserInput
from rover_pkg.msg import ArmData

global boom_pos
global arm_pos
global slew_pos

global cur_boom_pos
global cur_arm_pos
global cur_slew_pos

global boom
global arm
global slew

global alpha
global theta
global phi

global boom
global arm
global slew

x = 5
y = 5
z = 0
stepsize = 0.25
delay = 0.05
L_0 = 24

maxpota = 200
minpota = 300
maxpotb = 200
minpotb = 300
maxpots = 200
maxpots = 300

def getter():
    alpha = 2*math.asin(math.sqrt((x**2)+(y**2))/(2*L_0))
    if x != 0 :
    	btheta = math.atan(y/x)
    else :
    	btheta = math.pi/2
    eps = (2*math.pi)-alpha
    theta = eps + btheta
    
    err_theta = theta - curposb
    err_alpha = alpha - curposa
    
    if err_theta < -5 or err_theta > 5:
    	bm = int(1490+((etheta/abs(etheta)*40)+((etheta/ttheta)*80))
    else :
    	bm = 1490    	
    if err_alpha < -5 or err_alpha > 5:
    	am = int(1490+((ealpha/abs(ealpha))*40)+((ealpha/talpha)*80))
    else :
    	am = 1490
    
    armdata = ArmData()
    armdata.arm_bm = bm
    armdata.arm_am = am
    ArmPosPublisher = rospy.Publisher('arm_control_stream', ArmData, queue_size=10)
    ArmPosPublisher.publish(armdata)

def tracking(SensorData):
    arm_pos = SensorData.arm_am
    boom_pos = SensorData.arm_bm
    cur_posa = arm_pos*((maxpota-minpota)/maxpota)*(90)
    cur_posb = boom_pos*((maxpotb-minpotb)/maxpotb)*(90)


def controller(user_data):
    KEYLIST = user_data.keysPressed
    if 'i' in KEYLIST:
        x = x + stepsize
        #time.sleep(delay)
    if 'k' in KEYLIST:
        x = x - stepsize
        #time.sleep(delay)
    if 'y' in KEYLIST:
        y = y + stepsize
        #time.sleep(delay)
    if 'h' in KEYLIST:
        y = y - stepsize
        #time.sleep(delay)
    if x > 39 :
        x = 39
    if y > 39 :
        y = 39
    if x < 6 :
        x = 6
    if y < -9 :
        y = -9

#initializes the the function chain
def init():
    rospy.Subscriber('sensor_data', SensorData, tracking)
    rospy.Subscriber('user_input_stream', UserInput, controller)
    rospy.Timer(rospy.Duration(0.01), getter)
    rospy.spin()

if __name__ == '__main__':
	try:
        rospy.init_node('arm_control', anonymous=True)
        
        init()
	except rospy.ROSInterruptException:
		pass
