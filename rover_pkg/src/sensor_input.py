#!/usr/bin/env python3

import time
import board
import busio
import rospy
import Jetson.GPIO as GPIO
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

from rover_pkg.msg import FastAnalogData
from rover_pkg.msg import SlowAnalogData

slow_i2c = busio.I2C(board.SCL_1, board.SDA_1)
fast_i2c = busio.I2C(board.SCL, board.SDA)

slow_ADC = ADS.ADS1015(slow_i2c)
fast_ADC = ADS.ADS1015(fast_i2c)

fast_read_p0 = AnalogIn(fast_ADC, ADS.P0)
fast_read_p1 = AnalogIn(fast_ADC, ADS.P1)
fast_read_p2 = AnalogIn(fast_ADC, ADS.P2)
fast_read_p3 = AnalogIn(fast_ADC, ADS.P3)

slow_read_p0 = AnalogIn(slow_ADC, ADS.P0)

#pins used, S0-GPIO19-35-BIT0 , S1-GPIO13-33-BIT1 , S2-GPIO6-31-BIT2 , S3-GPIO5-29-BIT3
BIT0=35
BIT1=33
BIT2=31
BIT3=29

# channel encoder teehee
CH0 = [0,0,0,0] 
CH1 = [1,0,0,0]
CH2 = [0,1,0,0]
CH3 = [1,1,0,0]
CH4 = [0,0,1,0]
CH5 = [1,0,1,0]
CH6 = [0,1,1,0]
CH7 = [1,1,1,0]
CH8 = [0,0,0,1]
CH9 = [1,0,0,1]
CH10 = [0,1,0,1]
CH11 = [1,1,0,1]
CH12 = [0,0,1,1]
CH13 = [1,0,1,1]
CH14 = [0,1,1,1]
CH15 = [1,1,1,1]

#Set GPIO mode
GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)

GPIO.setup(BIT0, GPIO.OUT, initial=0) 
GPIO.setup(BIT1, GPIO.OUT, initial=0) 
GPIO.setup(BIT2, GPIO.OUT, initial=0) 
GPIO.setup(BIT3, GPIO.OUT, initial=0) 

v12_csense = 0
v12_vsense = 1
wrd_sig = 2
rb_rb_sig = 3
rb_lb_sig = 4
rb_da_sig = 5
rbl_temp = 6
rbr_temp = 7

#list of channels haha
channel_list = [CH0,CH1,CH2,CH3,CH4,CH5,CH6,CH7,CH8,CH9,CH10,CH11,CH12,CH13,CH14,CH15]
slow_read_list = [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]

def slow_sensor_sweep(event):
	for channel in channel_list:
		GPIO.output(BIT0, channel[0])
		GPIO.output(BIT1, channel[1])
		GPIO.output(BIT2, channel[2])
		GPIO.output(BIT3, channel[3])
		
		slow_read_list[channel_list.index(channel)]=round(slow_read_p0.value,2)
	
	SlowPublisher = rospy.Publisher('slow_sensor_data_stream', SlowAnalogData, queue_size=1)
	slowdata = SlowAnalogData()
	
	slowdata.v12_csense = slow_read_list[v12_csense]
	slowdata.v12_vsense = slow_read_list[v12_vsense]
	slowdata.wrd_sig = slow_read_list[wrd_sig]
	slowdata.rb_rb_sig = slow_read_list[rb_rb_sig]
	slowdata.rb_lb_sig = slow_read_list[rb_lb_sig]
	slowdata.rb_da_sig = slow_read_list[rb_da_sig]
	slowdata.rbl_temp = slow_read_list[rbl_temp]
	slowdata.rbr_temp = slow_read_list[rbr_temp]
	
	SlowPublisher.publish(slowdata);
	
def fast_sensor_sweep(event):	
	FastPublisher = rospy.Publisher('fast_sensor_data_stream', FastAnalogData, queue_size=1)
	fastdata = FastAnalogData()	
	fastdata.table_sig = fast_read_p0.value
	fastdata.bd_sig = fast_read_p1.value	
	fastdata.ad_sig = fast_read_p2.value	
	fastdata.wpd_sig = fast_read_p3.value	
	
	FastPublisher.publish(fastdata)

def init():
	rospy.init_node('sensor_input_node', anonymous=True)
	rospy.loginfo("Sensor Input node started")
	rospy.Timer(rospy.Duration(0.1), slow_sensor_sweep)	
	rospy.Timer(rospy.Duration(0.01), fast_sensor_sweep)
	rospy.spin()

if __name__ == '__main__':
    try :
    	init()
    except rospy.ROSInterruptException:
        pass
        

