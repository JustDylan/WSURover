#!/usr/bin/env python3.6

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
cv2.startWindowThread()

def callback(data):
	br=CvBridge()
	current_frame = br.compressed_imgmsg_to_cv2(data)
	cv2.namedWindow('feed', cv2.WINDOW_AUTOSIZE)
	cv2.imshow("camera",current_frame)
	cv2.waitKey(10)

def recieve_message():
	rospy.init_node('videoreturn', anonymous=True)
	rospy.Subscriber('videostream', CompressedImage, callback)
	rospy.spin()
	print("hey")
	cv2.destroyAllWindows()


if __name__ == '__main__':
	recieve_message() 


