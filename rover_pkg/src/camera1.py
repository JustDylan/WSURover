#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

def publish_image():
    pub = rospy.Publisher('videostream', CompressedImage, queue_size=1)
    rospy.init_node('video_pub_py', anonymous=True)
    rate = rospy.Rate(10)
    cap = cv2.VideoCapture(0)
    br = CvBridge()
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        #frame = cv2.resize(frame,(640,360),cv2.INTER_LINEAR)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        if ret == True:
            pub.publish(br.cv2_to_compressed_imgmsg(frame))
        rate.sleep()

if __name__ == '__main__':
    try :
        publish_image()
    except rospy.ROSInterruptException:
        pass
