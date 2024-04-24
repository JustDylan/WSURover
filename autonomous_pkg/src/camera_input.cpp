// Include the ROS library
#include <ros/ros.h>

// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/registry.hpp>

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
//using namespace cv;

//ROS publish topic
static const std::string PUBLISH_TOPIC = "videostream";

//Video settings
//640x480 or 1280x960 at 30 fps, 2592x1944 at 15 fps
static const int FRAME_RATE = 30;
static const int VIDEO_WIDTH = 1280;
static const int VIDEO_HEIGHT = 960; 

int main(int argc, char *argv[])
{
	//initialize ros
	ros::init(argc, argv, "camera_input");
	
	ros::NodeHandle nodeHandle;
	
	//Image publisher
	ros::Publisher pub = nodeHandle.advertise<sensor_msgs::Image>(
			PUBLISH_TOPIC, 
			0);
			
	ros::Rate loop_rate(FRAME_RATE);

	//Matrix to load frames
	cv::Mat capFrame;
	
	//Camera's videocapture object
	cv::VideoCapture cap;
	
	//Open camera's video stream
	cap.open(0, cv::CAP_V4L2);
	
	//Set video stream settings
	cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
	cap.set(cv::CAP_PROP_FPS, FRAME_RATE);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT);
	
	//Check if video stream was found
	if (!cap.isOpened()){
		cout << "No video stream detected" << endl;
		return -1;
	}
	
	//Read frames from video stream
	while (ros::ok())
	{
		cap >> capFrame;
		
		//Breaking the loop if no video frame is detected
		if (capFrame.empty()){
			break;
		}
		
		//header for image message
		std_msgs::Header imgMsgHeader = std_msgs::Header();
		imgMsgHeader.stamp = ros::Time::now();
		
		//publish converted image message
		pub.publish(
			cv_bridge::CvImage(
				imgMsgHeader, 
					"bgr8", 
					capFrame
				).toImageMsg());
		
		loop_rate.sleep();
	}
	
	//Releasing the buffer memory
	cap.release();
	return 0;
}
