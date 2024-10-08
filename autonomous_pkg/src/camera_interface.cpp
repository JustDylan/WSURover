// Include the ROS library
  #include <ros/ros.h>

  // Include opencv2
  #include <opencv2/imgproc/imgproc.hpp>
  #include <opencv2/highgui/highgui.hpp>

  // Include CvBridge, Image Transport, Image msg
  #include <image_transport/image_transport.h>
  #include <cv_bridge/cv_bridge.h>
  #include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;

  // OpenCV Window Name
  static const std::string OPENCV_WINDOW = "Image window";

  // Topics
  static const std::string IMAGE_TOPIC = "/camera/rgb/image_raw";
  static const std::string SUBSCRIBE_TOPIC = "/videostream";

  // Publisher
  ros::Publisher pub;
  
  //Subscriber
  ros::Subscriber sub;

  void imageCallBack(const sensor_msgs::ImageConstPtr& msg)
  {
    std_msgs::Header msg_header = msg->header;
    std::string frame_id = msg_header.frame_id.c_str();
    //ROS_INFO_STREAM("New Image from " << frame_id);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    //TODO Aruco tag Test
    /*
    cv::Mat markerImage;
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4x4_50);
    cv::aruco::generage
    */
    
    

    // Draw a timestamp of the current date and time in the top left of the image
    // FIX-ME: std::asctime appends a '\n' character to the end of the string
    std::time_t result = msg_header.stamp.sec;
    std::stringstream ss;
    ss << std::asctime(std::localtime(&result));

    // Get the size of the text for measurement
    //cv::Size text = cv::getTextSize(ss.str().c_str(), FONT_HERSHEY_SIMPLEX, 0.4, 1, 0);

    // Put the text in the bottom right corner
    //cv::Point text_point = cvPoint(cv_ptr->image.cols - 20 - text.width, cv_ptr->image.rows - 20 - text.height);

    // Draw a black background behind text
    //cv::rectangle(cv_ptr->image, text_point - cv::Point(2, -2), text_point + cv::Point(text.width+2, -text.height-2), CV_RGB(0,0,0), FILLED);

    // Draw the timestamp on the rectangle
    //cv::putText(cv_ptr->image, ss.str().c_str(), text_point, FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255,255,255));

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Draw an example crosshair
    cv::drawMarker(cv_ptr->image, cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2),  cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    //pub.publish(cv_ptr->toImageMsg());
  }
  

int main(int argc, char *argv[])
{
	//create opencv window
	namedWindow(OPENCV_WINDOW, WINDOW_NORMAL);
	resizeWindow(OPENCV_WINDOW, 640, 480);

	//initialize ros
	ros::init(argc, argv, "camera_interface");
	
	ros::NodeHandle nodeHandle;
	
	//camera stream subscriber		
	sub = nodeHandle.subscribe(SUBSCRIBE_TOPIC, 0, imageCallBack);
	
	while(ros::ok())
	{
		ros::spinOnce();
		waitKey(33);
	}

	return 0;
}
