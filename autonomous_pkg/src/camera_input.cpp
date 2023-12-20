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
using namespace cv;

  // OpenCV Window Name
  static const std::string OPENCV_WINDOW = "Image window";

  static const std::string PUBLISH_TOPIC = "camera_stream1";

  // Publisher
  ros::Publisher pub;

  void image_cb(const sensor_msgs::ImageConstPtr& msg)
  {
    std_msgs::Header msg_header = msg->header;
    std::string frame_id = msg_header.frame_id.c_str();
    ROS_INFO_STREAM("New Image from " << frame_id);

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

    // Draw a timestamp of the current date and time in the top left of the image
    // FIX-ME: std::asctime appends a '\n' character to the end of the string
    std::time_t result = msg_header.stamp.sec;
    std::stringstream ss;
    ss << std::asctime(std::localtime(&result));

    // Get the size of the text for measurement
    cv::Size text = cv::getTextSize(ss.str().c_str(), FONT_HERSHEY_SIMPLEX, 0.4, 1, 0);

    // Put the text in the bottom right corner
    cv::Point text_point = cvPoint(cv_ptr->image.cols - 20 - text.width, cv_ptr->image.rows - 20 - text.height);

    // Draw a black background behind text
    cv::rectangle(cv_ptr->image, text_point, text_point + cv::Point(text.width, -text.height), CV_RGB(0,0,0), FILLED);

    // Draw the timestamp on the rectangle
    cv::putText(cv_ptr->image, ss.str().c_str(), text_point, FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255,255,255));

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Draw an example crosshair
    cv::drawMarker(cv_ptr->image, cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2),  cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    pub.publish(cv_ptr->toImageMsg());
  }
  

int main(int argc, char *argv[])
{
/*
	auto backendList = videoio_registry::getBackends();
	
	for(auto backend : backendList)
	{
		cout << videoio_registry::getBackendName(backend) << ", ";
	}
	*/

	//initialize ros
	ros::init(argc, argv, "camera_input");
	
	ros::NodeHandle nodeHandle;
	
	//game pad publisher
	pub = nodeHandle.advertise<sensor_msgs::Image>(
			PUBLISH_TOPIC, 
			0);
			
	ros::Rate loop_rate(30);

	sensor_msgs::ImagePtr frameMsg;

	Mat myImage;//Declaring a matrix to load the frames//
	//namedWindow("Video Player", WINDOW_NORMAL);//Declaring the video to show the video//
	//resizeWindow("Video Player", 640, 480);
	
	VideoCapture cap;//Declaring an object to capture stream of frames from default camera//
	cap.open(0, CAP_V4L2); //CAP_OPENCV_MJPEG
	//cap.set(CAP_PROP_SETTINGS, 1);
	cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
	cap.set(CAP_PROP_FPS, 30);
	cap.set(CAP_PROP_FRAME_WIDTH, 1280);//640x480 or 1280x960 at 30 fps, 2592x1944 at 15 fps
	cap.set(CAP_PROP_FRAME_HEIGHT, 960);
	
	if (!cap.isOpened()){ //This section prompt an error message if no video stream is found//
		cout << "No video stream detected" << endl;
		return -1;
	}
	while (ros::ok())
	{
		cap >> myImage;
		
		//Breaking the loop if no video frame is detected
		if (myImage.empty()){
			break;
		}
		
		cv_bridge::CvImage frameMsg;
		frameMsg.image = myImage;
		
		//publish image msg
		//frameMsg = frameMsg.toImageMsg();
		pub.publish(frameMsg.toImageMsg());
		
		//Showing the video
		//imshow("Video Player", myImage);
		
		loop_rate.sleep();
		
		//waitKey(25);
	}
	
	//Releasing the buffer memory
	cap.release();
	return 0;
}
