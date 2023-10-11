cmake_minimum_required(VERSION 3.0.2)
project(rover_pkg)


find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
)



#Generate messages in the 'msg' folder
 add_message_files(
   FILES
   UserInput.msg
   ArmData.msg
   RockerBogieData.msg
   ScienceData.msg
   SlowAnalogData.msg
   FastAnalogData.msg
   ToolData.msg
   CameraControlData.msg
   RoverStateData.msg
   
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
	#INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

file(GLOB python_scripts
	"src/*.py"
)


catkin_install_python( 
	PROGRAMS ${python_scripts}
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

























