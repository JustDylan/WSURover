cmake_minimum_required(VERSION 2.8.3)
project(pwm_pca9685_0)

add_compile_options(-std=c++11 -li2c)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs)

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pca9685_node_0 src/pca9685_activity_0.cpp  src/pca9685_node_0.cpp)

include_directories(${catkin_INCLUDE_DIRS})

target_link_libraries(pca9685_node_0 ${catkin_LIBRARIES})

install(TARGETS pca9685_node_0
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

