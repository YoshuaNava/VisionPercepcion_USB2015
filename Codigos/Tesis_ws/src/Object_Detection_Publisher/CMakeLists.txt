cmake_minimum_required(VERSION 2.8.3)
project(Object_Detection_Publisher)

Find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  message_generation
)

find_package(OpenCV)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES convert_to_grayscale
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)




add_executable(Object_Detection_Publisher src/Object_Detection_Publisher.cpp)
 target_link_libraries(Object_Detection_Publisher ${catkin_LIBRARIES})
 target_link_libraries(Object_Detection_Publisher ${OpenCV_LIBS})
 ##add_dependencies(Convert_Image ar_pose_generate_messages_cpp)

