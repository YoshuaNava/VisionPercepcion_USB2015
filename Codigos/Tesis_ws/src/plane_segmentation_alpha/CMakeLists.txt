cmake_minimum_required(VERSION 2.8.3)
project(plane_segmentation_alpha)

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



SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES convert_to_grayscale
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

#hola
#target_link_libraries (plane_segmentation_alpha slic)
#add_library(slic slic.cpp)

#add_executable(foo src/foo.cpp)


 add_executable(plane_segmentation_alpha src/plane_segmentation_alpha.cpp)
 target_link_libraries(plane_segmentation_alpha ${catkin_LIBRARIES})
 target_link_libraries(plane_segmentation_alpha ${OpenCV_LIBS})
 
 add_library(slic_modified slic_modified/slic.cpp)
 add_library(superpixel_modified slic_modified/superpixel.cpp)
 target_link_libraries(plane_segmentation_alpha slic_modified)
 target_link_libraries(plane_segmentation_alpha superpixel_modified)
 ##add_dependencies(Convert_Image ar_pose_generate_messages_cpp)

