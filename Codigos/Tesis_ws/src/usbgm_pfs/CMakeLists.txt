cmake_minimum_required(VERSION 2.8.3)
project(usbgm_pfs)


SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo


Find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  message_generation
)

find_package(OpenCV REQUIRED)
FIND_PACKAGE(svo REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)
include_directories(${catkin_INCLUDE_DIRS} ${svo_INCLUDE_DIRS} include)



SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
catkin_package(
	DEPENDS OpenCV fast svo
	CATKIN_DEPENDS roscpp
)

#hola
#target_link_libraries (usbgm_pfs slic)
#add_library(slic slic.cpp)

#add_executable(foo src/foo.cpp)


 add_executable(usbgm_pfs src/plane_segmentation_alpha.cpp)
 target_link_libraries(usbgm_pfs ${catkin_LIBRARIES})
 target_link_libraries(usbgm_pfs ${OpenCV_LIBS})
 
 add_library(slic_modified slic_modified/slic.cpp)
 add_library(superpixel_modified slic_modified/superpixel.cpp)
 target_link_libraries(usbgm_pfs slic_modified)
 target_link_libraries(usbgm_pfs superpixel_modified)
 ##add_dependencies(Convert_Image ar_pose_generate_messages_cpp)

