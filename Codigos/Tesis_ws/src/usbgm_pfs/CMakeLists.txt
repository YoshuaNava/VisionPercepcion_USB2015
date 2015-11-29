CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(usbgm_pfs)


SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")


FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(svo REQUIRED)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
	roscpp
	image_transport
	cv_bridge
)


CATKIN_PACKAGE(
	INCLUDE_DIRS include
	DEPENDS OpenCV svo fast
	CATKIN_DEPENDS roscpp image_transport cv_bridge
)




INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${svo_INCLUDE_DIRS} include)



##add_dependencies(Convert_Image ar_pose_generate_messages_cpp)


ADD_EXECUTABLE(pfs src/plane_segmentation_alpha.cpp)
ADD_EXECUTABLE(test_prob_functions test/test_prob_functions.cpp)

ADD_LIBRARY(slic_superpixels slic_modified/slic.cpp slic_modified/superpixel.cpp)
TARGET_LINK_LIBRARIES(pfs ${catkin_LIBRARIES})
TARGET_LINK_LIBRARIES(pfs ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(pfs slic_superpixels)


TARGET_LINK_LIBRARIES(test_prob_functions ${catkin_LIBRARIES})
TARGET_LINK_LIBRARIES(test_prob_functions ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(test_prob_functions slic_superpixels)



