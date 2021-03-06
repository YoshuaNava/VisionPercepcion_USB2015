CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(gps_sapienza_ros)


SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")


FIND_PACKAGE(OpenCV REQUIRED)


FIND_PACKAGE(catkin REQUIRED COMPONENTS
	roscpp
	image_transport
	cv_bridge
)


CATKIN_PACKAGE(
	INCLUDE_DIRS include
	DEPENDS OpenCV
	CATKIN_DEPENDS roscpp image_transport cv_bridge
)


INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)

LIST(APPEND LINK_LIBS 
	${OpenCV_LIBRARIES}
	${roscpp_LIBRARIES}
	${cv_bridge_LIBRARIES}
	${image_transport_LIBRARIES}
)



LIST(APPEND SOURCEFILES
  src/capture.cpp
  src/img_proc_fcns.cpp
  src/init_structures.cpp
  src/ms_overwrite_safe_buffer.cpp
  )


ADD_EXECUTABLE(gps_sapienza src/main.cpp)

ADD_LIBRARY(gps_sapienza_lib ${SOURCEFILES})
TARGET_LINK_LIBRARIES(gps_sapienza gps_sapienza_lib)
TARGET_LINK_LIBRARIES(gps_sapienza ${OpenCV_LIBS})


