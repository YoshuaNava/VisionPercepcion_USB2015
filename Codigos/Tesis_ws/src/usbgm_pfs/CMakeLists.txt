CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(usbgm_pfs)


SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")


FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
	roscpp
	image_transport
	cv_bridge
)


CATKIN_PACKAGE(
	INCLUDE_DIRS include
	DEPENDS OpenCV Eigen
	CATKIN_DEPENDS roscpp image_transport cv_bridge
)

INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} include)
INCLUDE_DIRECTORIES(include test)


LIST(APPEND LINK_LIBS 
	${OpenCV_LIBRARIES}
	${roscpp_LIBRARIES}
	${cv_bridge_LIBRARIES}
	${image_transport_LIBRARIES}
	${Eigen_LIBRARIES}
)

ADD_LIBRARY(libsuperpixel SHARED
	include/libsuperpixel/superpixel.cpp
	include/libsuperpixel/superpixel.h
)

ADD_LIBRARY(slic SHARED
	include/slic_modified/slic.cpp
	include/slic_modified/slic.h
)
TARGET_LINK_LIBRARIES(slic libsuperpixel)

ADD_LIBRARY(egbis SHARED 
	include/egbis/convolve.h
	include/egbis/disjoint-set.h
	include/egbis/filter.h
	include/egbis/image.h
	include/egbis/imconv.h
	include/egbis/imutil.h
	include/egbis/misc.h
	include/egbis/segment-graph.h
	include/egbis/segment-image.h
	include/egbis/egbis.h
	include/egbis/egbis.cpp
)
TARGET_LINK_LIBRARIES(egbis libsuperpixel)


ADD_LIBRARY(fps SHARED 
	include/probabilistic_functions.h
	src/probabilistic_functions.cpp
	include/hough_horizon_search.h
	src/hough_horizon_search.cpp
	include/segmentation_handler.h
	src/segmentation_handler.cpp
	include/global.h
)
TARGET_LINK_LIBRARIES(fps egbis)
TARGET_LINK_LIBRARIES(fps slic)
TARGET_LINK_LIBRARIES(fps ${LINK_LIBS})


ADD_LIBRARY(gps_sapienza_mod SHARED 
	include/gps_sapienza/img_proc_functions.h
	include/gps_sapienza/img_proc_functions.cpp
	include/gps_sapienza/init_structures.h
	include/gps_sapienza/init_structures.cpp
)
TARGET_LINK_LIBRARIES(gps_sapienza_mod ${LINK_LIBS})




ADD_EXECUTABLE(test_prob_floor_prior test/horizon_search/test_prob_floor_prior.cpp)
TARGET_LINK_LIBRARIES(test_prob_floor_prior fps)

ADD_EXECUTABLE(test_encapsulated_horizon_search test/horizon_search/test_encapsulated_horizon_search.cpp)
TARGET_LINK_LIBRARIES(test_encapsulated_horizon_search fps)


ADD_EXECUTABLE(test_egbis_superpixel_segmentation test/segmentation/test_egbis_superpixel_segmentation.cpp)
TARGET_LINK_LIBRARIES(test_egbis_superpixel_segmentation ${LINK_LIBS})
TARGET_LINK_LIBRARIES(test_egbis_superpixel_segmentation egbis)

ADD_EXECUTABLE(test_slic_superpixel_segmentation test/segmentation/test_slic_superpixel_segmentation.cpp)
TARGET_LINK_LIBRARIES(test_slic_superpixel_segmentation ${LINK_LIBS})
TARGET_LINK_LIBRARIES(test_slic_superpixel_segmentation slic)

ADD_EXECUTABLE(test_histogram_superpixel_comparison test/segmentation/test_histogram_superpixel_comparison.cpp)
TARGET_LINK_LIBRARIES(test_histogram_superpixel_comparison fps)


ADD_EXECUTABLE(test_sapienza_features test/gps_sapienza/test_sapienza_features.cpp)
TARGET_LINK_LIBRARIES(test_sapienza_features fps)
TARGET_LINK_LIBRARIES(test_sapienza_features gps_sapienza_mod)


ADD_EXECUTABLE(test_sapienza_full test/gps_sapienza/test_sapienza_full.cpp)
TARGET_LINK_LIBRARIES(test_sapienza_full fps)
TARGET_LINK_LIBRARIES(test_sapienza_full gps_sapienza_mod)

# ADD_EXECUTABLE(test_sapienza_features test/gps_sapienza/test_sapienza_features.cpp)
# TARGET_LINK_LIBRARIES(test_sapienza_features fps)
# TARGET_LINK_LIBRARIES(test_sapienza_features gps_sapienza_mod)






