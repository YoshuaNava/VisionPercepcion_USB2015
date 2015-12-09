CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(usbgm_pfs)


SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")


FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(svo REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
	roscpp
	image_transport
	cv_bridge
)


CATKIN_PACKAGE(
	INCLUDE_DIRS include
	DEPENDS OpenCV svo Eigen
	CATKIN_DEPENDS roscpp image_transport cv_bridge
)

INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${svo_INCLUDE_DIRS} include)
INCLUDE_DIRECTORIES(include test)


LIST(APPEND LINK_LIBS 
	${OpenCV_LIBRARIES}
	${svo_LIBRARIES}
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


ADD_EXECUTABLE(pfs src/plane_segmentation_alpha.cpp)
TARGET_LINK_LIBRARIES(pfs ${LINK_LIBS})
TARGET_LINK_LIBRARIES(pfs slic)


ADD_EXECUTABLE(test_prob_floor_prior test/test_prob_floor_prior.cpp)
TARGET_LINK_LIBRARIES(test_prob_floor_prior ${LINK_LIBS})
TARGET_LINK_LIBRARIES(test_prob_floor_prior slic)
TARGET_LINK_LIBRARIES(test_prob_floor_prior egbis)


ADD_EXECUTABLE(test_horizon_search test/test_horizon_search.cpp)
TARGET_LINK_LIBRARIES(test_horizon_search ${LINK_LIBS})
TARGET_LINK_LIBRARIES(test_horizon_search slic)
TARGET_LINK_LIBRARIES(test_horizon_search egbis)


ADD_EXECUTABLE(test_prob_horizon_search test/test_prob_horizon_search.cpp)
TARGET_LINK_LIBRARIES(test_prob_horizon_search ${LINK_LIBS})
TARGET_LINK_LIBRARIES(test_prob_horizon_search slic)
TARGET_LINK_LIBRARIES(test_prob_horizon_search egbis)


ADD_EXECUTABLE(test_egbis_superpixel_segmentation test/test_egbis_superpixel_segmentation.cpp)
TARGET_LINK_LIBRARIES(test_egbis_superpixel_segmentation ${LINK_LIBS})
TARGET_LINK_LIBRARIES(test_egbis_superpixel_segmentation egbis)

ADD_EXECUTABLE(test_slic_superpixel_segmentation test/test_slic_superpixel_segmentation.cpp)
TARGET_LINK_LIBRARIES(test_slic_superpixel_segmentation ${LINK_LIBS})
TARGET_LINK_LIBRARIES(test_slic_superpixel_segmentation slic)

ADD_EXECUTABLE(test_histogram_superpixel_comparison test/test_histogram_superpixel_comparison.cpp)
TARGET_LINK_LIBRARIES(test_histogram_superpixel_comparison ${LINK_LIBS})
TARGET_LINK_LIBRARIES(test_histogram_superpixel_comparison slic)
TARGET_LINK_LIBRARIES(test_histogram_superpixel_comparison egbis)

ADD_EXECUTABLE(test_svo_depth_filter test/test_svo_depth_filter.cpp)
TARGET_LINK_LIBRARIES(test_svo_depth_filter ${LINK_LIBS})








