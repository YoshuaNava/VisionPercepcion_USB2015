# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(usbgm_pfs_CONFIG_INCLUDED)
  return()
endif()
set(usbgm_pfs_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(usbgm_pfs_SOURCE_PREFIX /home/alfredoso/GitHub/VisionPercepcion_USB2015/Code/Thesis_ws/src/usbgm_pfs)
  set(usbgm_pfs_DEVEL_PREFIX /home/alfredoso/GitHub/VisionPercepcion_USB2015/Code/Thesis_ws/devel)
  set(usbgm_pfs_INSTALL_PREFIX "")
  set(usbgm_pfs_PREFIX ${usbgm_pfs_DEVEL_PREFIX})
else()
  set(usbgm_pfs_SOURCE_PREFIX "")
  set(usbgm_pfs_DEVEL_PREFIX "")
  set(usbgm_pfs_INSTALL_PREFIX /home/alfredoso/GitHub/VisionPercepcion_USB2015/Code/Thesis_ws/install)
  set(usbgm_pfs_PREFIX ${usbgm_pfs_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'usbgm_pfs' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(usbgm_pfs_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/home/alfredoso/GitHub/VisionPercepcion_USB2015/Code/Thesis_ws/src/usbgm_pfs/include;/usr/include/opencv;/usr/include;/usr/include/eigen3 " STREQUAL " ")
  set(usbgm_pfs_INCLUDE_DIRS "")
  set(_include_dirs "/home/alfredoso/GitHub/VisionPercepcion_USB2015/Code/Thesis_ws/src/usbgm_pfs/include;/usr/include/opencv;/usr/include;/usr/include/eigen3")
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${usbgm_pfs_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'usbgm_pfs' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  Ask the maintainer 'Rafael Colmenares <rafaelcolmenaresusb@gmail.com>' to fix it.")
      endif()
    else()
      message(FATAL_ERROR "Project 'usbgm_pfs' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/alfredoso/GitHub/VisionPercepcion_USB2015/Code/Thesis_ws/src/usbgm_pfs/${idir}'.  Ask the maintainer 'Rafael Colmenares <rafaelcolmenaresusb@gmail.com>' to fix it.")
    endif()
    _list_append_unique(usbgm_pfs_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "/usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8;/usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8;/usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8;/usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8;/usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8;/usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8;/usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8;/usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8;/usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8;/usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8;/usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8;/usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8;/usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8;/usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8;/usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8;/usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8;/usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND usbgm_pfs_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND usbgm_pfs_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND usbgm_pfs_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/alfredoso/GitHub/VisionPercepcion_USB2015/Code/Thesis_ws/devel/lib;/home/alfredoso/Pleiades/artags_22_nov_2015/devel/lib;/home/alfredoso/Pleiades/spirigo_18_nov_2015_ws/devel/lib;/home/alfredoso/Pleiades/visual_slam_12_oct_2015/ps4eye_ws/devel/lib;/home/alfredoso/Pleiades/rune_finder_11_oct_2015/spiri_catkin/devel/lib;/home/alfredoso/Pleiades/arducopter-pixhawk/simulators/catkin_ws/devel/lib;/opt/ros/indigo/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(usbgm_pfs_LIBRARY_DIRS ${lib_path})
      list(APPEND usbgm_pfs_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'usbgm_pfs'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND usbgm_pfs_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(usbgm_pfs_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${usbgm_pfs_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "roscpp;image_transport;cv_bridge")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 usbgm_pfs_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${usbgm_pfs_dep}_FOUND)
      find_package(${usbgm_pfs_dep} REQUIRED)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${usbgm_pfs_dep} REQUIRED ${depend_list})
  endif()
  _list_append_unique(usbgm_pfs_INCLUDE_DIRS ${${usbgm_pfs_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(usbgm_pfs_LIBRARIES ${usbgm_pfs_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${usbgm_pfs_dep}_LIBRARIES})
  _list_append_deduplicate(usbgm_pfs_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(usbgm_pfs_LIBRARIES ${usbgm_pfs_LIBRARIES})

  _list_append_unique(usbgm_pfs_LIBRARY_DIRS ${${usbgm_pfs_dep}_LIBRARY_DIRS})
  list(APPEND usbgm_pfs_EXPORTED_TARGETS ${${usbgm_pfs_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${usbgm_pfs_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
