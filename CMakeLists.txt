cmake_minimum_required(VERSION 2.8.0)
project(human_cognition)
find_package(catkin)

find_package(catkin REQUIRED COMPONENTS qt_build roscpp tf urdf)

include_directories(${catkin_INCLUDE_DIRS})

include_directories($CMAKE_CURRENT_BINARY_DIR}/node_common)

rosbuild_prepare_qt4(QtCore QtGui) 

catkin_package(CATKIN_DEPENDS roscpp)

foreach(subdir   
	node_common
	node_listener
    node_receiver    
    ) 
  add_subdirectory(${subdir})
endforeach()
