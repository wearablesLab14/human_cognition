cmake_minimum_required(VERSION 2.8.0)
project(human_cognition)
find_package(catkin)

find_package(catkin REQUIRED COMPONENTS qt_build roscpp tf)
include_directories(${catkin_INCLUDE_DIRS})

rosbuild_prepare_qt4(QtCore QtGui) 

catkin_package(CATKIN_DEPENDS roscpp tf)

foreach(subdir   
	node_common
	node_listener
    node_receiver    
    )
  #install(DIRECTORY ${subdir} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/qt_tutorials)
  add_subdirectory(${subdir})
endforeach()

install(DIRECTORY resources DESTINATION ${CMAKE_INSTALL_PREFIX}/share/qt_tutorials)
install(DIRECTORY ui DESTINATION ${CMAKE_INSTALL_PREFIX}/share/qt_tutorials)


