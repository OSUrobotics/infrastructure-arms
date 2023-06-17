cmake_minimum_required(VERSION 2.8.3)
project(kinova_description)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  actionlib
  actionlib_msgs
)
#find_package(catkin REQUIRED)

add_service_files(
  FILES
  Data.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

#include_directories(
#	${catkin_INCLUDE_DIRS}
#)
#
#add_executable(drawer_updater
#	src/drawer_updater.py
#)
#target_link_libraries(drawer_updater
#	${catkin_LIBRARIES}
#)
#
#add_executable(data_plotter
#	src/data_plotter.py
#)
#target_link_libraries(data_plotter
#	${catkin_LIBRARIES}
#)

install(DIRECTORY launch urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   FILES_MATCHING PATTERN "*.dae"
 )
#install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
