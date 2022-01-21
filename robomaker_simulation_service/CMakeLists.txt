cmake_minimum_required(VERSION 2.8.3)
project(robomaker_simulation_service)

find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup()

catkin_package(
	CATKIN_DEPENDS rospy message_runtime robomaker_simulation_msgs std_msgs
)

catkin_install_python(PROGRAMS 
  scripts/robomaker_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
