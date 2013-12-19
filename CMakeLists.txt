cmake_minimum_required(VERSION 2.8.3)
project(odometry_evaluation)
find_package(catkin REQUIRED COMPONENTS 
	rospy  
	tf)

catkin_package()

catkin_python_setup()

install(PROGRAMS scripts/evaluate scripts/utils
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})