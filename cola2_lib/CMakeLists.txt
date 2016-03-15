cmake_minimum_required(VERSION 2.8.3)
project(cola2_lib)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
                  genmsg
                  rospy
                  auv_msgs
                  cola2_msgs
                  diagnostic_msgs
                  geometry_msgs
                  nav_msgs
                  roscpp
                  sensor_msgs
                  std_msgs
                  std_srvs
                  tf
                  tf_conversions
                  visualization_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES cola2_lib
   CATKIN_DEPENDS auv_msgs
                  cola2_msgs
                  diagnostic_msgs
                  geometry_msgs
                  nav_msgs
                  roscpp
                  sensor_msgs
                  std_msgs
                  std_srvs
                  tf
                  tf_conversions
                  visualization_msgs
   DEPENDS Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

## Declare a cpp library
add_library(cola2_lib
   src/${PROJECT_NAME}/cola2_util.cpp
   src/${PROJECT_NAME}/SGDerivation.cpp
   src/${PROJECT_NAME}/cola2_control/IAUVController.cpp
   src/${PROJECT_NAME}/cola2_control/IAUVROSController.cpp
   src/${PROJECT_NAME}/cola2_control/IController.cpp
   src/${PROJECT_NAME}/cola2_control/Merge.cpp
   src/${PROJECT_NAME}/cola2_control/Model.cpp
   src/${PROJECT_NAME}/cola2_control/NDofController.cpp
   src/${PROJECT_NAME}/cola2_control/Pid.cpp
   src/${PROJECT_NAME}/cola2_control/Poly.cpp
   src/${PROJECT_NAME}/cola2_control/Request.cpp
   src/${PROJECT_NAME}/cola2_control/RobustifyingTerms.cpp
   src/${PROJECT_NAME}/cola2_io/CCamera.cpp
   src/${PROJECT_NAME}/cola2_io/ISerialPort.cpp
   src/${PROJECT_NAME}/cola2_io/SerialPort.cpp
   src/${PROJECT_NAME}/cola2_io/TcpSocket.cpp
   src/${PROJECT_NAME}/cola2_io/UdpSocket.cpp
   src/${PROJECT_NAME}/cola2_navigation/EkfBase.cpp
   src/${PROJECT_NAME}/cola2_navigation/EkfRosBase.cpp
   src/${PROJECT_NAME}/cola2_navigation/EkfSlamAuv.cpp
   src/${PROJECT_NAME}/cola2_navigation/nav_utils.cpp
   src/${PROJECT_NAME}/cola2_navigation/Ned.cpp
   src/${PROJECT_NAME}/cola2_navigation/transformations.cpp
   src/${PROJECT_NAME}/cola2_rosutils/DiagnosticHelper.cpp
   src/${PROJECT_NAME}/cola2_rosutils/RosParamLoader.cpp
   src/${PROJECT_NAME}/cola2_rosutils/RosUtil.cpp
)

## Declare a cpp executable
# add_executable(cola2_lib_node src/cola2_lib_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(cola2_lib_node auv_msgs_generate_messages_cpp)
add_dependencies(cola2_lib auv_msgs_generate_messages_cpp cola2_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(cola2_lib_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  src/cola2_lib/cola2_lib.py
  src/cola2_lib/cola2_ros_lib.py
  src/cola2_lib/diagnostic_helper.py
  src/cola2_lib/JoystickBase.py
  src/cola2_lib/NED.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
# install(TARGETS cola2_lib cola2_lib_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cola2_lib.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
