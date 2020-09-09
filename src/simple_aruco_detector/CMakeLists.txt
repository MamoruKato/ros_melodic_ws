cmake_minimum_required(VERSION 2.8.3)
project(simple_aruco_detector)

## Compile as C++11, supported in ROS Kinetic and newer


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        cv_bridge image_geometry geometry_msgs roscpp rospy std_msgs tf2 tf2_ros message_generation)

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################

add_message_files(
	FILES
	aruco_msg.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
        CATKIN_DEPENDS cv_bridge image_geometry geometry_msgs roscpp rospy std_msgs tf2 tf2_ros message_runtime
)

###########
## Build ##
###########


include_directories(
        ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/simple_aruco_detector_node.cpp)
add_executable(aruco_marker_detector_node src/aruco_marker_detector.cpp)
add_executable(aruco_camera_calibration src/camera_calibration.cpp)
add_executable(gazebo_aruco_detector src/gazebo_aruco_detector.cpp)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS})
target_link_libraries(aruco_marker_detector_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(aruco_camera_calibration ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(gazebo_aruco_detector ${catkin_LIBRARIES} ${OpenCV_LIBS})
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(aruco_marker_detector_node ${catkin_EXPORTED_TARGETS} simple_aruco_detecor_generate_messages_cpp)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
#install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES
#        launch/detector.launch
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#        )

