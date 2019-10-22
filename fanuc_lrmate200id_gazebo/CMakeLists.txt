cmake_minimum_required(VERSION 2.8.3)
project(fanuc_lrmate200id_gazebo)

find_package(OpenCV REQUIRED )
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  tf
  gazebo_ros
  gazebo_plugins
  roscpp
  fanuc_arm_for_chicony
 )

find_package(gazebo 9 REQUIRED)
 
if(NOT EIGEN3_INCLUDE_DIRS)
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}  )
include_directories(include ${fanuc_arm_for_chicony_INCLUDE_DIRS})
include_directories(include /home/atas/catkin_ws/src/ROS_FANUC_LRMATE200ID/fanuc_arm_for_chicony/include/)

catkin_package(
  #INCLUDE_DIRS  ${GAZEBO_INCLUDE_DIRS}
  LIBRARIES  
  CATKIN_DEPENDS roscpp gazebo_ros  
#  DEPENDS system_lib EIGEN3 fanuc_arm_for_chicony
)  

set(SOURCES
      /home/atas/catkin_ws/src/ROS_FANUC_LRMATE200ID/fanuc_arm_for_chicony/src/pickplace/PickandPlacer.cpp
      /home/atas/catkin_ws/src/ROS_FANUC_LRMATE200ID/fanuc_arm_for_chicony/src/utils.cpp
      /home/atas/catkin_ws/src/ROS_FANUC_LRMATE200ID/fanuc_arm_for_chicony/src/control/RobotController.cpp
      /home/atas/catkin_ws/src/ROS_FANUC_LRMATE200ID/fanuc_arm_for_chicony/src/pickplace/PickandPlacer.cpp)
 
add_library(model_push SHARED plugins/MoveObject.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(sim_pickplace SHARED plugins/SimulationPickandPlace.cc  ${SOURCES})
target_link_libraries(sim_pickplace ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${OpenCV_LIBS} ${fanuc_arm_for_chicony})
install(TARGETS sim_pickplace DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(sim_pickplace ${PROJECT_NAME}_EXPORTED_TARGETS})


install(TARGETS  
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
