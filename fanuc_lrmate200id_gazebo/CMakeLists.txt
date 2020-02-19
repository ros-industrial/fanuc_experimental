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
  arm_perception_utilities
 )

find_package(gazebo 9 REQUIRED)
 
if(NOT EIGEN3_INCLUDE_DIRS)
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}  )
include_directories(include ${arm_perception_utilities_INCLUDE_DIRS})
include_directories(include ~/catkin_ws/src/ROS_FANUC_LRMATE200ID/arm_perception_utilities/include/)

catkin_package(
  #INCLUDE_DIRS  ${GAZEBO_INCLUDE_DIRS}
  LIBRARIES  
  CATKIN_DEPENDS roscpp gazebo_ros  
#  DEPENDS system_lib EIGEN3 arm_perception_utilities
)  

set(SOURCES
      ~/catkin_ws/src/ROS_FANUC_LRMATE200ID/arm_perception_utilities/src/pickplace/PickandPlacer.cpp
      ~/catkin_ws/src/ROS_FANUC_LRMATE200ID/arm_perception_utilities/src/utils.cpp
      ~/catkin_ws/src/ROS_FANUC_LRMATE200ID/arm_perception_utilities/src/control/RobotController.cpp
      ~/catkin_ws/src/ROS_FANUC_LRMATE200ID/arm_perception_utilities/src/pickplace/PickandPlacer.cpp)
 
add_library(model_push SHARED plugins/MoveObject.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(random_model_push SHARED plugins/RandomMoveObject.cc)
target_link_libraries(random_model_push ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(sim_pickplace SHARED plugins/SimulationPickandPlace.cc  ${SOURCES})
target_link_libraries(sim_pickplace ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${OpenCV_LIBS} ${arm_perception_utilities})
install(TARGETS sim_pickplace DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(sim_pickplace ${PROJECT_NAME}_EXPORTED_TARGETS})


install(TARGETS  
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
