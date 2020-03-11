cmake_minimum_required(VERSION 2.8.3)
project(fanuc_lrmate200id_gazebo_demo)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_python(${PROJECT_SOURCE_DIR}/scripts)
  roslint_add_test()
endif()
