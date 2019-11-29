cmake_minimum_required(VERSION 3.1.0)

project(fanuc_r2000ib_support)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(tests/roslaunch_test_r2000ib165f.xml)
endif()

install(DIRECTORY config launch meshes urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
