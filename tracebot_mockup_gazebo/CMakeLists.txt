cmake_minimum_required(VERSION 3.0.2)
project(tracebot_mockup_gazebo)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

catkin_install_python(PROGRAMS scripts/gazebo_model_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
