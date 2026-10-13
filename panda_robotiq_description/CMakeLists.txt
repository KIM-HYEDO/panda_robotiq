cmake_minimum_required(VERSION 3.0.2)
project(panda_robotiq_description)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/set_ee_frame.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY urdf launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
