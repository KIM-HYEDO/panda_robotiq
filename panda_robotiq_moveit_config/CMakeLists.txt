cmake_minimum_required(VERSION 3.0.2)
project(panda_robotiq_moveit_config)

find_package(catkin REQUIRED)

# Python module panda_robotiq_moveit_config.gripper (src/)
catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS scripts/gripper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
