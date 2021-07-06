cmake_minimum_required(VERSION 3.0.2)
project(rg2_driver)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rg2_ft_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
