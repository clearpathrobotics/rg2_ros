cmake_minimum_required(VERSION 3.0.2)
project(rg2_ft_description)

find_package(catkin REQUIRED)

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
