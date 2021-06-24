cmake_minimum_required(VERSION 3.0.2)
project(rg2_description)

find_package(catkin REQUIRED)

install(DIRECTORY meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
