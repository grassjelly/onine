cmake_minimum_required(VERSION 2.8.3)
project(onine_description)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY images meshes launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})