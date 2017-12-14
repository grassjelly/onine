cmake_minimum_required(VERSION 2.8.3)
project(onine_apps)

find_package(catkin REQUIRED COMPONENTS moveit_core moveit_ros_planning moveit_ros_planning_interface cmake_modules)

catkin_package(
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)
