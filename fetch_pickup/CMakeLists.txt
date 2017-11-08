cmake_minimum_required(VERSION 2.8.3)
project(fetch_pickup)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    tf2
    tf2_ros
    tf2_eigen
    pluginlib
    geometric_shapes
    moveit_visual_tools
    gpd
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    gpd
    tf2
    tf2_ros
    tf2_eigen
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(move_group_interface_tutorial src/move_group_interface_tutorial.cpp)
target_link_libraries(move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pickUp src/pickUp.cpp)
target_link_libraries(pickUp ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pickUp ${catkin_EXPORTED_TARGETS})
