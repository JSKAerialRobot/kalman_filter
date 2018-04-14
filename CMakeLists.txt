cmake_minimum_required(VERSION 2.8.3)
project(kalman_filter)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  dynamic_reconfigure
  cmake_modules
)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
generate_dynamic_reconfigure_options(
  cfg/KalmanFilterPosVelAccBias.cfg
  cfg/KalmanFilterPosVelAcc.cfg
  )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs pluginlib
  DEPENDS Eigen3
)

###########
## Build ##
###########
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)


## pluginlib_tutorials library
add_library(kf_pos_vel_acc_pluginlib src/kf_pos_vel_acc_plugin.cpp)
target_link_libraries(kf_pos_vel_acc_pluginlib ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
add_dependencies(kf_pos_vel_acc_pluginlib ${PROJECT_NAME}_gencfg)

## plugin test
add_executable(kf_plugin_test src/kf_plugin_test.cpp)
target_link_libraries(kf_plugin_test ${catkin_LIBRARIES})
add_dependencies(kf_pos_vel_acc_pluginlib ${PROJECT_NAME}_gencfg ${Eigen3_LIBRARIES})


