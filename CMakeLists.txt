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

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

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
  DEPENDS Eigen
)

###########
## Build ##
###########
include_directories(
  include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
)


## pluginlib_tutorials library
add_library(kf_pos_vel_acc_pluginlib src/kf_pos_vel_acc_plugin.cpp)
target_link_libraries(kf_pos_vel_acc_pluginlib ${catkin_LIBRARIES})

## plugin test
add_executable(kf_plugin_test src/kf_plugin_test.cpp)
target_link_libraries(kf_plugin_test ${catkin_LIBRARIES})



