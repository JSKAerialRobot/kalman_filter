cmake_minimum_required(VERSION 2.8.3)
project(kalman_filter)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  tf_conversions
  eigen_conversions
  dynamic_reconfigure
  cmake_modules
)

find_package(Eigen3 REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
generate_dynamic_reconfigure_options(
  cfg/KalmanFilter.cfg
  )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs tf_conversions eigen_conversions pluginlib
  DEPENDS Eigen3
)

###########
## Build ##
###########
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")


## pluginlib_tutorials library
add_library(kf_pos_vel_acc_pluginlib src/kf_pos_vel_acc_plugin.cpp)
target_link_libraries(kf_pos_vel_acc_pluginlib ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(kf_pos_vel_acc_pluginlib ${PROJECT_NAME}_gencfg)

## test
add_executable(kf_plugin_test src/kf_plugin_test.cpp)
target_link_libraries(kf_plugin_test kf_pos_vel_acc_pluginlib ${catkin_LIBRARIES}  ${EIGEN3_LIBRARIES})
add_dependencies(kf_plugin_test ${PROJECT_NAME}_gencfg)

add_executable(lpf_test src/lpf_test.cpp)
target_link_libraries(lpf_test ${catkin_LIBRARIES})



