#!/bin/bash

set -x

apt-get update -qq && apt-get install -y -q wget sudo lsb-release gnupg git sed # for docker
echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections

echo "Testing branch $TRAVIS_BRANCH of $REPOSITORY_NAME"
sudo sh -c "echo \"deb ${REPOSITORY} `lsb_release -cs` main\" > /etc/apt/sources.list.d/ros-latest.list"
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
sudo apt-get install -qq -y python-catkin-pkg python-rosdep python-catkin-tools python-wstool ros-${ROS_DISTRO}-catkin

source /opt/ros/${ROS_DISTRO}/setup.bash
sudo rosdep init
rosdep update
# script:
(cd ${CI_SOURCE_PATH}; git log --oneline | head -10)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
ln -sf ${CI_SOURCE_PATH} src/${REPOSITORY_NAME}
wstool init src
wstool update -t src
rosdep install --from-paths src -y -q -r --ignore-src --rosdistro ${ROS_DISTRO} # -r is indisapensible
env | grep ROS
rosversion catkin
# Build
catkin build -p 1 -j 1
# TODO: rostest
## Run tests
#catkin run_tests
## check test (this only works on indigo)
#catkin_test_results --verbose build

