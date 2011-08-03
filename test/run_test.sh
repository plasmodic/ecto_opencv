#!/bin/sh -e
echo sourcing $1
. $1
echo PYTHONPATH=$PYTHONPATH
echo PATH=$PATH
echo ROS_ROOT=$ROS_ROOT
echo ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH

$2 $3


