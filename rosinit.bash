#!/usr/bin/env bash

export ROS_MASTER_URI="http://$1.cs.pitt.edu:11311"
#export ROS_IP=150.212.81.7
export ROS_HOSTNAME="$2.cs.pitt.edu"
source devel/setup.bash

