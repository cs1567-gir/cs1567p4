#!/usr/bin/env bash


export ROS_MASTER_URI="http://$1.cs.pitt.edu:11311"
export ROS_IP=$(/sbin/ifconfig wlan0 | grep "inet addr" | awk -F: '{print $2}' | awk '{print $1}')
source devel/setup.bash

