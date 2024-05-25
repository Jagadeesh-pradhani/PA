#!/bin/bash

ARG1=$1

if [ "$ARG1" == "forward" ]; then
    echo 'Moving forward and backward ...'
    rosrun rss_linux_pkg move_turtlebot_forward_backward.py

elif [ "$ARG1" == "rotate" ]; then
    echo 'Rotating ...'
    rosrun rss_linux_pkg move_turtlebot_circle.py

fi


