#!/bin/bash

# this is the default, override if needed
VIDEO_MODE="-h 480 -w 640 -fps 15"
# this is best to reduce motion blur VIDEO_MODE="-ex sports -awb off -h 480 -w 640 -fps 30"
if [ $# = 1 ]; then
	VIDEO_MODE=$1
fi

rostopic pub --once /pi_cmd std_msgs/String "setvideomode ${VIDEO_MODE}"
