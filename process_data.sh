#!/bin/sh

# This is a comment!

if [ "$1" != "" ]; then
    echo "processing data at bag $1"
else
    echo "add in bag file as the first argument"
    exit
fi

# roslaunch blue_benchmarking process_data.launch file_name:=/home/zobot/.ros/$1
roslaunch blue_benchmarking process_data.launch file_name:=$1
mv ~/.ros/*.csv .
