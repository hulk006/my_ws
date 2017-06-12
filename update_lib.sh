#!/bin/sh

# catkin_make
cd ..
catkin_make

# update the header file
cp src/lidar_classify/src/classify/track.h src/classify_sdk/src/classify/track.h
cp src/lidar_classify/src/classify/drawing.h src/classify_sdk/src/classify/drawing.h

# update the lib file
cp devel/lib/libtrack.so src/classify_sdk/libso/libtrack.so
cp devel/lib/libdrawing.so src/classify_sdk/libso/libdrawing.so
