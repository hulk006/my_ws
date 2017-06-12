#!/bin/sh

# remove CMakeLists.txt.user*
rm $(find ./ -name "CMakeLists.txt.user*") -f

# remove any build directory
rm $(find ./ -name "*build*" -type d) -rf

# remove any .idea directory
rm $(find ./ -name "*.idea*" -type d) -rf


# make
cd ..
rm bulid -rf
rm devel -rf
catkin_make --pkg lidar_msg -j8
catkin_make --pkg rslidar -j8
catkin_make -j8
