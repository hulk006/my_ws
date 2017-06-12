# How to run the code first time

## Install the ROS Indigo Distribution
Install the ROS Indigo Distribution on you ubuntu 14.04. 

Please follow the belowing:
http://wiki.ros.org/indigo/Installation/Ubuntu

## Let ROS know the packages
Clone the respostory in the directory you like, e.g ~/catkin_ws/.
``` Shell
mkdir -p ~/catkin_ws/
cd ~/catkin_ws
git clone http://192.168.1.20/pointcloud_group/autoCar.git src
```
Name the autoCar to src in the catkin_ws so that ROS can find the packages.

Now you can run the command:
``` Shell
catkin_make
```
Note, since classify_sdk added, "__throw_out_of_range" not reference error may occur,
please upgrade you gcc and g++ version to 4.9.

## Solve the package dependence on velodyne_driver problem
We have the velodyne submodule.
``` Shell
git submodule update --init --recursive
```
## Run catkin_make to build
```Shell
cd ~/catkin_ws
catkin_make
```
## Run the demo
Copy the data in the directory data, include the files: lidar.pcap obd.txt roi.txt.

Run the following command:

```Shell
roslaunch lidar_classify launchROI.launch
```
When the rviz show up:
1.  Set Fixed Frame: map2
2.  Add Display:         PointCloud2
*    Choose Topic:     /pc_cloud
*    Select the Style: Points

The default rviz config is in the lidar_classify/config/lidar_classify_default.rviz.
