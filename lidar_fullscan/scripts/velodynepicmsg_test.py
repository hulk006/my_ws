#!/usr/bin/env python
import rospy

from lidar_msg.msg import VelodynePic
def callback(data):
    print data.col

def velodynepic_listener():
    rospy.Subscriber("rawdata_fullscan", VelodynePic, callback)
    rospy.init_node('vepic_test', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    try:
        velodynepic_listener()
    except KeyboardInterrupt, e:
        pass
    print "exiting"
