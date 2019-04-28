#!/usr/bin/env python

import rospy
from people_msgs.msg import PositionMeasurementArray

def callback(data):
    if data.people:
        print "success"
    else:
        pass

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ptm_listener', anonymous=True)

    rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
