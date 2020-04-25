#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64
from ubiquitous_display_pantilt.srv import AddPoints, AddPointsRequest

import sys
import math
import time

if __name__ == '__main__':

    rospy.init_node('test_angular_speed')

    pan_pub = rospy.Publisher('/ubiquitous_display/pan_controller/command', Float64, queue_size=10)
    rospy.sleep(2)

    set_pantilt = rospy.ServiceProxy('neuro/action_plan_pantilt', AddPoints)
    pt_msg = AddPointsRequest()
    pt_msg.position.x = -(math.pi / 2.0)

    pan_pub.publish(0.0)
    rospy.sleep(3)
    start = time.time()
    print ("start: ", start)
    responce = set_pantilt(pt_msg)
    if responce.success:
        elapsed_time = time.time() - start
        print ("diff time: ", elapsed_time)
