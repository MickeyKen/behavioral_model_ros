#!/usr/bin/env python

import rospy
import numpy as np
import math

from people_msgs.msg import PositionMeasurementArray
from people_msgs.msg import PositionMeasurement
from nav_msgs.srv import GetMap

import message_filters


class Subscribe():
    def __init__(self):
        # get map_data
        self.map_service = rospy.ServiceProxy('static_map', GetMap)
        self.map_data = self.map_service()

        # Declaration Publisher
        self.pose_pub = rospy.Publisher("/filter/people_tracker_measurement", PositionMeasurementArray, queue_size = 10)

        # Declaration Subscriber (message_filters)
        self.ptm_sub = message_filters.Subscriber('/people_tracker_measurements', PositionMeasurementArray)
        self.leg_sub = message_filters.Subscriber('/leg_tracker_measurements', PositionMeasurementArray)
        ts = message_filters.ApproximateTimeSynchronizer([self.ptm_sub, self.leg_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)


    ### callback function for /people_tracker_measurements ###
    def callback(self, msg, leg):
        filterArray_msg = PositionMeasurementArray()
        filter_msg = PositionMeasurement()
        leg_count = 0

        # people found
        if msg.people:
            filterArray_msg.header = msg.header
            # search minimum distance between US and target_human
            for i in msg.people:
                leg_name = i.object_id
                leg_list = leg_name.split('|')
                print leg_list
                for j in leg.people:
                    if j.object_id == leg_list[0] or j.object_id == leg_list[1]:
                        map_occ = ((int((j.pos.y / self.map_data.map.info.resolution)-1) * self.map_data.map.info.width)+ int(j.pos.x / self.map_data.map.info.resolution)+1900+(4000*1899))
                        print self.map_data.map.data[map_occ]
                        if (self.map_data.map.data[map_occ] == 0):
                            leg_count += 1
                        else:
                            pass

                if leg_count == 2:
                    print "pass2"
                    filter_msg = i
                    filterArray_msg.people.append(filter_msg)
                leg_count = 0

            self.pose_pub.publish(filterArray_msg)

        # people not found
        else:
            pass

if __name__ == '__main__':
    rospy.init_node('filter_leg')

    Subscribe = Subscribe()

    rospy.spin()
