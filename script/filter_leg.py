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

        self.width = self.map_data.map.info.width
        self.height = self.map_data.map.info.height
        self.resolution = self.map_data.map.info.resolution
        self.origin_x = self.map_data.map.info.origin.position.x
        self.origin_y = self.map_data.map.info.origin.position.y

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
        cell_x = 0
        cell_y = 0
        map_occ = 0
        detect_flag = 0

        # people found
        if msg.people:
            filterArray_msg.header = msg.header
            # search minimum distance between US and target_human
            for i in msg.people:
                leg_name = i.object_id
                leg_list = leg_name.split('|')
                for j in leg.people:
                    if j.object_id == leg_list[0] or j.object_id == leg_list[1]:
                        cell_x = int((j.pos.x - self.origin_x) / self.resolution)
                        cell_y = int((j.pos.y - self.origin_y) / self.resolution)
                        map_occ = cell_x + (self.width * (cell_y - 1))
                        if (self.map_data.map.data[map_occ] == 0):
                            # for ang in range(0, 360, 60):
                            #     rad = math.radians(ang)
                            for c in range(1, 6):
                                if self.map_data.map.data[map_occ + c] == 0 and self.map_data.map.data[map_occ + (self.width * c)] == 0 and self.map_data.map.data[map_occ - c] == 0 and self.map_data.map.data[map_occ - (self.width * c)] == 0:
                                    pass
                                else:
                                    detect_flag = 1
                            if detect_flag != 1:
                                leg_count += 1
                        else:
                            pass
                if leg_count == 2:
                    filter_msg = i
                    filterArray_msg.people.append(filter_msg)
                    leg_count = 0

            filterArray_msg.header.stamp = rospy.Time.now()
            self.pose_pub.publish(filterArray_msg)

        # people not found
        else:
            pass

if __name__ == '__main__':
    rospy.init_node('filter_leg')

    Subscribe = Subscribe()

    rospy.spin()
