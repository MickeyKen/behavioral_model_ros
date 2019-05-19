#!/usr/bin/python
import rospy

import numpy as np

from people_msgs.msg import PositionMeasurementArray
from people_msgs.msg import PositionMeasurement

from nav_msgs.srv import GetMap

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int8

import message_filters


class Subscribe():
    def __init__(self):
        # get map_data
        self.static_map_service = rospy.ServiceProxy('static_map', GetMap)
        self.static_map_data = self.static_map_service()
        print self.static_map_data.map.info.origin

        self.heatmap_msg = OccupancyGrid()

        self.heatmap_msg.header = self.static_map_data.map.header
        # self.heatmap_msg.header.stamp = rospy.Time.now()

        self.heatmap_msg.info = self.static_map_data.map.info

        # set z aixs value for rviz
        self.heatmap_msg.info.origin.position.z = 0.2

        self.width = self.heatmap_msg.info.width
        self.height = self.heatmap_msg.info.height
        self.resolution = self.heatmap_msg.info.resolution
        self.origin_x = self.heatmap_msg.info.origin.position.x
        self.origin_y = self.heatmap_msg.info.origin.position.y

        self.heatmap_num = self.width * self.height
        self.heatmap_data = np.array(range(self.heatmap_num))

        # Declaration Publisher
        self.heatmap_pub = rospy.Publisher('my_heatmap', OccupancyGrid, queue_size=100)

        # Declaration Subscriber (message_filters)
        self.ptm_sub = message_filters.Subscriber('/people_tracker_measurements', PositionMeasurementArray)
        self.leg_sub = message_filters.Subscriber('/leg_tracker_measurements', PositionMeasurementArray)
        ts = message_filters.ApproximateTimeSynchronizer([self.ptm_sub, self.leg_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self, poe, leg):
        cell_x = 0.0
        cell_y = 0.0

        for i in range(self.heatmap_num):
            self.heatmap_data[i] = 50

        # fix x,y(m) to cells (/people_tracker_measurements)
        for poe in poe.people:
            cell_x = int((poe.pos.x - self.origin_x) / self.resolution)
            cell_y = int((poe.pos.y - self.origin_y) / self.resolution)
            print cell_x, cell_y
            for i in range(self.width):
                # self.heatmap_data[cell_x * cell_y] = 0
                self.heatmap_data[i + (self.width * (cell_y -1))] = 0


        self.heatmap_msg.data = self.heatmap_data
        self.heatmap_pub.publish(self.heatmap_msg)






if __name__ == '__main__':
    rospy.init_node('heatmap_pub_node', anonymous=True)

    Subscribe = Subscribe()

    rospy.spin()
