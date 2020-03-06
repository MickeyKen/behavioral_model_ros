#!/usr/bin/python
import rospy

import numpy as np

import math

from people_msgs.msg import PositionMeasurementArray
from people_msgs.msg import PositionMeasurement

from nav_msgs.srv import GetMap

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int8

import message_filters


class Subscribe():
    def __init__(self):

        # hennsuu for calculate
        self.projection_distance = 50 # 2.5[m] / 0.05[resolution]
        self.personal_space_min = 15 # 0.75[m] / 0.05[resolution]
        self.personal_space_max = 40 # 2.0[m] / 0.05[resolution]
        self.limited_distance_min = 15 # 1.5[m] / 0.05[resolution]
        self.limited_distance_max = 40 # 4.0[m] / 0.05[resolution]
        self.angle_min = 30
        self.angle_max = 60



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

        self.flag = 0

        # Declaration Publisher
        self.heatmap_pub = rospy.Publisher('my_heatmap', OccupancyGrid, queue_size=100)

        # Declaration Subscriber (message_filters)
        self.ptm_sub = rospy.Subscriber('/filter/people_tracker_measurement', PositionMeasurementArray, self.callback)

    def callback(self, poe):
        # print poe
        cell_x = 0.0
        cell_y = 0.0
        occ = 0
        occ_x = 0
        occ_y = 0

        if self.flag == 0:

            for i in range(self.heatmap_num):
                self.heatmap_data[i] = -1

            evaluate_array = np.copy(self.heatmap_data)

            value_grid = np.zeros((self.heatmap_num, 4))
            # print value_grid

            # fix x,y(m) to cells (/people_tracker_measurements)
            for poe in poe.people:
                # get angle default 45?
                target_angle = 270 - 45
                cell_x = int((poe.pos.x - self.origin_x) / self.resolution)
                cell_y = int((poe.pos.y - self.origin_y) / self.resolution)
                occ = cell_x + (self.width * (cell_y-1))

                #calculate projection point (need reversion)
                projection_cell_x = int((poe.pos.x - self.origin_x) / self.resolution)
                projection_cell_y = int((poe.pos.y - self.origin_y - 2.5) / self.resolution)

                #calculate angle usgin target_angle
                view_cell_x = int((poe.pos.x - self.origin_x) / self.resolution)
                view_cell_y = int((poe.pos.y - self.origin_y - 5.0) / self.resolution)


                ### personal_space
                for ang in np.arange(float(-60), float(60),0.1):
                    rad = math.radians(target_angle + ang)
                    for c in range(self.personal_space_min, self.personal_space_max):
                        occ_x = int((c * math.cos(rad)) - (c * math.sin(rad))) + cell_x
                        occ_y = int((c * math.sin(rad)) + (c * math.cos(rad))) + cell_y
                        value_grid[occ_x + (self.width * (occ_y-1))][0] = 1
                        # evaluate_array[occ_x + (self.width * (occ_y-1))] = 30

                ### limited_space
                for dang in np.arange(float(0), float(360),0.1):
                    rad = math.radians(dang)
                    for d in range(self.limited_distance_min, self.limited_distance_max):
                        occ_x = int((d * math.cos(rad)) - (d * math.sin(rad))) + projection_cell_x
                        occ_y = int((d * math.sin(rad)) + (d * math.cos(rad))) + projection_cell_y
                        value_grid[occ_x + (self.width * (occ_y-1))][1] = 1
                        # evaluate_array[occ_x + (self.width * (occ_y-1))] = 60

                ### eye view ###
                for eang in np.arange(float(15 + target_angle),float(30 + target_angle),0.1):
                    rad = math.radians(eang)
                    for e in range(100):
                        occ_x = int((e * math.cos(rad)) - (e * math.sin(rad))) + cell_x
                        occ_y = int((e * math.sin(rad)) + (e * math.cos(rad))) + cell_y
                        value_grid[occ_x + (self.width * (occ_y-1))][2] = 1

                for eang in np.arange(float(330 + target_angle),float(345 + target_angle),0.1):
                    rad = math.radians(eang)
                    for e in range(100):
                        occ_x = int((e * math.cos(rad)) - (e * math.sin(rad))) + cell_x
                        occ_y = int((e * math.sin(rad)) + (e * math.cos(rad))) + cell_y
                        value_grid[occ_x + (self.width * (occ_y-1))][3] = 1


            for num in range(self.heatmap_num):
                # if value_grid[num][2] == 1:
                self.heatmap_data[num] = value_grid[num][0] * 30 + value_grid[num][1] * 30 + value_grid[num][2] * 30 + value_grid[num][3] * 30

            self.heatmap_msg.data = self.heatmap_data
            self.heatmap_pub.publish(self.heatmap_msg)
            self.flag += 1






if __name__ == '__main__':
    rospy.init_node('heatmap_pub_node_2', anonymous=True)

    Subscribe = Subscribe()

    rospy.spin()
