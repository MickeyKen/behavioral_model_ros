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

        for i in range(self.heatmap_num):
            self.heatmap_data[i] = -1

        evaluate_array = np.copy(self.heatmap_data)

        # fix x,y(m) to cells (/people_tracker_measurements)
        for poe in poe.people:
            # get angle default 45?
            target_angle = 270 - 45
            cell_x = int((poe.pos.x - self.origin_x) / self.resolution)
            cell_y = int((poe.pos.y - self.origin_y) / self.resolution)
            occ = cell_x + (self.width * (cell_y-1))

            #calculate angle usgin target_angle
            projection_cell_x = int((poe.pos.x - self.origin_x) / self.resolution)
            projection_cell_y = int((poe.pos.y - self.origin_y - 2.5) / self.resolution)

            #calculate angle usgin target_angle
            view_cell_x = int((poe.pos.x - self.origin_x) / self.resolution)
            view_cell_y = int((poe.pos.y - self.origin_y - 5.0) / self.resolution)


            ### personal_space
            for ang in range(-60, 60):
                rad = math.radians(target_angle + ang)
                for c in range(self.personal_space_min, self.personal_space_max):
                    occ_x = int((c * math.cos(rad)) - (c * math.sin(rad))) + cell_x
                    occ_y = int((c * math.sin(rad)) + (c * math.cos(rad))) + cell_y
                    self.heatmap_data[occ_x + (self.width * (occ_y-1))] = 30
                    # evaluate_array[occ_x + (self.width * (occ_y-1))] = 30

            ### limited_space
            for dang in range(0, 360):
                rad = math.radians(dang)
                for d in range(self.limited_distance_min, self.limited_distance_max):
                    occ_x = int((d * math.cos(rad)) - (d * math.sin(rad))) + projection_cell_x
                    occ_y = int((d * math.sin(rad)) + (d * math.cos(rad))) + projection_cell_y
                    if self.heatmap_data[occ_x + (self.width * (occ_y-1))] == 30:
                        self.heatmap_data[occ_x + (self.width * (occ_y-1))] = 60
                    else:
                        self.heatmap_data[occ_x + (self.width * (occ_y-1))] = 20
                    # evaluate_array[occ_x + (self.width * (occ_y-1))] = 60

            ### eye view ###
            for eang in range(15 + target_angle,30 + target_angle):
                rad = math.radians(eang)
                for e in range(100):
                    occ_x = int((e * math.cos(rad)) - (e * math.sin(rad))) + cell_x
                    occ_y = int((e * math.sin(rad)) + (e * math.cos(rad))) + cell_y
                    if self.heatmap_data[occ_x + (self.width * (occ_y-1))] == 60:
                        self.heatmap_data[occ_x + (self.width * (occ_y-1))] = 95
                    elif self.heatmap_data[occ_x + (self.width * (occ_y-1))] == 30:
                        self.heatmap_data[occ_x + (self.width * (occ_y-1))] = 60
                    elif self.heatmap_data[occ_x + (self.width * (occ_y-1))] == 20:
                        self.heatmap_data[occ_x + (self.width * (occ_y-1))] = 30
                    else:
                        self.heatmap_data[occ_x + (self.width * (occ_y-1))] = 10

            for eang in range(330 + target_angle,345 + target_angle):
                rad = math.radians(eang)
                for e in range(100):
                    occ_x = int((e * math.cos(rad)) - (e * math.sin(rad))) + cell_x
                    occ_y = int((e * math.sin(rad)) + (e * math.cos(rad))) + cell_y
                    if self.heatmap_data[occ_x + (self.width * (occ_y-1))] == 60:
                        self.heatmap_data[occ_x + (self.width * (occ_y-1))] = 95
                    elif self.heatmap_data[occ_x + (self.width * (occ_y-1))] == 30:
                        self.heatmap_data[occ_x + (self.width * (occ_y-1))] = 60
                    elif self.heatmap_data[occ_x + (self.width * (occ_y-1))] == 20:
                        self.heatmap_data[occ_x + (self.width * (occ_y-1))] = 30
                    else:
                        self.heatmap_data[occ_x + (self.width * (occ_y-1))] = 10






            ### for debug ###
            # for i in range(self.width):
            #     self.heatmap_data[i + (self.width * (cell_y-1))] = 0
            # for j in range(self.height):
            #     self.heatmap_data[cell_x + (j * self.width)] = 0


        self.heatmap_msg.data = self.heatmap_data
        self.heatmap_pub.publish(self.heatmap_msg)






if __name__ == '__main__':
    rospy.init_node('heatmap_pub_node', anonymous=True)

    Subscribe = Subscribe()

    rospy.spin()
