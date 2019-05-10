#!/usr/bin/env python

import rospy
import numpy as np
import math

from people_msgs.msg import PositionMeasurementArray, PositionMeasurement
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from std_msgs.msg import String
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Pose

import message_filters


class Publishsers():
    def pub_msg(self, x, y):
        self.pose_msg.position.x = x
        self.pose_msg.position.y = y
        self.pose_pub.publish(self.pose_msg)

class Subscribe(Publishsers):
    def __init__(self):
        # get map_data
        self.map_service = rospy.ServiceProxy('static_map', GetMap)
        self.map_data = self.map_service()

        # Marker Array for rviz
        self.pose_msg = Pose()

        # message for result topic
        self.result = String()

        # numpy array for person xy
        self.people1 = np.zeros((100,2))

        # ubiquitous display current pose
        self.amcl_pose_x = 0.0
        self.amcl_pose_y = 0.0

        # Declaration Publisher
        self.pose_pub = rospy.Publisher("/target_human/pose", Pose, queue_size = 10)
        self.result_pub = rospy.Publisher("/ptm/server/result", String, queue_size = 10)

        # Declaration Subscriber
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.ptm_sub = message_filters.Subscriber('/people_tracker_measurements', PositionMeasurementArray)
        self.leg_sub = message_filters.Subscriber('/leg_tracker_measurements', PositionMeasurementArray)

        ts = message_filters.ApproximateTimeSynchronizer([self.ptm_sub, self.leg_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)


    ### callback function for amcl node (pose) ###
    def pose_callback(self, msg):
        self.amcl_pose_x = msg.pose.pose.position.x
        self.amcl_pose_y = msg.pose.pose.position.y


    ### callback function for /people_tracker_measurements ###
    def callback(self, msg, leg):
        ud_person_distance = 5.0
        x = 1000.0
        y = 1000.0
        min_person = PositionMeasurement()
        person_name = "nohuman"
        leg_name = ""
        self.result.data = person_name
        both_leg_counter = 0

        # people found
        if msg.people:

            # search minimum distance between US and target_human
            for i in msg.people:
                d = math.sqrt(((i.pos.x - self.amcl_pose_x) ** 2) + ((i.pos.y - self.amcl_pose_y) ** 2))
                if d < ud_person_distance:
                    # person_name = i.name
                    # x = i.pos.x
                    # y = i.pos.y
                    ud_person_distance = d
                    leg_name = i.object_id

                    # check leg_position comparing static_map
                    if leg_name != "":
                        # split leg name leg_list[0],leg_list[1]
                        leg_list = leg_name.split('|')
                        for j in leg.people:
                            if j.object_id == leg_list[0] or j.object_id == leg_list[1]:
                                # print "pass"
                                # print self.map_data.map.info.width
                                # print self.map_data.map.info.height
                                # print ((int((abs(j.pos.y) / self.map_data.map.info.resolution)-1) * self.map_data.map.info.width)+ int(abs(j.pos.x) / self.map_data.map.info.resolution))
                                # check static map
                                map_occ = ((int((j.pos.y / self.map_data.map.info.resolution)-1) * self.map_data.map.info.width)+ int(j.pos.x / self.map_data.map.info.resolution)+1900+(4000*1899))
                                if (self.map_data.map.data[map_occ] == 0):
                                    both_leg_counter += 1
                                else:
                                    pass
                    if both_leg_counter == 2:
                        min_person = i
                    #initiala counter
                    both_leg_counter = 0

                if min_person.name != "":
                    self.result.data = min_person.name
                    #publish target_human_pose for rviz
                    self.pub_msg(min_person.pos.x, min_person.pos.y)


        # people not found
        else:
            self.result.data = "nohuman"

        self.result_pub.publish(self.result)



if __name__ == '__main__':
    rospy.init_node('detect_person_server')

    Subscribe = Subscribe()

    # rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray , server.ptm_callback)
    # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, server.pose_callback)

    rospy.spin()
