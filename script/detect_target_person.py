#!/usr/bin/env python

import rospy
import numpy as np
import math

from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from std_msgs.msg import String
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Pose


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
        self.ptm_sub = rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray , self.ptm_callback)


    ### callback function for amcl node (pose) ###
    def pose_callback(self, msg):
        self.amcl_pose_x = msg.pose.pose.position.x
        self.amcl_pose_y = msg.pose.pose.position.y


    ### callback function for /people_tracker_measurements ###
    def ptm_callback(self, msg):
        ud_person_distance = 5.0
        x = 0
        y = 0
        person_name = "nohuman"

        # people found
        if msg.people:

            # search minimum distance between US and target_human
            for i in msg.people:
                d = math.sqrt(((i.pos.x - self.amcl_pose_x) ** 2) + ((i.pos.y - self.amcl_pose_y) ** 2))
                if d < ud_person_distance:
                    person_name = i.name
                    x = i.pos.x
                    y = i.pos.y
                    ud_person_distance = d
                    # rospy.set_param('/target_person_name', person_name)
            # print x, y

            # check static map
            if (self.map_data.map.data[ int(y / self.map_data.map.info.resolution) * self.map_data.map.info.width + int(x / self.map_data.map.info.resolution)] != 100):
                #publish target_human_pose for rviz
                self.pub_msg(i.pos.x, i.pos.y)
                #publish person_name
                self.result.data = person_name
                self.result_pub.publish(self.result)
            else:
                self.result.data = "nohuman"
                self.result_pub.publish(self.result)

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
