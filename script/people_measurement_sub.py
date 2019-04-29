#!/usr/bin/env python

import rospy
import numpy as np
import math

from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from nav_msgs.srv import GetMap


class Publishsers():
    def make_msg(self, x, y):
        self.range_rviz.header.stamp = rospy.Time.now()
        self.range_rviz.ns = "basic_shapes"
        self.range_rviz.id = 0

        self.range_rviz.action = Marker.ADD

        self.range_rviz.pose.position.x = 0
        self.range_rviz.pose.position.y = 0
        self.range_rviz.pose.position.z = 0.15

        self.range_rviz.pose.orientation.x=0.0
        self.range_rviz.pose.orientation.y=0.0
        self.range_rviz.pose.orientation.z=0.0
        self.range_rviz.pose.orientation.w=0.0

        self.range_rviz.color.r = 0.0
        self.range_rviz.color.g = 1.0
        self.range_rviz.color.b = 0.0
        self.range_rviz.color.a = 0.5

        self.range_rviz.scale.x = 5
        self.range_rviz.scale.y = 5
        self.range_rviz.scale.z = 0.1

        self.range_rviz.lifetime = rospy.Duration()

        self.range_rviz.type = 3
        self.range_pub.publish(self.range_rviz)

class Server(Publishsers):
    def __init__(self):
        # get map_data
        self.map_service = rospy.ServiceProxy('static_map', GetMap)
        self.map_data = self.map_service()

        # Marker Array for rviz
        self.range_rviz = Marker()
        self.range_rviz.header.frame_id = "base_link"

        # message for result topic
        self.result = String()

        # numpy array for person xy
        self.people1 = np.zeros((100,2))

        # ubiquitous display current pose
        self.amcl_pose_x = 0.0
        self.amcl_pose_y = 0.0

        # Declaration Publisher
        self.range_pub = rospy.Publisher("/range_pub", Marker, queue_size = 10)
        self.result_pub = rospy.Publisher("/ptm/server/result", String, queue_size = 10)

        # Declaration Subscriber
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.ptm_sub = rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray , self.ptm_callback)


    ### callback function for amcl node (pose) ###
    def pose_callback(self, msg):
        self.amcl_pose_x = msg.pose.pose.position.x
        self.amcl_pose_y = msg.pose.pose.position.y
        # self.make_msg(msg.pose.pose.position.x, msg.pose.pose.position.y)


    ### callback function for /people_tracker_measurements ###
    def ptm_callback(self, msg):
        ud_person_distance = 5.0
        x = 0
        y = 0
        person_name = ""

        # people found
        if msg.people:
            for i in msg.people:
                d = math.sqrt((i.pos.x - self.amcl_pose_x) ** 2 + (i.pos.y - self.amcl_pose_y) ** 2)
                if d < ud_person_distance:
                    person_name = i.name
                    x = i.pos.x
                    y = i.pos.y
                    # rospy.set_param('/target_person_name', person_name)
            # print x, y

            if (self.map_data.map.data[ int(y / self.map_data.map.info.resolution) * self.map_data.map.info.width + int(x / self.map_data.map.info.resolution)] != 100):
                self.people1 = np.append(self.people1, np.array([[x, y]]), axis=0)
                # print self.people1
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
    rospy.init_node('detect_optimazed_point_server')

    server = Server()

    # rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray , server.ptm_callback)
    # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, server.pose_callback)

    rospy.spin()
