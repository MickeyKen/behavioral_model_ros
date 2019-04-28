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
        ### get map_data ###
        self.map_service = rospy.ServiceProxy('static_map', GetMap)
        self.map_data = self.map_service()
        self.range_rviz = Marker()
        self.range_rviz.header.frame_id = "base_link"
        self.range_pub = rospy.Publisher("/range_pub", Marker, queue_size = 10)
        self.result_pub = rospy.Publisher("/ptm/server/result", String, queue_size = 10)
        self.result = String()
        self.people1 = np.zeros((100,2))
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.ptm_sub = rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray , self.ptm_callback)


    def pose_callback(self, msg):
        pass
        # self.make_msg(msg.pose.pose.position.x, msg.pose.pose.position.y)


    def ptm_callback(self, msg):
        x = 0
        y = 0
        if msg.people:
            for i in msg.people:
                if i.name == "Person0":
                    x = i.pos.x
                    y = i.pos.y
                    break

            if (self.map_data.map.data[ int(y / self.map_data.map.info.resolution) * self.map_data.map.info.width + int(x / self.map_data.map.info.resolution)] != 100):
                self.people1 = np.append(self.people1, np.array([[x, y]]), axis=0)
                self.result = "human"
                self.result_pub.publish(self.result)
        else:
            pass



if __name__ == '__main__':
    rospy.init_node('detect_optimazed_point_server')

    server = Server()

    # rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray , server.ptm_callback)
    # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, server.pose_callback)

    rospy.spin()
