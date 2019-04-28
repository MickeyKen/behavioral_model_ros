#!/usr/bin/env python

import rospy
from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from visualization_msgs.msg import Marker

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
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.ptm_sub = rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray , self.ptm_callback)
        self.range_rviz = Marker()
        self.range_rviz.header.frame_id = "base_link"
        self.range_pub = rospy.Publisher("/range_pub", Marker, queue_size = 10)

    def pose_callback(self, msg):
        self.make_msg(msg.pose.pose.position.x,msg.pose.pose.position.y)


    def ptm_callback(self, msg):
        if msg.people:
            pass
        else:
            pass



if __name__ == '__main__':
    rospy.init_node('detect_optimazed_point_server')

    server = Server()

    # rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray , server.ptm_callback)
    # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, server.pose_callback)

    rospy.spin()
