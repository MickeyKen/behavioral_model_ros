#!/usr/bin/env python

import rospy
import numpy as np
import math

from geometry_msgs.msg import PoseArray, Pose, PoseWithCovarianceStamped, PoseStamped
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import String

from behavioral_model.srv import AddStrRetPose
from behavioral_model.srv import AddStrRetPoseResponse

from nav_msgs.msg import OccupancyGrid


class Decide():
    def decide(self):
        RetPosition = Point()
        RetOrientation = Quaternion()

        ud_x = self.amcl_pose[0][0]
        ud_y = self.amcl_pose[0][1]
        rad = math.radians(90)

        for j in range(5):
            RetPosition.x = ((self.human_pose[j][0] - self.projection_pose[j][0]) * math.cos(rad))-((self.human_pose[j][1] - self.projection_pose[j][1]) * math.sin(rad))+self.projection_pose[j][0]
            RetPosition.y = ((self.human_pose[j][0] - self.projection_pose[j][0]) * math.sin(rad))+((self.human_pose[j][1] - self.projection_pose[j][1]) * math.cos(rad))+self.projection_pose[j][1]

            d = math.sqrt(((RetPosition.x - self.amcl_pose[0][0]) ** 2) + ((RetPosition.y - self.amcl_pose[0][1]) ** 2))
            if d < (j + 1) * 0.5:
                self.make_opt_pub(RetPosition.x, RetPosition.y)
                RetOrientation.w = 1.0
                break

        return RetPosition, RetOrientation

    def make_opt_pub(self, x, y):
        self.opt_msg.position.x = x
        self.opt_msg.position.y = y
        self.opt_msg.orientation.w = 1.0
        self.opt_pub.publish(self.opt_msg)




class Subscribe(Decide):
    def __init__(self):

        self.amcl_pose = np.zeros((1,2))
        self.human_pose = np.zeros((5,2))
        self.projection_pose = np.zeros((5,2))

        self.heat_msg = OccupancyGrid()
        self.heat_msg.header.frame_id = '/map'
        # msg.header.stamp = rospy.Time.now()

        self.heat_msg.info.resolution = 0.05
        self.heat_msg.info.width = 100
        self.heat_msg.info.height = 100

        self.opt_msg = Pose()

        # Declaration Publisher
        self.occ_pub = rospy.Publisher('/heat_map', OccupancyGrid, queue_size=100)
        self.opt_pub = rospy.Publisher('/ud/optimize/pose', Pose, queue_size=10)


        # Declaration Subscriber
        self.robot_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.robot_pose_callback)
        self.pose_sub = rospy.Subscriber('/target_human/prediction/pose', PoseArray, self.pose_callback)
        self.projection_pose_sub = rospy.Subscriber('/target_human/prediction/projection/pose', PoseArray, self.projection_pose_callback)

        # Declaration Service Server
        self.server = rospy.Service("/detection/optimize_point", AddStrRetPose, self.service_callback)

    ### callback function for amcl node (pose) ###
    def robot_pose_callback(self, msg):
        self.amcl_pose[0][0] = msg.pose.pose.position.x
        self.amcl_pose[0][1] = msg.pose.pose.position.y


    ### callback function for amcl node (pose) ###
    def pose_callback(self, msg):
        for i in range(len(msg.poses)):
            if i > 0:
                self.human_pose[i-1][0] = msg.poses[i-1].position.x
                self.human_pose[i-1][1] = msg.poses[i-1].position.y

    ### callback function for /people_tracker_measurements ###
    def projection_pose_callback(self, msg):
        for i in range(len(msg.poses)):
            self.projection_pose[i][0] = msg.poses[i].position.x
            self.projection_pose[i][1] = msg.poses[i].position.y


    def service_callback(self, req):
        return_pose = PoseStamped()

        position, orientation = self.decide()

        return_pose.pose.position = position
        return_pose.pose.orientation = orientation
        return AddStrRetPoseResponse(return_pose)




if __name__ == '__main__':
    rospy.init_node('detect_optimize_point_server')

    Subscribe = Subscribe()

    # rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray , server.ptm_callback)
    # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, server.pose_callback)

    rospy.spin()
