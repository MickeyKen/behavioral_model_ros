#!/usr/bin/env python

import rospy
import numpy as np
import math

from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose
from behavioral_model.srv import AddPoseRetStr
from behavioral_model.srv import AddPoseRetStrResponse

class Publishsers():

    def prediction_make(self, x, y):
        pose_msg = Pose()

        pose_msg.position.x = x
        pose_msg.position.y = y
        pose_msg.position.z = 0.0

        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 0.0
        pose_msg.orientation.w = 1.0

        self.prediction_msg.poses.append(pose_msg)

    def optimize_make_msg(self, x, y):
        pass



class Server(Publishsers):
    def __init__(self):

        rospy.set_param('/target_human/name', "")

        # message for result topic
        self.result = String()

        # numpy array for person xy
        self.people1 = np.zeros((100,2))
        self.past_x = 0.0
        self.past_y = 0.0

        # get current time
        self.now = rospy.get_time()
        self.past = rospy.get_time()

        # Declaration Publisher
        # self.trajectory_pub = rospy.Publisher("/trajectory_pub", Marker, queue_size = 10)
        self.prediction_pub = rospy.Publisher("/target_human/prediction/pose", PoseArray, queue_size = 10)

        # Declaration message
        self.prediction_msg = PoseArray()

        # Declaration Subscriber
        self.ptm_sub = rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray , self.ptm_callback)

        # Declaration Service Server
        self.server = rospy.Service("/prediction/target_human", AddPoseRetStr, self.service_callback)


    ### callback function for /people_tracker_measurements ###
    def ptm_callback(self, msg):
        current_x = 0.0
        current_y = 0.0
        diff_time = 0.0
        diff_x = 0.0
        diff_y = 0.0

        # people found
        if msg.people:
            target_name = rospy.get_param('/target_human/name')
            for i in msg.people:
                if i.name == target_name:
                    # get target human information
                    current_x = i.pos.x
                    current_y = i.pos.y
                    self.now = rospy.get_time()

                    # calculate prediction_human pose and time
                    diff_x = current_x - self.past_x
                    diff_y = current_y - self.past_y
                    diff_time = self.now - self.past

                    if diff_time != 0.0:
                        scale = 1.0 / diff_time

                        # init PoseArray() and create and publish
                        self.prediction_msg = PoseArray()
                        for count in range(5):
                            self.prediction_make((diff_x * scale * (count+1)) + current_x, (diff_y * scale * (count+1)) + current_y)
                        self.prediction_msg.header.stamp = rospy.Time.now()
                        self.prediction_msg.header.frame_id = "/base_scan"
                        self.prediction_pub.publish(self.prediction_msg)

                        # set old data
                        self.past = self.now
                        self.past_x = current_x
                        self.past_y = current_y

        # people not found
        else:
            pass

    def service_callback(self, req):
        return_string = String()
        print req.target_pose.pose.position.x

        return_string.data = "true"
        return AddPoseRetStrResponse(return_string)


if __name__ == '__main__':
    rospy.init_node('prediction_human')

    server = Server()

    # rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray , server.ptm_callback)
    # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, server.pose_callback)

    rospy.spin()
