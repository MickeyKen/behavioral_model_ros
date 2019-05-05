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

    def filter(self):
        u = np.zeros((1,4))
        F = np.eye(4)
        P = np.array([[0., 0., 0., 0.],[0., 0., 0., 0.],[0., 0., 1000., 0.],[0., 0., 0., 1000.]])
        H = np.array([[1., 0., 0., 0.],[0., 1., 0., 0.]])
        R = np.array([[0.1, 0.],[0., 0.1]])
        I = np.eye(4)
        for n in range(len(self.measurements)):

            F[0][2] = self.dt[n]
            F[1][3] = self.dt[n]

            # prediction
            self.x = (F * self.x) + u
            P = F * P * F.T

            # measurement update
            Z = matrix([self.measurements[n]])
            y = Z.T - (H * self.x)
            S = H * P * H.T + R
            K = P * H.T * np.linalg.det(S)
            self.x = self.x + (K * y)
            P = (I - (K * H)) * P

        return self.x, P



class Server(Publishsers):
    def __init__(self):

        rospy.set_param('/target_human/name', "")

        # message for result topic
        self.result = String()

        # numpy array for person xy
        self.measurements = np.empty((100,2))
        self.dtArr = np.empty(100)
        self.past_x = 0.0
        self.past_y = 0.0

        # for kalman filter
        self.x = np.zeros((1,4))
        # self.u = np.zeros((1,4))
        # self.P = np.array([[0., 0., 0., 0.],[0., 0., 0., 0.],[0., 0., 1000., 0.],[0., 0., 0., 1000.]])
        # self.F = np.eye(4)
        # self.H = np.array([[1., 0., 0., 0.],[0., 1., 0., 0.]])
        # self.R = np.array([[0.1, 0.],[0., 0.1]])
        # self.I = np.eye(4)

        # prepare x,y for initial_pose(self.x)
        self.current_x = 0.0
        self.current_y = 0.0


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
        dt = 0.0
        diff_x = 0.0
        diff_y = 0.0

        # people found
        if msg.people:
            target_name = rospy.get_param('/target_human/name')
            for i in msg.people:
                if i.name == target_name:
                    if len(self.measurements) == 0:
                        self.x[0][0] = i.pos.x
                        self.y[1][0] = i.pos.y
                    else:
                        self.now = rospy.get_time()
                        dt = self.now - self.past
                        np.append(self.measurements, [[i.pos.x,i.pos.y]], axis=0)
                        np.append(self.dtArr, [dt], axis=0)


        # people not found
        else:
            pass

    def service_callback(self, req):
        return_string = String()
        print req.target_pose.pose.position.x

        self.x = np.zeros((1,4))
        self.measurements = np.empty((0,2))
        self.dtArr = np.empty(0)

        rospy.sleep(1.0)

        x, P = self.filter()
        print x
        print P


        # init PoseArray() and create and publish
        # self.prediction_msg = PoseArray()
        # for i in pose:
        #     self.prediction_make(pose[i]][0],pose[i][1])
        # self.prediction_msg.header.stamp = rospy.Time.now()
        # self.prediction_msg.header.frame_id = "/base_scan"
        # self.prediction_pub.publish(self.prediction_msg)


        return_string.data = "true"
        return AddPoseRetStrResponse(return_string)


if __name__ == '__main__':
    rospy.init_node('prediction_human')

    server = Server()

    # rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray , server.ptm_callback)
    # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, server.pose_callback)

    rospy.spin()
