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
        # print self.measurements
        # print self.x
        for n in range(len(self.measurements)):

            if self.dtArr[n] > 0.03 and self.dtArr[n] < 0.045:
                F[0][2] = self.dtArr[n]
                F[1][3] = self.dtArr[n]
                # print self.dtArr[n]

                # prediction
                # self.x = (F * self.x) + u
                self.x = np.dot(F, self.x) + u
                # P = F * P * F.T
                P = np.dot(np.dot(F, P), F.T)



                # measurement update
                Z = np.array([self.measurements[n]])
                # y = Z.T - (H * self.x)
                y = Z.T - np.dot(H, self.x)

                # S = H * P * H.T + R
                S = np.dot(np.dot(H, P), H.T) + R

                # K = P * H.T * np.linalg.inv(S)
                K = np.dot(np.dot(P, H.T), np.linalg.inv(S))

                # self.x = self.x + (K * y)
                self.x = self.x + np.dot(K, y)

                # P = (I - (K * H)) * P
                P = np.dot((I - np.dot(K, H)), P)

            # print self.x[0][0]

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
        # self.x = np.zeros((1,4))
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
                    if self.x[0][0] == 0.0 and self.x[1][0] == 0.0:
                        self.x[0][0] = i.pos.x
                        self.x[1][0] = i.pos.y
                        self.now = rospy.get_time()
                    else:
                        # print "pass"
                        self.now = rospy.get_time()
                        dt = self.now - self.past
                        self.measurements = np.append(self.measurements, [[i.pos.x,i.pos.y]], axis=0)
                        self.dtArr = np.append(self.dtArr, [dt], axis=0)
                        self.past = self.now


        # people not found
        else:
            pass

    def service_callback(self, req):
        return_string = String()
        # print req.target_pose.pose.position.x

        # self.x = np.zeros((4,1))
        self.x = np.array([[0.], [0.], [0.], [0.]])
        self.measurements = np.empty((0,2))
        self.dtArr = np.empty(0)

        # rospy.spin()
        rospy.sleep(0.7)

        x, P = self.filter()

        # init PoseArray() and create and publish
        self.prediction_msg = PoseArray()
        for i in range(5):
            self.prediction_make(x[0][0] + x[2][0] * (i + 1), x[1][0] + x[3][0] * (i + 1))
        self.prediction_msg.header.stamp = rospy.Time.now()
        self.prediction_msg.header.frame_id = "/map"
        self.prediction_pub.publish(self.prediction_msg)


        return_string.data = "true"
        return AddPoseRetStrResponse(return_string)


if __name__ == '__main__':
    rospy.init_node('prediction_human')

    server = Server()

    # rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray , server.ptm_callback)
    # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, server.pose_callback)

    rospy.spin()
