#!/usr/bin/env python

import rospy
import numpy as np
import math

from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray


class Publishsers():

    def prediction_msg(self, x, y, count):
        self.prediction_msg.poses[count].position.x = x
        self.prediction_msg.poses[count].position.y = y

    def optimize_make_msg(self, x, y):
        pass



class Server(Publishsers):
    def __init__(self):

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
        self.prediction_pub = rospy.Publisher("/target_human/prediction", PoseArray, queue_size = 10)

        # Declaration message
        self.prediction_msg = PoseArray()

        # Declaration Subscriber
        self.ptm_sub = rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray , self.ptm_callback)

    ### callback function for /people_tracker_measurements ###
    def ptm_callback(self, msg):
        current_x = 0.0
        current_y = 0.0
        diff_time = 0.0

        # people found
        if msg.people:
            target_name = rospy.get_param('/target_human/name')
            for i in msg.people:
                if i.name == target_name:
                    # get target human information
                    current_x = i.pos.x
                    current_y = i.pos.y
                    self.now = rospy.get_time()

                    # calculate prediction_human pose
                    current_x -= self.past_x
                    current_y -= self.past_y
                    diff_time = self.now - self.past
                    scale = 1.0 / diff_time
                    for count in range(5):
                        self.prediction_msg(current_x * scale * (count+1), current_y * scale * (count+1), count)
                    self.prediction_pub.publish(self.prediction_msg)

                    # set old data
                    self.past = self.now
                    self.past_x = current_x
                    self.past_y = current_y

        # people not found
        else:
            pass



if __name__ == '__main__':
    rospy.init_node('prediction_human')

    server = Server()

    # rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray , server.ptm_callback)
    # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, server.pose_callback)

    rospy.spin()
