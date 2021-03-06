#!/usr/bin/env python

import rospy
import math
import time
import tf
from ubiquitous_display_pantilt.srv import AddPoints
from ubiquitous_display_pantilt.srv import AddPointsResponse
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

MAX_PAN_RADIAN = 1.24 #71
MIN_PAN_RADIAN = 3.8  #217

MAX_TILT_RADIAN = 1.04 # 2.96
MIN_TILT_RADIAN = 0.52 #29

class Publishsers():

    def pan_make(self, pan):
        pan_msg = Float64()

        pan_msg.data = pan

        self.pan_pub.publish(pan_msg)

    def tilt_make(self, tilt):
        tilt_msg = Float64()

        tilt_msg.data = tilt

        self.tilt_pub.publish(tilt_msg)

class Server(Publishsers):
    def __init__(self):
        self.pan_angle = 0.0
        self.tilt_angle = 0.0

        self.pan_pub = rospy.Publisher('/ubiquitous_display/pan_controller/command', Float64, queue_size=10)
        self.tilt_pub = rospy.Publisher('/ubiquitous_display/tilt_controller/command', Float64, queue_size=10)

        self.jsp_sub = rospy.Subscriber('/ubiquitous_display/joint_states', JointState, self.jsp_callback)
        self.server = rospy.Service('action_plan_pantilt', AddPoints, self.callback)

    ### check limit radian for pan and tilt ###
    def checkPanRadian(self, radian):

        radian = self.constrain(radian, -MIN_PAN_RADIAN, MAX_PAN_RADIAN)

        return radian

    def checkTiltRadian(self, radian):

        radian = self.constrain(radian, -MIN_TILT_RADIAN, MAX_TILT_RADIAN)

        return radian

    def constrain(self, input, low, high):
        if input < low:
          input = low
        elif input > high:
          input = high
        else:
          input = input

        return input

    ### check tf (joint_states) for pan and tilt ###
    def jsp_callback(self, msg):
        self.pan_angle = msg.position[4]
        self.tilt_angle = msg.position[5]

    def callback(self, point):

        x_point = point.position.x
        y_point = point.position.y
        z_point = point.position.z

        rad_pan = math.atan2(y_point, x_point)
        #print math.degrees(rad_pan)

        distance = math.sqrt(x_point*x_point + y_point*y_point)
        #print distance

        ud_z_point = 1.21

        rad_tilt = math.atan2(ud_z_point, distance)
        #print math.degrees(rad_tilt)

        float_pan = Float64()
        float_tilt = Float64()

        float_pan = rad_pan - 1.5708
        float_pan = self.checkPanRadian(float_pan)
        target_pan = float_pan

        float_tilt = rad_tilt
        float_tilt = self.checkTiltRadian(float_tilt)
        target_tilt = float_tilt

        self.pan_make(float_pan)
        self.tilt_make(float_tilt)

        offset_tilt = 0.04 #2.5 degree
        offset_pan = 0.04
        while True:
          # print self.pan_angle, self.tilt_angle
          current_tilt = self.tilt_angle
          current_pan = self.pan_angle
          if current_tilt < target_tilt + offset_tilt and current_tilt > target_tilt - offset_tilt and current_pan < target_pan + offset_pan and current_pan > target_pan - offset_pan:
            # point.success = True
            break

        return AddPointsResponse(True)


if __name__ == '__main__':
    rospy.init_node('simple_service_server_for_pantilt', anonymous=True)

    server = Server()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
