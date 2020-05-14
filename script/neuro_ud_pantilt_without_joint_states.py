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

MAX_PAN_RADIAN = 2.9670
MIN_PAN_RADIAN = 2.9670

MAX_TILT_RADIAN = 1.3
MIN_TILT_RADIAN = 0.2617

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

        self.init_pan_ang = -(math.pi / 2.0)
        self.init_tilt_ang = 0.0

        self.pan_pub = rospy.Publisher('/ubiquitous_display/pan_controller/command', Float64, queue_size=10)
        self.tilt_pub = rospy.Publisher('/ubiquitous_display/tilt_controller/command', Float64, queue_size=10)

        self.server = rospy.Service('neuro/action_plan_pantilt', AddPoints, self.callback)

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

    def callback(self, point):

        pan_ang = point.position.x
        tilt_ang = point.position.y

        diff_pan_ang = abs(pan_ang - self.init_pan_ang)
        time_pan = diff_pan_ang * 1.1
        # print diff_pan_ang, time_pan
        diff_tilt_ang = abs(tilt_ang - self.init_tilt_ang)
        time_tilt = diff_tilt_ang * 1.08

        if time_pan > time_tilt:
            target_time = time_pan
        else:
            target_time = time_tilt

        float_pan = Float64()
        float_tilt = Float64()

        float_pan = pan_ang
        float_tilt = tilt_ang

        self.pan_make(float_pan)
        self.tilt_make(float_tilt)

        start = time.time()
        while True:
          elapsed_time = time.time() - start
          if elapsed_time >= target_time:
              # print elapsed_time
              break

        return AddPointsResponse(True)


if __name__ == '__main__':
    rospy.init_node('neuro_pantilt_server_without_joint_states', anonymous=True)

    server = Server()

    rospy.spin()
