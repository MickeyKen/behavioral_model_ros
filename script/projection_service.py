#!/usr/bin/env python

import rospy
import numpy as np
import math

from people_msgs.msg import PositionMeasurementArray

from std_msgs.msg import String, Bool, Int32

from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolResponse

from ubiquitous_display_pantilt.srv import AddPoints
from geometry_msgs.msg import Point

from std_msgs.msg import Float64

import time

class Server():
    def __init__(self):

        self.x_position = 4.0
        self.y_position = 1.25
        self.offset = 0.5

        self.fil_leg_msg = PositionMeasurementArray()
        self.flag = 0

        self.pan_pub = rospy.Publisher('/ubiquitous_display/pan_controller/command', Float64, queue_size=10)
        self.tilt_pub = rospy.Publisher('/ubiquitous_display/tilt_controller/command', Float64, queue_size=10)

        self.image_pub = rospy.Publisher('/ubiquitous_display/image', Int32, queue_size=10)
        self.on_off_project(0)

        # Declaration Subscriber
        self.ptm_sub = rospy.Subscriber('/filter/people_tracker_measurement', PositionMeasurementArray , self.ptm_callback)

        self.set_pantilt = rospy.ServiceProxy('action_plan_pantilt', AddPoints)

        # Declaration Service Server
        self.server = rospy.Service("/projection/service", SetBool, self.service_callback)


    ### callback function for /people_tracker_measurements ###
    def ptm_callback(self, msg):
        dt = 0.0
        diff_x = 0.0
        diff_y = 0.0

        # target people found
        if msg.people:
            self.fil_leg_msg = msg
            self.flag = 1
        # people not found
        else:
            self.flag = 0

    def service_callback(self, req):
        resp = SetBoolResponse()
        resp.message = "called"

        if self.flag == 1:
            # result.success = True
            for i in self.fil_leg_msg.people:
                if i.pos.x > self.x_position - self.offset and i.pos.x < self.x_position + self.offset:
                    if abs(i.pos.y) > self.y_position - self.offset and abs(i.pos.y) < self.y_position + self.offset:
                        response = self.set_pantilt_func(0.0, i.pos.y, 0.0)
                        print response
                        if response.success:
                            self.on_off_project(1)
                            time.sleep(0.5)
                            self.pan_pub.publish(-1.57)
                            self.tilt_pub.publish(0.0)
                            self.on_off_project(0)
                            resp.success = True
                        else:
                            resp.success = False
        elif self.flag == 0:
            pass

        return resp

    def set_pantilt_func(self, x,y,z):
        set_point = Point()

        set_point.x = x
        set_point.y = y
        set_point.z = z

        response = self.set_pantilt(set_point)

        return response

    def on_off_project(self, on_off):
        print "in"
        int_msg = Int32()
        if on_off == 1:
            int_msg = 1
        elif on_off == 0:
            int_msg = 0

        self.image_pub.publish(int_msg)





if __name__ == '__main__':
    rospy.init_node('projection_service')

    server = Server()

    rospy.spin()
