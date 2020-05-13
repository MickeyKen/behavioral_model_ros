#!/usr/bin/env python

import rospy
import numpy as np
import math
import time
import tf

from std_msgs.msg import String, Bool, Int32, Float64
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import LaserScan

from std_srvs.srv import SetBool, SetBoolResponse
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from ubiquitous_display_pantilt.srv import AddPoints, AddPointsRequest

MAX_PAN_RADIAN = 2.9670
MIN_PAN_RADIAN = -2.9670

MAX_TILT_RADIAN = 1.3
MIN_TILT_RADIAN = -0.2617

class Server():
    def __init__(self):

        self.x_position = 4.0
        self.y_position = 1.25
        self.offset = 0.5

        human_number = 5
        self.name_list = []
        for i in range(human_number):
            self.name_list.append("actor" + str(i))

        self.scan_flag = 0

        self.pan_pub = rospy.Publisher('/ubiquitous_display/pan_controller/command', Float64, queue_size=10)
        self.tilt_pub = rospy.Publisher('/ubiquitous_display/tilt_controller/command', Float64, queue_size=10)

        self.image_pub = rospy.Publisher('/ubiquitous_display/image', Int32, queue_size=10)

        self.set_pantilt = rospy.ServiceProxy('neuro/action_plan_pantilt', AddPoints)

        # Declaration Service Server
        self.server = rospy.Service("/projection/service", SetBool, self.service_callback)

        self.call = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.ptm_sub = rospy.Subscriber('/front_laser_scan', LaserScan , self.scan_callback)

        rospy.sleep(2)
        self.image_pub.publish(0)
        self.pan_pub.publish(-(math.pi / 2.0))
        rospy.loginfo("--- Start projection server ---")

    def service_callback(self, req):
        resp = SetBoolResponse()
        resp.message = "called"
        resp.success = False

        ud_pose = self.get_pose("ubiquitous_display")
        ud_ang = self.quaternion_to_euler(ud_pose.orientation)

        for name in self.name_list:
            actor_pose = self.get_pose(name)
            actor_ang = self.quaternion_to_euler(actor_pose.orientation)

            if actor_ang.z > 0:
                print ("plus")
            elif actor_ang.z < 0 and actor_pose.position.x - ud_pose.position.x > 4.0:
                proj_pos = actor_pose.position.x - 4.0
                print ("target pose: ", proj_pos, actor_pose.position.y, name)
                print (int(ud_ang.z))
                distance, radian = self.get_distance(proj_pos, actor_pose.position.y, ud_pose.position.x, ud_pose.position.y,)
                if distance > 1.5 and distance < 2.5:
                    ### calculate pan and tilt radian
                    pan_ang = self.calculate_pan_ang(radian, ud_ang.z)
                    tilt_ang = self.calculate_tilt_ang(distance)

                    ### check pan angle (limit +-2.9670)
                    if abs(pan_ang) < 2.9670:
                        responce = self.set_pantilt_func(pan_ang,tilt_ang)
                        if responce:
                            resp.success = True
                            self.on_off_project(1)
                            while True:
                                actor_pose = self.get_pose(name)
                                if actor_pose.position.x < proj_pos:
                                    break
                                if self.scan_flag == 1:
                                    resp.success = False
                                    break
                            self.on_off_project(0)
                            responce = self.set_pantilt_func(-(math.pi / 2.0),0.0)
            else:
                pass
        return resp

    def scan_callback(self, msg):
        state,done = self.calculate_observation(msg)
        # print done
        if done:
            self.scan_flag = 1
        else:
            self.scan_flag = 0
        # print self.scan_flag

    def calculate_observation(self, data):
        min_range = 0.4
        done = False
        for i, item in enumerate(data.ranges):
            if (min_range > data.ranges[i] > 0):
                done = True
        return data.ranges,done

    def set_pantilt_func(self, pan, tilt):
        pt_msg = AddPointsRequest()

        pt_msg.position.x = pan
        pt_msg.position.y = tilt

        response = self.set_pantilt(pt_msg)

        return response

    def on_off_project(self, on_off):
        int_msg = Int32()
        if on_off == 1:
            int_msg = 1
        elif on_off == 0:
            int_msg = 0

        self.image_pub.publish(int_msg)

    def get_pose(self, name):
        set = GetModelStateRequest()
        set.model_name = name
        response = self.call(set)
        return response.pose

    def quaternion_to_euler(self, quaternion):
        """Convert Quaternion to Euler Angles

        quarternion: geometry_msgs/Quaternion
        euler: geometry_msgs/Vector3
        """
        e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return Vector3(x=e[0], y=e[1], z=e[2])

    def get_distance(self, x1, y1, x2, y2):
        d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        r = math.atan2(y1 - y2,x1 - x2)
        return d, r

    def calculate_tilt_ang(self, distance):
        rad_tilt = math.atan2(1.21, distance)
        return rad_tilt

    def calculate_pan_ang(self, radian, ud_ang):
        if radian < 3.14:
            rad_pan = -(math.pi / 2.0) + radian - ud_ang
        else:
            rad_pan = -(math.pi / 2.0) - ((math.pi*2.0)-radian) - ud_ang
        return rad_pan

if __name__ == '__main__':
    rospy.init_node('neuro_projection_service')

    server = Server()

    rospy.spin()
