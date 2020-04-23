#!/usr/bin/env python

import rospy
import numpy as np
import math
import time
import tf

from std_msgs.msg import String, Bool, Int32, Float64
from geometry_msgs.msg import Point, Vector3

from std_srvs.srv import SetBool, SetBoolResponse
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from ubiquitous_display_pantilt.srv import AddPoints

class Server():
    def __init__(self):

        self.x_position = 4.0
        self.y_position = 1.25
        self.offset = 0.5

        human_number = 5
        self.name_list = []
        for i in range(human_number):
            self.name_list.append("actor" + str(i))

        self.pan_pub = rospy.Publisher('/ubiquitous_display/pan_controller/command', Float64, queue_size=10)
        self.tilt_pub = rospy.Publisher('/ubiquitous_display/tilt_controller/command', Float64, queue_size=10)

        self.image_pub = rospy.Publisher('/ubiquitous_display/image', Int32, queue_size=10)
        self.on_off_project(0)

        self.set_pantilt = rospy.ServiceProxy('action_plan_pantilt', AddPoints)

        # Declaration Service Server
        self.server = rospy.Service("/projection/service", SetBool, self.service_callback)

        self.call = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    def service_callback(self, req):
        resp = SetBoolResponse()
        resp.message = "called"
        resp.success = False

        ud_pose = self.get_pose("ubiquitous_display")

        for name in self.name_list:
            actor_pose = self.get_pose(name)
            # print ud_pose.position.x - actor_pose.position.x
            # print ud_pose.position.y - actor_pose.position.y
            ang = self.quaternion_to_euler(actor_pose.orientation)

            if ang.z > 0:
                print ("plus")
            elif ang.z < 0:
                print ("minus")
                print actor_pose.position
                proj_pos = actor_pose.position.x - 2.5
                distance = self.get_distance(proj_pos, actor_pose.position.y, ud_pose.position.x, ud_pose.position.y,)
                print distance
                if distance > 2.0 and distance < 4.5:
                    break
            else:
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
        return d

if __name__ == '__main__':
    rospy.init_node('neuro_projection_service')

    server = Server()

    rospy.spin()
