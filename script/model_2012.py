#!/usr/bin/env python
import rospy, actionlib, math, tf
from actionlib_msgs.msg import *
import smach
import smach_ros
import random
from smach_ros import ServiceState, SimpleActionState
from geometry_msgs import Pose, Quaternion
from move_base_msgs import MoveBaseAction, MoveBaseGoal

class Search_and_Wander(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_A'])

        self.subscriber = rospy.Subscriber('/test', Pose, self.callback)

    def callback(self, data):
        rospy.set_param('/target_human_pose/x', data.x)
        rospy.set_param('/target_human_pose/y', data.y)
        return data.data

def calc_pose():
    ### get human position ###
    x_value = rospy.get_param("/target_human_pose/x")
    y_value = rospy.get_param("/target_human_pose/y")

class Approach(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_SW', 'to_P'])

    def execute(self, userdata):
        rospy.loginfo('Executing int state BSTATE')
        rospy.sleep(1)

        if random.random() > 0.5:
            return 'to_SW'

        return 'to_P'

class Provide(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_SW', 'to_C'])

    def execute(self, userdata):
        rospy.loginfo('Executing int state Communicate')
        rand = random.random()
        rospy.sleep(1)

        if rand > 0.6:
            return 'to_SW'

        return 'to_C'


if __name__ == '__main__':

    rospy.init_node('new_behaviora_model_for_ud')

    sm = smach.StateMachine(outcomes=['succeed'])

    with sm:

        ###start Search_and_Wander ###
        sw_sub = smach.StateMachine(outcomes=['success'])
        with sw_sub:
            ### init position ###
            pose  = MoveBaseGoal()
            pose.target_pose.header.frame_id = "base_link"
            pose.target_pose.header.stamp = rospy.Time.now()
            pose.target_pose.pose.position.x =  0.0
            pose.target_pose.pose.position.y =  1.0
            q = tf.transformations.quaternion_from_euler(0, 0, 3.14)
            pose.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
            smach.StateMachine.add('Set_Pose',
                                   SimpleActionState('/move_base/goal',
                                                     MoveBaseAction,
                                                     goal=pose),
                                   transitions={'succeeded': 'Search_and_Wander',
                                                'preempted': 'Set_Pose',
                                                'aborted': 'Set_Pose'})

            smach.StateMachine.add('Search_and_Wander', Search_and_Wander(),
                               transitions={'to_A':'Approach'})

            smach.StateMachine.add('Calcurate_Pose',
                                   CBState(calc_pose),
                                   {'success': 'success'})


        smach.StateMachine.add('Search_and_Wander', sw_sub,
                               transitions={'success': 'Approach'})

        ### start Approach ###
        a_sub = smach.StateMachine(outcomes=['success'])
        with a_sub:
            smach.StateMachine.add('Approach', Approach(),
                               transitions={'to_SW':'Search_and_Wander'})


        smach.StateMachine.add('Approach', a_sub,
                               transitions={'success': 'Provide'})

        ### Start Provide ###
        p_sub = smach.StateMachine(outcomes=['success'])

        with p_sub:
            smach.StateMachine.add('Provide', Provide(),
                               transitions={'to_SW':'Search_and_Wander'})

        smach.StateMachine.add('Provide', p_sub,
                               transitions={'success': 'success'})


    sis = smach_ros.IntrospectionServer('behaviora_model_for_ud_server', sm, '/ROOT')
    sis.start()
    rospy.sleep(1)

    result = sm.execute()
    rospy.loginfo('result: %s' % result)
    rospy.spin()
    sis.stop()
