#!/usr/bin/env python
import rospy, actionlib, math, tf
from actionlib_msgs.msg import *
import smach
import smach_ros
import random
from smach_ros import ServiceState, SimpleActionState
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from behavioral_model.srv import AddPoseRetStr

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

    initial_pose  = MoveBaseGoal()
    initial_pose.target_pose.header.frame_id = "map"
    initial_pose.target_pose.header.stamp = rospy.Time.now()
    initial_pose.target_pose.pose.position.x =  0.0
    initial_pose.target_pose.pose.position.y =  1.0
    q = tf.transformations.quaternion_from_euler(0, 0, 3.14)
    initial_pose.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])


    a_pose  = PoseStamped()
    a_pose.target_pose.header.frame_id = "map"
    a_pose.target_pose.header.stamp = rospy.Time.now()
    a_pose.target_pose.pose.position.x =  1.0
    a_pose.target_pose.pose.position.y =  0.0
    q = tf.transformations.quaternion_from_euler(0, 0, 3.14)
    a_pose.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

    b_pose  = PoseStamped()
    b_pose.target_pose.header.frame_id = "map"
    b_pose.target_pose.header.stamp = rospy.Time.now()
    b_pose.target_pose.pose.position.x =  -1.0
    b_pose.target_pose.pose.position.y =  0.0
    q = tf.transformations.quaternion_from_euler(0, 0, 3.14)
    b_pose.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

    c_pose  = PoseStamped()
    c_pose.target_pose.header.frame_id = "base_link"
    c_pose.target_pose.header.stamp = rospy.Time.now()
    c_pose.target_pose.pose.position.x =  -1.0
    c_pose.target_pose.pose.position.y =  2.0
    q = tf.transformations.quaternion_from_euler(0, 0, 3.14)
    c_pose.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

    d_pose  = PoseStamped()
    d_pose.target_pose.header.frame_id = "base_link"
    d_pose.target_pose.header.stamp = rospy.Time.now()
    d_pose.target_pose.pose.position.x =  2.0
    d_pose.target_pose.pose.position.y =  1.0
    q = tf.transformations.quaternion_from_euler(0, 0, 3.14)
    d_pose.target_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

    sm = smach.StateMachine(outcomes=['success'])
    with sm:

        init_sub = smach.StateMachine(outcomes=['success'])
        with init_sub:

            smach.StateMachine.add('init_pose',
                                   SimpleActionState('/move_base/goal',
                                                     MoveBaseAction,
                                                     goal=initial_pose),
                                   transitions={'succeeded': 'success',
                                                'preempted': 'success',
                                                'aborted': 'init_pose'})

        smach.StateMachine.add('Set_Pose', init_sub,
                           transitions={'success': 'Search_and_Wander'})

        ###start Search_and_Wander ###
        sw_sub = smach.StateMachine(outcomes=['success'])
        with sw_sub:
            smach.StateMachine.add('a_pose',
                            ServiceState('/search_and_wander/move_base/goal',
                                         AddPoseRetStr,
                                         responce = a_pose(b_pose)),
                                         transitions={'succeeded':'b_pose'})
            smach.StateMachine.add('b_pose',
                            ServiceState('/search_and_wander/move_base/goal',
                                         AddPoseRetStr,
                                         responce = AddPoseRetStr(c_pose)),
                                         transitions={'succeeded':'c_pose'})
            smach.StateMachine.add('c_pose',
                            ServiceState('/search_and_wander/move_base/goal',
                                         AddPoseRetStr,
                                         responce = AddPoseRetStr(d_pose)),
                                         transitions={'succeeded':'d_pose'})
            smach.StateMachine.add('d_pose',
                            ServiceState('/search_and_wander/move_base/goal',
                                         AddPoseRetStr,
                                         responce = AddPoseRetStr(a_pose)),
                                         transitions={'succeeded':'a_pose'})
            smach.StateMachine.add('Apoint', Search_and_Wander(),
                               transitions={'to_c':'Calcurate'})

            smach.StateMachine.add('Calcurate_Pose',
                                   CBState(calc_pose),
                                   {'success': 'success'})


        smach.StateMachine.add('Search_and_Wander', a_sub,
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
