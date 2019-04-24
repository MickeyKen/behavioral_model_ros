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
        smach.State.__init__(self, outcomes=['to_Pa', 'to_C'])

    def callback(self, data):
        request = rospy.ServiceProxy('/patrol/2012', AddPoseRetStr)
        pose = PoseStamped()
        responce = request(pose)

        if responce.result.data == "success":
            return 'to_Pa'
        elif responce.result.data == "human":
            return 'to_C'

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

    sm = smach.StateMachine(outcomes=['success'])
    with sm:

        ### moving initial_pose ###
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
            smach.StateMachine.add('patrol',
                            ServiceState('/search_and_wander/move_base/goal',
                                         AddPoseRetStr,
                                         responce = a_pose(b_pose)),
                                         transitions={'to_c':'calcurate_pose',
                                                      'to_pa': 'patrol'})

            smach.StateMachine.add('calcurate_pose',
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


    sis = smach_ros.IntrospectionServer('behaviora_model_2012', sm, '/ROOT')
    sis.start()
    rospy.sleep(1)

    result = sm.execute()
    rospy.loginfo('result: %s' % result)
    rospy.spin()
    sis.stop()
