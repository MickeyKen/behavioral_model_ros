#!/usr/bin/env python
import rospy, actionlib, math, tf
from actionlib_msgs.msg import *
import smach
import smach_ros
import random
from smach_ros import ServiceState, SimpleActionState
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from behavioral_model.srv import AddPoseRetStr, AddStrRetPose
from people_msgs.msg import PositionMeasurementArray
from std_msgs.msg import String

class Patrol(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_Pa', 'success'])

    def execute(self, userdata):
        # rospy.set_param('/target_human/name', "")
        request = rospy.ServiceProxy('/search/target_human', AddPoseRetStr)
        pose = PoseStamped()
        responce = request(pose)

        # print responce.result.data

        if responce.result.data == "nohuman":
            return 'to_Pa'
        else:
            rospy.set_param('/target_human/name', responce.result.data)
            return 'success'

class Prediction(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_De', 'to_SW'])

    def execute(self, userdata):
        request = rospy.ServiceProxy('/prediction/target_human', AddPoseRetStr)
        pose = PoseStamped()
        responce = request(pose)
        if responce.result.data == "true":
            return 'to_De'

        return 'to_SW'

        # print responce.result.da

class Decide(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_Go', 'to_SW'])

    def execute(self, userdata):
        request = rospy.ServiceProxy('/detection/optimize_point', AddStrRetPose)
        req = String()
        responce = request(req)
        if responce.result_pose:
            rospy.set_param('/optimize/point/x', responce.result_pose.pose.position.x)
            rospy.set_param('/optimize/point/y', responce.result_pose.pose.position.y)
            return 'to_Go'

        return 'to_SW'
        # print responce.result.da

class Go(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        return 'success'

class Calc_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'to_SW'])

    def execute(self, userdata):
        rospy.loginfo('Executing int state BSTATE')
        rospy.sleep(1)

        if random.random() > 0.5:
            return 'success'

        return 'to_SW'

# class Approach(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['to_SW', 'to_P'])
#
#     def execute(self, userdata):
#         rospy.loginfo('Executing int state BSTATE')
#         rospy.sleep(1)
#
#         if random.random() > 0.5:
#             return 'to_SW'
#
#         return 'to_P'

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

        # ### moving initial_pose ###
        # init_sub = smach.StateMachine(outcomes=['success'])
        # with init_sub:
        #
        #     smach.StateMachine.add('init_pose',
        #                            SimpleActionState('/move_base/goal',
        #                                              MoveBaseAction,
        #                                              goal=initial_pose),
        #                            transitions={'succeeded': 'success',
        #                                         'preempted': 'success',
        #                                         'aborted': 'init_pose'})
        #
        # smach.StateMachine.add('Set_Pose', init_sub,
        #                    transitions={'success': 'Search_and_Wander'})

        ###start Search_and_Wander ###
        sw_sub = smach.StateMachine(outcomes=['success'])
        with sw_sub:
            smach.StateMachine.add('patrol',Patrol(),
                               transitions={'success':'success',
                                            'to_Pa': 'patrol'})

        smach.StateMachine.add('Search_and_Wander', sw_sub,
                               transitions={'success': 'Approach'})


        # start Approach ###
        a_sub = smach.StateMachine(outcomes=['success', 'to_SW'])
        with a_sub:
            smach.StateMachine.add('prediction_human', Prediction(),
                               transitions={'to_De':'decide_optimize_point',
                                            'to_SW':'to_SW'})

            smach.StateMachine.add('decide_optimize_point', Decide(),
                               transitions={'to_Go':'go_point',
                                            'to_SW':'to_SW'})

            smach.StateMachine.add('go_point', Go(),
                               transitions={'success':'success'})

        smach.StateMachine.add('Approach', a_sub,
                               transitions={'success': 'success',
                                            'to_SW': 'Search_and_Wander'})

        #
        # ### Start Provide ###
        # p_sub = smach.StateMachine(outcomes=['success'])
        #
        # with p_sub:
        #     smach.StateMachine.add('Provide', Provide(),
        #                        transitions={'to_SW':'Search_and_Wander'})
        #
        # smach.StateMachine.add('Provide', p_sub,
                               # transitions={'success': 'success'})


    sis = smach_ros.IntrospectionServer('behaviora_model_2012', sm, '/ROOT')
    sis.start()
    rospy.sleep(1)

    result = sm.execute()
    rospy.loginfo('result: %s' % result)
    rospy.spin()
    sis.stop()
