#!/usr/bin/env python
import rospy
import smach
import smach_ros
import random
from smach_ros import ServiceState
from smach_ros import SimpleActionState

class Search_and_Wander(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_A'])

    def execute(self, userdata):
        rospy.loginfo('Executing int state ISTATE')
        rospy.sleep(1)
        return 'to_A'


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
            smach.StateMachine.add('Search_and_Wander', Search_and_Wander(),
                               transitions={'to_A':'Approach'})


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
