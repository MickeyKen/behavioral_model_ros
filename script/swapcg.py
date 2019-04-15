#!/usr/bin/env python
import rospy
import smach
import smach_ros
import random

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


class Communicate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_SW', 'to_G'])

    def execute(self, userdata):
        rospy.loginfo('Executing int state DSTATE')
        rospy.sleep(1)

        if random.random() > 0.5:
            return 'to_SW'

        return 'to_G'

class Guide(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_C', 'succeed'])

    def execute(self, userdata):
        rospy.loginfo('Executing int state DSTATE')
        rospy.sleep(1)

        if random.random() > 0.5:
            return 'to_C'

        return 'succeed'


if __name__ == '__main__':

    rospy.init_node('behaviora_model_for_ud')

    sm = smach.StateMachine(outcomes=['succeed', 'failed'])

    with sm:
        smach.StateMachine.add('Search_and_Wander', Search_and_Wander(),
                               transitions={'to_A':'Approach'})

        smach.StateMachine.add('Approach', Approach(),
                               transitions={'to_SW':'Search_and_Wander',
                                            'to_P':'Provide'})

        smach.StateMachine.add('Provide', Provide(),
                               transitions={'to_SW':'Search_and_Wander',
                                            'to_C':'Communicate'})

        smach.StateMachine.add('Communicate', Communicate(),
                               transitions={'to_G':'Guide',
                                            'to_SW':'Search_and_Wander'})

        smach.StateMachine.add('Guide', Guide(),
                               transitions={'to_C':'Communicate',
                                            'succeed':'succeed'})


    sis = smach_ros.IntrospectionServer('behaviora_model_for_ud_server', sm, '/ROOT')
    sis.start()
    rospy.sleep(1)

    result = sm.execute()
    rospy.loginfo('result: %s' % result)
    rospy.spin()
    sis.stop()
