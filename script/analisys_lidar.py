#!/usr/bin/python
import rospy
from sensor_msgs.msg import LaserScan

import numpy as np



def discretize_observation(data,new_ranges):
    discretized_ranges = []
    min_range = 0.2
    done = False
    mod = len(data.ranges)/new_ranges
    for i, item in enumerate(data.ranges):
        if (i%mod==0):
            if data.ranges[i] == float ('Inf') or np.isinf(data.ranges[i]):
                discretized_ranges.append(6)
            elif np.isnan(data.ranges[i]):
                discretized_ranges.append(0)
            else:
                discretized_ranges.append(int(data.ranges[i]))
        if (min_range > data.ranges[i] > 0):
            done = True
    return discretized_ranges,done

if __name__ == '__main__':
    rospy.init_node('analysis_lidar_node', anonymous=True)

    data = rospy.wait_for_message('/scan', LaserScan, timeout=5)

    state,done = discretize_observation(data,5)

    print state, done


    # rospy.spin()
