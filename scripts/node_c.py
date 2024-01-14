#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger, TriggerResponse

min_distance = float('inf')  # Initialize min_distance to positive infinity

def clbk_laser(msg):
    global min_distance
    # 720/5 = 144
    regions = [
        min(min(msg.ranges[0:143]), 10),
        min(min(msg.ranges[144:287]), 10),
        min(min(msg.ranges[288:431]), 10),
        min(min(msg.ranges[432:575]), 10),
        min(min(msg.ranges[576:713]), 10),
    ]
    min_distance = min(regions)

def min_distance_callback(req):
    global min_distance
    rospy.loginfo(f"Minimum Distance: {min_distance}")
    return TriggerResponse(success=True, message=f"Minimum Distance: {min_distance}")

def main():
    rospy.init_node('/node_c_firstpart')
    sub = rospy.Subscriber("/scan", LaserScan, clbk_laser)
    srv = rospy.Service('get_min_distance', Trigger, min_distance_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

