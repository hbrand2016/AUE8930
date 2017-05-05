#!/usr/bin/env python

import math
import rospy
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent


def processBump(data):
    global bump
    if(data.state == BumperEvent.PRESSED):
        bump = True
    else:
        bump = False
    rospy.loginfo("Bumped")
    rospy.loginfo(data.bumper)

def main():
    
    rospy.init_node('bumper_sensor')
    rospy.Subscriber('mobile_base/events/bumper',BumperEvent,processBump)
    rospy.spin()

if __name__ == '__main__':
    main()
