#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    values = str(data).split()
    values.pop(0)
    #print values
    x_gain = float(values[0])
    xmin = float(values[1])
    xmax = float(values[2])
    st_gain = float(values[3])
    print "x_gain ", x_gain, " xmin ", xmin, " xmax ", xmax, " st_gain ", st_gain
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('set_listener', anonymous=True)

    rospy.Subscriber("gains", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
