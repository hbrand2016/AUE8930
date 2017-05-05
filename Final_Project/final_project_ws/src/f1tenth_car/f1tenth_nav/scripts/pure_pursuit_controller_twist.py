#!/usr/bin/env python

import rospy
import tf
import numpy as np
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from math import *
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf2_ros
import copy

class pure_pursuit_twist():
    
    def __init__(self):
        # initiliaze
        rospy.init_node('pure_pursuit', anonymous=True)

        # What to do you ctrl + c    
        self.laser_sub = rospy.Subscriber('/scan',
		LaserScan, self.laser_callback)
		
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
			Twist, queue_size=1)
		
        self.gain_sub = rospy.Subscriber('gains',
		String, self.gain_callback)

		self.twist = Twist()

		self.error = 0.0
		self.prev = rospy.get_time()
        #r = rospy.Rate(20)
        
        #r.sleep()
        self.x_gain = 4.2
        self.xmin = 18.0
        self.xmax = 27.0
        self.st_gain = 150.0

        self.twist.linear.x = 0.0
        self.twist.angular.z = 5.0
        self.cmd_vel_pub.publish(self.twist)        

        while not rospy.is_shutdown():
            self.twist.linear.x = 0.0
            self.twist.angular.z = 5.0
            self.cmd_vel_pub.publish(self.twist)


    def gain_callback(gain_str):
        values = str(gain_str).split()
        values.pop(0)
        #print values
        x_gain = float(values[0])
        xmin = float(values[1])
        xmax = float(values[2])
        st_gain = float(values[3])
        #print "x_gain ", x_gain, " xmin ", xmin, " xmax ", xmax, " st_gain ", st_gain

    def laser_callback(self, scan):
		PI = 3.14159265359
        laser_data = scan.ranges
		min_ang = scan.angle_min
		max_ang = scan.angle_max
		ang_inc = scan.angle_increment
		
		start_ang = int((PI/4)/ang_inc)
        end_ang = len(laser_data)-start_ang
		laser_data = laser_data[start_ang:end_ang]
		
        s_ang = (PI/2) - scan.AngleMax + (start_ang*ang_inc)
        
        max_arr = np.isinf(laser_data)*scan.range_max
        laser_data = np.logical_not(np.isinf(laser_data))*laser_data + max_arr
        inc_arr = range(len(laser_data))
        x_coor = laser_data*np.cos((np.ones(laser_data.shape))*s_ang + inc_arr*ang_inc)
        y_coor = laser_data*np.sin((np.ones(laser_data.shape))*s_ang + inc_arr*ang_inc)

        x_mean = np.mean(x_coor)
        y_mean = np.mean(y_coor)

        axle_dis = 0.34

        l = np.sqrt((x_mean**2 + y_mean**2))
        x_vel = self.xmin + (self.x_gain*l)
        if x_vel > self.xmax:
            x_vel = self.xmax

        curv_ = (2*x_mean)/(x_mean**2 + y_mean**2)
        phi = atan(curv_*axle_dis)
        self.twist.linear.x = x_vel
        self.twist.angular.z = (-phi*self.st_gain)+5
        self.cmd_vel_pub.publish(self.twist)

if __name__ == '__main__':
    try:
        pure_pursuit()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")