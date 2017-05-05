#!/usr/bin/env python

import rospy
import tf
import numpy as np
import geometry_msgs.msg
from geometry_msgs.msg import Twist
import math
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf2_ros
import copy
from scipy import signal

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
        self.x_gain = 8.14
        self.xmin = 5.7
        self.xmax = 29.8000
        self.st_gain = 300.0
        self.variance = 10000

        self.twist.linear.x = 0.0
        self.twist.angular.z = 5.0
        self.cmd_vel_pub.publish(self.twist)        

        while rospy.is_shutdown():
            self.twist.linear.x = 0.0
            self.twist.angular.z = 5.0
            self.cmd_vel_pub.publish(self.twist)


    def gain_callback(self, gain_str):
        values = str(gain_str).split()
        values.pop(0)
        #print values
        self.x_gain = float(values[0])
        self.xmin = float(values[1])
        self.xmax = float(values[2])
        self.st_gain = float(values[3])
        self.variance = float(values[4])
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
	
        s_ang = (PI/2) - scan.angle_max + (start_ang*ang_inc)
        
        max_arr = np.isinf(laser_data)*scan.range_max
        laser_data = np.logical_not(np.isinf(laser_data))*laser_data + max_arr
        inc_arr = np.array(range(len(laser_data)))
        x_coor = laser_data*np.cos((np.ones(laser_data.shape))*s_ang + inc_arr*ang_inc)
        y_coor = laser_data*np.sin((np.ones(laser_data.shape))*s_ang + inc_arr*ang_inc)

        x_mean = np.mean(x_coor)
        y_mean = np.mean(y_coor)
		
		
	a=1;            # Scales the magnitude of the magnification
	b = .15e-8;     # Scales the stdev of magnification such that when the mean 
					# is in the center of the scan,the outermost magnification 
					# is equal to b*mu_center (ask for clarity) 0<b<1
	variance = self.variance
	print "variance is ", variance

	mu = math.atan2(y_mean,x_mean);
	mu_index = math.floor((mu-s_ang)/ang_inc);
    
	#x_magnified = np.zeros((len(laser_data),1));
	#y_magnified = np.zeros((len(laser_data),1));
	x_magnified=[];
	y_magnified=[];	
	for i in range(1,len(laser_data)):
		magnification=(a/(2*PI*variance)**.5)*math.exp((-(i-mu_index)**2)/(2*variance));
		#print "magnification ", magnification, " index i is ", i
		if (laser_data[i] == np.inf):
			mag = laser_msg.range_max*(magnification);
		else:
			mag = laser_data[i]*(magnification);			
		x_magnified.append(mag*math.cos(s_ang + (i*ang_inc))); 
		y_magnified.append(mag*math.sin(s_ang + (i*ang_inc)));
    
	x_mean_mag=np.mean(x_magnified);
	y_mean_mag=np.mean(y_magnified);

	#print "x_mean_map is ", x_mean_mag, " y_mean_mag is ", y_mean_mag
	gmask = signal.gaussian(9, std=2)
	gmask = gmask/np.max(gmask)
	gl_array = np.convolve(laser_data,gmask)

	der_mask = np.array([0, -1, 1])
	dgl_arr = np.convolve(gl_array,der_mask)
	
	tolerance = 0.75

	theta_axis = inc_arr*ang_inc
	if np.sum(dgl_arr > tolerance)> 0.0:
		thresh = dgl_arr > tolerance
		dtemp = thresh*dgl_array
		ind = np.argwhere(dtemp == np.max(dtemp))[0][0]-1
		x1 = x_coor[0]
		y1 = y_coor[0]
		x2 = x_coor[ind]
		y2 = y_coor[ind]
		m = (y2 - y1)/(x2 - x1)
		laser_data[0:ind] = (-m*x1 + y1)/(np.sin(theta_axis[0:ind]) - m*np.cos(theta_axis[0:ind]))

	elif np.sum(dgl_arr < -tolerance)> 0.0:
		thresh = dgl_arr < -tolerance
		dtemp = thresh*dgl_array
		ind = np.argwhere(dtemp == np.max(dtemp))[0][0]+1
		x1 = x_coor[len(laser_data)-1]
		y1 = y_coor[len(laser_data)-1]
		x2 = x_coor[ind]
		y2 = y_coor[ind]
		m = (y2 - y1)/(x2 - x1)
		laser_data[ind:len(laser_data)] = (-m*x1 + y1)/(np.sin(theta_axis[ind:len(laser_data)]) - m*np.cos(theta_axis[ind:len(laser_data)]))
	
	x_coor = laser_data*np.cos((np.ones(laser_data.shape))*s_ang + inc_arr*ang_inc)
        y_coor = laser_data*np.sin((np.ones(laser_data.shape))*s_ang + inc_arr*ang_inc)

        x_mean = np.mean(x_coor)
        y_mean = np.mean(y_coor)
	l_mag = math.sqrt(math.pow(x_mean_mag,2) + math.pow(y_mean_mag,2))
	x_vel = self.xmin + (self.x_gain*l_mag);
	if x_vel > self.xmax:
		x_vel = self.xmax;
	
	print "x_vel", x_vel


        axle_dis = 0.34

        
        curv_ = (2*x_mean)/(x_mean**2 + y_mean**2)
        phi = math.atan(curv_*axle_dis)
        self.twist.linear.x = x_vel
        self.twist.angular.z = (-phi*self.st_gain)+5
        self.cmd_vel_pub.publish(self.twist)

if __name__ == '__main__':
    try:
        pure_pursuit_twist()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
