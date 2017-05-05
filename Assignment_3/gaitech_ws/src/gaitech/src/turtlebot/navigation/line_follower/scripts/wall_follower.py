#!/usr/bin/env python


#This Program is tested on Gazebo Simulator
#This script uses the cv_bridge package to convert images coming on the topic
#sensor_msgs/Image to OpenCV messages and then convert their colors from RGB to HSV
#then apply a threshold for hues near the color yellow to obtain the binary image 
#to be able to see only the yellow line and then follow that line
#It uses an approach called proportional and simply means 



import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist

class Follower:

	def __init__(self):
		
		#self.bridge = cv_bridge.CvBridge()
		#cv2.namedWindow("window", 1)
		
		#self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
		#	Image, self.image_callback)

		self.laser_sub = rospy.Subscriber('scan',
		LaserScan, self.laser_callback)
		
		self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
			Twist, queue_size=1)
		
		self.twist = Twist()

#	def image_callback(self, msg):
#
#		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
#		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#		lower_yellow = numpy.array([ 10, 10, 10])
#		upper_yellow = numpy.array([255, 255, 250])
#		mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
#		
#		h, w, d = image.shape
#		search_top = 3*h/4
#		search_bot = 3*h/4 + 20
#		mask[0:search_top, 0:w] = 0
#		mask[search_bot:h, 0:w] = 0
#
#		M = cv2.moments(mask)
#		if M['m00'] > 0:
#			cx = int(M['m10']/M['m00'])
#			cy = int(M['m01']/M['m00'])
#			cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
#The proportional controller is implemented in the following four lines	which
#is reposible of linear scaling of an error to drive the control output.	
#			err = cx - w/2
#			self.twist.linear.x = 0.2
#			self.twist.angular.z = -float(err) / 100
#			self.cmd_vel_pub.publish(self.twist)
#		cv2.imshow("window", image)
#		cv2.waitKey(3)
	
    
	def laser_callback(self, scan):
		theta = 0.5235987756
		dlen = 3*((1/numpy.cos(30))-(1/numpy.cos(30+scan.angle_increment)))
		laser_arr = scan.ranges
		right = laser_arr[0:5]
		#right = laser_arr[len(laser_arr)-5:len(laser_arr)]
		target = [(3/numpy.cos(30+scan.angle_increment))+(4*dlen),(3/numpy.cos(30+scan.angle_increment))+(3*dlen),(3/numpy.cos(30+scan.angle_increment))+(2*dlen),(3/numpy.cos(30+scan.angle_increment))+dlen,(3/numpy.cos(30+scan.angle_increment))]
		target = numpy.array(target)
		right = numpy.array(right)
		print(right)
		err = numpy.sum(target - right)/5.0
		self.twist.linear.x = 0.2
		if numpy.isnan(err):
			self.twist.angular.z = float(0) / 100
		else:
			self.twist.angular.z = float(err) / 100
		self.cmd_vel_pub.publish(self.twist)


rospy.init_node('wall_follower')
follower = Follower()
rospy.spin()
