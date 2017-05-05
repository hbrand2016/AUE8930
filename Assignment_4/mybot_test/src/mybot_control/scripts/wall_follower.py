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

		self.laser_sub = rospy.Subscriber('/mybot/laser/scan',
		LaserScan, self.laser_callback)
		
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
			Twist, queue_size=1)
		
		self.twist = Twist()

		self.error = 0.0
		self.prev = rospy.get_time()

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
		
		min_ang = scan.angle_min
		max_ang = scan.angle_max
		ang_in = scan.angle_increment
		laser_arr = scan.ranges
		ind_front = int(len(laser_arr)/2)
		depth_front = laser_arr[ind_front-2:ind_front+3]
		depth_avg = numpy.mean(depth_front)

		Ks = 1 - (0.5/depth_avg)

		cur = rospy.get_time()
		dt = cur - self.prev
		self.prev = cur
		
		print "dt is: ", dt

		theta = 3.14159265359/4
		ang_zero_i = 0 #numpy.int(numpy.abs(max_ang/ang_in))
		ang_45_i = ang_zero_i + numpy.int(numpy.abs(theta/ang_in))
		print "laser data indices 0, 45 ", ang_zero_i, ang_45_i
		a = laser_arr[ang_45_i]
		print "a is ", a
		b = laser_arr[ang_zero_i]
		print "b is ", b
		alpha = numpy.arctan(((a*numpy.cos(theta)) - b)/(a*numpy.sin(theta)))
		
		target_dist = 1.0
		target_bearing = 0.0
		
		Kp = 0.7
		Kd = 0.001

		
		dist_err = target_dist - (b*numpy.cos(alpha))
		angle_err = target_bearing - alpha
		total_err = angle_err + 0.35*dist_err
		print "total is ", total_err
		V_th = Kp*(total_err) + Kd*((total_err - self.error)/dt)
		self.error = total_err
		
		
		
		
		self.twist.linear.x = Ks*0.2
		if numpy.isnan(V_th):
			self.twist.angular.z = float(0) / 100
		else:
			self.twist.angular.z = V_th
			print V_th
		self.cmd_vel_pub.publish(self.twist)


rospy.init_node('wall_follower')
follower = Follower()
rospy.spin()
