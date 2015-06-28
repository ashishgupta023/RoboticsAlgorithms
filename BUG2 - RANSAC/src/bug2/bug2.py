#!/usr/bin/env python
import roslib
import rospy
import tf
from visualization_msgs.msg import Marker
import math
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
import random
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from laser_geometry import LaserProjection

from nav_msgs.msg import Odometry

class Bug2:
 	def __init__(self):
		self.msg = Twist()
		self.collinear = False
		self.collinearArea = False
		self.state = "GOALSEEK"
		
		self.left_line = 0 
		self.front_line = 0
		self.angle_line = 0
		rospy.init_node('bug2')
		self.pub = rospy.Publisher('/robot_0/cmd_vel', Twist,queue_size=100)
		rospy.Subscriber('/visualization_marker', Marker, self.callback)
		rospy.Subscriber('/robot_0/odom', Odometry, self.callback_odom)
		rospy.Subscriber('/robot_0/base_scan', LaserScan, self.callback_laser)

	def callback_odom(self,data):
		x = -8.0 - (data.pose.pose.position.y)
		y = -2.0 + (data.pose.pose.position.x)
		rospy.loginfo("Curr position " + str(x) + " "+ str(y))
		goal_distance = math.sqrt(math.pow((4.5 - x),2) + math.pow( (9-y) , 2 ))
		rospy.loginfo("Curr goal distance " + str(goal_distance))
		goal_angle = math.atan2((9-y),(4.5-x))
		fixed_goal_angle = math.atan2((9-(-2)),(4.5-(-8)))
		#goal_angle = math.degrees(goal_angle)
		rospy.loginfo("----------------" + str(fixed_goal_angle))
		robot_angle = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,data.pose.pose.orientation.w])
		curr_robot_angle = 0
		
		#if math.degrees(1.57 + robot_angle[2])  > 180:
		#	curr_robot_angle = math.degrees(1.57 + robot_angle[2]) - 360
		#else: 
		#	curr_robot_angle =  math.degrees(1.57 + robot_angle[2])
		curr_robot_angle =  1.57 + robot_angle[2]
		rospy.loginfo(curr_robot_angle)

		rospy.loginfo("Current turn required " + str( goal_angle - curr_robot_angle))
		rospy.loginfo("++++++++++++++++++++++" + self.state)

		#Calculate Area of Triangle A- start B - goal C- robot
		self.collinearArea =math.fabs( (-8.0 * (9 - y) + 4.5*( y - (-2)) + x *(-2.0 - (9)))) / 2
		rospy.loginfo("////////////////" + str(self.collinearArea))
		rospy.loginfo("``````````GOAL ANGLE````````````````" + str(goal_angle))
		rospy.loginfo("``````````Curr Robot````````````````" + str(curr_robot_angle))
		rospy.loginfo("``````````````Difference````````````" + str(goal_angle - curr_robot_angle ))
		rospy.loginfo("Line Angle" + str(self.angle_line))
		if goal_distance > 0.7:	
			if self.state == "GOALSEEK":		
				#if(curr_robot_angle != goal_angle):
				self.msg.angular.z = min(goal_angle - curr_robot_angle ,0.5)
				self.msg.linear.x = 1
				#else:
				#self.msg.angular.z = 0
				#self.msg.linear.x = 1
			else:
				if  self.front_line == 1:
					rospy.loginfo("---------FRONT LINE-------")
					self.msg.angular.z = -math.fabs(self.angle_line/2.5)
					self.msg.linear.x = 0
				else:
					rospy.loginfo("---------NO FRONT LINE-------")
					if self.left_line == 0:	
						rospy.loginfo("---------NO LEFT LINE-------")
						self.msg.angular.z = (0.5)
						self.msg.linear.x = 0.8
					else:
						rospy.loginfo("---------LEFT LINE-------")
						self.msg.angular.z = 0.1
						self.msg.linear.x = 0.8
		else:
			self.msg.angular.z = 0
			self.msg.linear.x = 0
		

	def callback(self,data):
		rospy.loginfo("Total published lines " + str(len(data.points) / 2))
		if len(data.points) != 0:
				slope_line = (data.points[1].y - data.points[0].y) /  (data.points[1].x - data.points[0].x)
				self.angle_line = math.atan2((data.points[1].y - data.points[0].y) ,( data.points[1].x - data.points[0].x ))
				rospy.loginfo("-----Obstacle orientation----- " + str(self.angle_line))
	
	def callback_laser(self,data):
		#rospy.loginfo(data.ranges[180:360])

		for scan_data in range(180,360):
			if data.ranges[scan_data] < 1:
				self.left_line = 1
			else:
				self.left_line = 0
		count = 0 

		for scan_data in range(120,240):
			if data.ranges[scan_data] < 1:
				count = count + 1
		print "~~~~~~~~Counter Front~~~~~~~~~~~",count 
		if count > 10:
			self.front_line = 1
		else:
			self.front_line = 0


		if self.state == "GOALSEEK":
			#for scan_data in range(150,210):
			if count > 10:
				obstacle = 1
				self.state = "WALLFOLLOW"
			else:
				obstacle = 0
				self.state = "GOALSEEK"	

		if self.collinearArea <= 0.8 and self.front_line == 0:
			self.collinear = True
			self.state = "GOALSEEK"
			
		
 	def run(self):
        	r = rospy.Rate(10)
		try:
        		while not rospy.is_shutdown():
				self.pub.publish(self.msg)				
        	    		r.sleep()
		except rospy.ROSInterruptException:
			pass


if __name__ == '__main__':
	obj = Bug2()
	obj.run()
