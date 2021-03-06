#!/usr/bin/env python
import roslib
import rospy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Perception:
 	def __init__(self):
		self.msg = Twist()
		rospy.init_node('perception')
		self.pub = rospy.Publisher('robot_0/cmd_vel', Twist, queue_size=10)
		#rospy.Subscriber('robot_0/odom', Odometry , self.callback_odom)
		rospy.Subscriber('robot_0/base_scan', LaserScan, self.callback)

	def callback(self,data):
	    obstacle = 0;
 	    for scan_data in data.ranges: 
		if scan_data < 1.0:
			obstacle = 1
	
	    if obstacle == 1:
		#rospy.loginfo(rospy.get_caller_id() + " Obstacle encountered in front :( Stopping " )			
	    	self.msg.linear.x = 0
		turn = 0.0
		for i in range(0,len(data.ranges)): 
			if data.ranges[i] >= 1.0:
				if i == 180:
					turn = 0
				else:
					turn = data.angle_min + (i * data.angle_increment)	
			break
		#rospy.loginfo(rospy.get_caller_id() + " Trying to recover %s",turn )
		self.msg.angular.z = turn		
	    else:
		#rospy.loginfo(rospy.get_caller_id() + " No Obstacle :) - Let's explore " )    	
		self.msg.angular.z = 0			
		self.msg.linear.x = 2

	    
 	def run(self):
        	r = rospy.Rate(10)
		try:
        		while not rospy.is_shutdown():
        	    		self.pub.publish(self.msg)
        	    		r.sleep()
		except rospy.ROSInterruptException:
			pass


if __name__ == '__main__':
	obj = Perception()
	obj.run()
