#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import rosbag
import tf
from visualization_msgs.msg import Marker
import math
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
import random
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose2D
from laser_geometry import LaserProjection

import os
from nav_msgs.msg import Odometry

class localization:
 	def __init__(self):
		rospy.init_node('localization')
		self.gridCellProbabilites = np.zeros((35,35,36),dtype=np.float64)
		self.gridCellProbabilites[11,27,20] = 1.0 # 12, 28 , 3
		self.pub = rospy.Publisher('visualization_marker', Marker , queue_size=100)
		self.msgRviz = Marker()
		self.pointId = 1
		self.msgRviz.header.frame_id = "base_link"	    
		self.msgRviz.header.stamp = rospy.Time.now()
		self.msgRviz.type = Marker.LINE_LIST
		self.msgRviz.pose.orientation.w = 1.0
		self.msgRviz.ns= "line_strip"
		self.msgRviz.id = self.pointId
		self.msgRviz.scale.x = 0.1
		self.msgRviz.color.a = 1.0
		self.msgRviz.color.b = 1.0	
		self.p1 = Point()
		self.p1.x = 11*.20 + .10
		self.p1.y = 27*.20 + .10
		self.p1.z = 0 


		
	def plotTags(self):
		msgRviz = Marker()
		msgRviz.action = Marker.ADD
		msgRviz.header.frame_id = "base_link"	    
		msgRviz.header.stamp = rospy.Time.now()
		msgRviz.type = Marker.POINTS
		msgRviz.pose.orientation.w = 1.0
		msgRviz.ns= "point_strip"
		msgRviz.id = 90
		msgRviz.scale.x = 0.2
		msgRviz.scale.y = 0.2
		msgRviz.color.a = 1.0
		msgRviz.color.r = 1.0	
		p1 = Point()
		p1.x = 1.25
		p1.y = 5.25
		p1.z = 0
		p2 = Point()
		p2.x = 1.25
		p2.y = 3.25
		p2.z = 0
		p3 = Point()
		p3.x = 1.25
		p3.y = 1.25
		p3.z = 0
		p4 = Point()
		p4.x = 4.25
		p4.y = 1.25
		p4.z = 0
		p5 = Point()
		p5.x = 4.25
		p5.y = 3.25
		p5.z = 0
		p6 = Point()
		p6.x = 4.25
		p6.y = 5.25
		p6.z = 0
		msgRviz.points.append(p1)
		msgRviz.points.append(p2)
		msgRviz.points.append(p3)
		msgRviz.points.append(p4)
		msgRviz.points.append(p5)
		msgRviz.points.append(p6)
		self.pub.publish(msgRviz)	
	
	def getRequiredRotation(self,line_angle,pose_cell_theta):
		if(pose_cell_theta - line_angle  > 0 and pose_cell_theta - line_angle <  math.pi ):
			return -1*(math.pi - abs(abs(line_angle - pose_cell_theta) - math.pi))
		else:
			return (math.pi - abs(abs(line_angle - pose_cell_theta) - math.pi))
	
	def getRequiredRotationDeg(self,line_angle,pose_cell_theta):
		if(pose_cell_theta - line_angle  > 0 and pose_cell_theta - line_angle <  180):
			return -1*(180 - abs(abs(line_angle - pose_cell_theta) - 180))
		else:
			return (180 - abs(abs(line_angle - pose_cell_theta) - 180))
	
	def findMinimumError(self,line_angle,pose_cell_theta,actualRotation):
		diff1 = line_angle - pose_cell_theta
		error1 = actualRotation - diff1
		diff2 = 0
		if diff1 < 0:
			diff2 = 2*math.pi + diff1
		if diff1 > 0:
			diff2 = -(2*math.pi - diff1)
		error2 = actualRotation - diff2
		return min(abs(error1),abs(error2))
		#print diff1
		#print diff2
			
			
	def run(self):			
		bag = rosbag.Bag(rospy.get_param('bag_path'))
		#f = open('/home/ashish/catkin_ws/src/lab3/src/localization/myfile2','a')
		for topic, msg, t in bag.read_messages(topics=['Movements', 'Observations']): 	 
    			if topic == "Movements":
				gridCellProbabilitesTemp = np.zeros((35,35,36),dtype=np.float64)
				#f.write("Movement \n")
				for i in range (0,35): #no of cells in x 35 ( x' )
 					for j in range (0,35):	 #no of cells in y 35 
						for k in range(0,36): #no of cells in z - 90deg discretization - 4 
							pose_cell1 = Pose2D()
							pose_cell1.x = i*20.0  + 10.0
							pose_cell1.y = j*20.0  + 10.0
							pose_cell1.theta = k*(math.pi/18.0)

							#print "----" + str(i) + " " + str(j) + " " + str(k)
							#print "----" + str(pose_cell1.x) + " " + str(pose_cell1.y) + " " + str(pose_cell1.theta)		
							if self.gridCellProbabilites[i,j,k] >  0.01: #1/(35*35*36):
								for a in range (0,35): #no of cells in x 35 ( x )
									for b in range (0,35):	 #no of cells in y 35 
										for c in range(0,36): #no of cells in z - 90deg discretization - 4 
											pose_cell2 = Pose2D()
											pose_cell2.x = a*20.0  + 10.0
											pose_cell2.y = b*20.0 + 10.0
											pose_cell2.theta = c*(math.pi/18.0)
											requiredMotionTrans = math.sqrt((pose_cell2.y - pose_cell1.y)**2 + (pose_cell2.x - pose_cell1.x)**2 )
											line_angle = math.atan2(pose_cell2.y - pose_cell1.y , pose_cell2.x - pose_cell1.x)						
											if line_angle < 0: # to convert to 0,360
												line_angle = math.pi + (math.pi - math.fabs(line_angle))
											requiredMotionRot1 = self.getRequiredRotation(line_angle,pose_cell1.theta)
											requiredMotionRot2 = self.getRequiredRotation(pose_cell2.theta,line_angle)
											#print "source" + str(i) + " " + str(j) + str(k)
											#print "dest" + str(a) + " " + str(b) + str(c)
											#print str(requiredMotionRot1) + " "
											#print str(requiredMotionRot2)
											#print str(requiredMotionTrans)

											errorTrans = (msg.translation*100.0) - requiredMotionTrans
											#print "---Translation Error--- : " + str(errorTrans)
											actualRot1= tf.transformations.euler_from_quaternion([msg.rotation1.x, msg.rotation1.y, msg.rotation1.z,msg.rotation1.w])
											'''if actualRot1[2] < 0:
												actualRot1[2] = math.pi + (math.pi - math.fabs(actualRot1[2]))'''
											actualRot2= tf.transformations.euler_from_quaternion([msg.rotation2.x, msg.rotation2.y, msg.rotation2.z,msg.rotation2.w])
											'''if actualRot2[2] < 0:
												actualRot2[2] = math.pi + (math.pi - math.fabs(actualRot2[2]))'''
											errorRot1 = actualRot1[2] - requiredMotionRot1
											#errorRot1 = self.findMinimumError(line_angle,pose_cell1.theta,actualRot1[2])
											#print "---Rotation1 Error--- : " + str(errorRot1)
											errorRot2 = actualRot2[2] - requiredMotionRot2
											#errorRot2 = self.findMinimumError(pose_cell2.theta,line_angle,actualRot2[2]) 
											#print "---Rotation2 Error--- : " + str(errorRot2)
																								
											PerrorTrans = (1.0/(math.sqrt(10.0) *math.sqrt(2*math.pi))) * np.exp(-((errorTrans**2) / (2.0*10.0)))
											PerrorRot1 = (1.0/(math.sqrt(math.pi/36) *math.sqrt(2*math.pi))) * np.exp(-((errorRot1**2) / (2.0*(math.pi/36))))
											PerrorRot2 = (1.0/(math.sqrt(math.pi/36) *math.sqrt(2*math.pi))) * np.exp(-((errorRot2**2) / (2.0*(math.pi/36))))
											#print PerrorTrans
											#print PerrorRot1
											#print PerrorRot2
											#raw_input("enter")
											gridCellProbabilitesTemp[a,b,c] +=  (PerrorTrans * PerrorRot1 * PerrorRot2*self.gridCellProbabilites[i,j,k] )
				self.gridCellProbabilites = gridCellProbabilitesTemp
				'''for x in range (0,35): #no of cells in x 35
					for y in range (0,35):	 #no of cells in y 35 
						for z in range(0,36): #no of cells in z - 90deg discretization - 4 	
							f.write(str(x) +" " +  str(y) +" " + str(z) + " " + str(self.gridCellProbabilites[x,y,z]) + "\n") '''


			if topic == "Observations":
				#f.write("Observation \n")
				tagX = 0.0
				tagY = 0.0
				if msg.tagNum == 0:
					tagX = 125
					tagY = 525
				if msg.tagNum == 1:
					tagX = 125
					tagY = 325
				if msg.tagNum == 2:
					tagX = 125
					tagY = 125
				if msg.tagNum == 3:
					tagX = 425
					tagY = 125
				if msg.tagNum == 4:
					tagX = 425
					tagY = 325
				if msg.tagNum == 5:
					tagX = 425
					tagY = 525
				normalization = 0.0
				for x in range (0,35): #no of cells in x 35
					for y in range (0,35):	 #no of cells in y 35 
						for z in range(0,36): #no of cells in z - 90deg discretization - 4 
							pose_cell = Pose2D()
							pose_cell.x = x*20.0  + 10.0
							pose_cell.y = y*20.0 + 10.0
							pose_cell.theta = z*(math.pi/18.0)
							#print tagY
							#print tagX
							requiredObvRange = math.sqrt((tagY - pose_cell.y)**2 + (tagX - pose_cell.x)**2 )
							line_angle = math.atan2(tagY - pose_cell.y , tagX - pose_cell.x)
							if line_angle < 0:
								line_angle = math.pi + (math.pi - math.fabs(line_angle))	
							requiredObvBearing = self.getRequiredRotation(line_angle,pose_cell.theta)
							errorRange = (msg.range*100.0) - requiredObvRange
							#print "---Range Error--- : " + str(errorRange)
							actualBearing= tf.transformations.euler_from_quaternion([msg.bearing.x, msg.bearing.y, msg.bearing.z,msg.bearing.w])
							'''if actualBearing[2] < 0:
								actualBearing[2] = math.pi + (math.pi - math.fabs(actualBearing[2]))'''
							errorBearing = actualBearing[2] - requiredObvBearing 
							#print "---Bearing Error--- : " + str(errorBearing)
							
							PerrorRange = (1.0/(math.sqrt(10.0) *math.sqrt(2*math.pi))) * np.exp(-((errorRange**2) / (2.0*10.0)))
							PerrorBearing = (1.0/(math.sqrt(math.pi/36) *math.sqrt(2*math.pi))) * np.exp(-((errorBearing**2) / (2.0*(math.pi/36)))) 
							#print PerrorRange
							#print PerrorBearing	
							self.gridCellProbabilites[x,y,z] = PerrorRange * PerrorBearing * self.gridCellProbabilites[x,y,z] 
							#normalization +=  self.gridCellProbabilites[x,y,z]
				self.gridCellProbabilites = self.gridCellProbabilites / np.sum(self.gridCellProbabilites)
				'''for x in range (0,35): #no of cells in x 35
					for y in range (0,35):	 #no of cells in y 35 
						for z in range(0,36): #no of cells in z - 90deg discretization - 4 	
							f.write(str(x) +" " +  str(y) +" " + str(z) + " " + str(self.gridCellProbabilites[x,y,z]) + "\n") '''
				max_prob_loc =  np.unravel_index(np.argmax(self.gridCellProbabilites),self.gridCellProbabilites.shape)
				print max_prob_loc 
				print np.max(self.gridCellProbabilites)	
				print msg.tagNum
				self.msgRviz = Marker()		
				self.pointId = self.pointId + 1
				self.msgRviz.header.frame_id = "base_link"	    
				self.msgRviz.header.stamp = rospy.Time.now()
				self.msgRviz.type = Marker.LINE_LIST
				self.msgRviz.pose.orientation.w = 1.0
				self.msgRviz.ns= "line_strip"
				self.msgRviz.id = self.pointId
				self.msgRviz.scale.x = 0.1
				self.msgRviz.color.a = 1.0
				self.msgRviz.color.b = 1.0	
				p = Point()
				p.x = max_prob_loc[0]*.20 + .10
				p.y = max_prob_loc[1]*.20 + .10
				p.z = 0
				self.msgRviz.points.append(self.p1)
				self.msgRviz.points.append(p)
				self.pub.publish(self.msgRviz)
				self.p1 = p
				self.plotTags()		
				#raw_input("Press Enter to continue...")


				
		bag.close()
		#f.close()
        	'''r = rospy.Rate(10)
		try:
        		while not rospy.is_shutdown():
				#rospy.loginfo("ok")				
        	    		r.sleep()
		except rospy.ROSInterruptException:
			pass'''

if __name__ == '__main__':
	obj = localization()
	obj.run()
	#print obj.getRequiredRotationDeg(320,340)
	#print math.atan2(4,0)
	#print obj.findMinimumError(15,30,9)
		
