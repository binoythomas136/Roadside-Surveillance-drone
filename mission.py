#!/usr/bin/env python
#import statements
from std_msgs.msg import String
import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped,Vector3Stamped
import numpy as np
from math import sin,cos,radians,atan2,sqrt,pi
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, ParamSet
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate, StreamRateRequest
from sensor_msgs.msg import NavSatFix
import time
from tf.transformations import euler_from_quaternion, euler_matrix, quaternion_from_euler
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import rospy
import cv2
#from __future__ import print_function
import numpy as np
import roslib
from subscribers import Subscribers
from computations import Computations
from commands import Commands
from navigation import Navigation
from arrow_detection import Arrow
from road_following import RoadFollow

#Create a class for the mission
class Mission():
	def __init__(self):
		self.subs=Subscribers()						#initialize streaming of subscribers
		self.comp=Computations()			
		self.commands=Commands()
		self.navigation=Navigation()
		self.arrow=Arrow()						#object for arrow detection class
		self.road_follow=RoadFollow()					#object for road detection class
		#initialize the publishers
		self.setpoint_publisher = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
	    	self.setvel_publisher = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size = 10)
	    	self.setaccel_publisher = rospy.Publisher("/mavros/setpoint_accel/accel",Vector3Stamped,queue_size=10)
		print('Publishers initialized')



	#main function for the drone mission
	def way_mission(self):
		rate=rospy.Rate(10)						#initialize the rate 
		self.comp.send_arb_waypoints()					#send arbitary number of waypoints for changing to offboard mode
		self.commands.set_mode('OFFBOARD')				#set mode to offboard
		self.commands.arm()						# arm the drone
		#drone takeoff		
		self.navigation.waypoint(self.subs.pose.pose.position.x,self.subs.pose.pose.position.y,10)
		#self.commands.set_mode('AUTO.TAKEOFF')
		rate.sleep()
		

		'''
		#arrow detection
		

		delta_orientation=1000						#initialize the change in orientation
		self.commands.set_mode('AUTO.LOITER')				#change mode to hold mode
		rate.sleep()	
		cv2.imwrite('Images123/image1.jpg', self.subs.cv_image )
		while(delta_orientation ==1000):				#loop to wait for drone to see the arrow
			print(self.subs.state.mode)
			#failsafe		
			if(self.subs.state.mode != 'AUTO.LOITER' and self.subs.state.mode != 'OFFBOARD'):
				return
			delta_orientation=self.arrow.arrow_angle(self.subs.cv_image)#get change in orientation 
		self.comp.send_arb_waypoints()
		self.commands.set_mode('OFFBOARD')				#move to offboard mode
		print(delta_orientation)
		
		self.comp.change_orientation(delta_orientation)			#rotate the drone to the desired arrow orientation
		rate.sleep()
	
		cv2.imwrite('Images123/image.jpg', self.subs.cv_image)		#save image for scrutiny
		rate.sleep()
		

		
		#road following
		self.delta=1000							#initialize the next waypoint angle
		#self.commands.set_mode('AUTO.LOITER')				#change later after making road following robust
		#rate.sleep()
		while(self.delta==1000):					#loop to go through to check if image is received
			#failsafe			
			if(self.subs.state.mode != 'AUTO.LOITER' and self.subs.state.mode != 'OFFBOARD'):
				return			
			cv2.imwrite('Images123/image.jpg', self.subs.cv_image)	#save image for scrutiny
			self.delta=self.road_follow.getRoadAngle(self.subs.cv_image)#get the change in the road's orientation
			print(self.delta)					
			
			
			
					
		#self.comp.send_arb_waypoints()
		#self.commands.set_mode('OFFBOARD')				#change later after making road following robust
		self.comp.road_next_waypoint(self.delta)			#compute the next waypoint and the orientation
		'''		
		self.commands.land()						#land the drone
		rate.sleep()							
		self.commands.disarm()						#disarm the drone

	def main(self):
		rospy.init_node("test_mission",log_level=rospy.INFO)		#initialize the pixhawk node
		print('node initialized')
		rate = rospy.Rate(10)
		while(1):		
			if (self.subs.state.mode == 'POSCTL'): 			#failsafe
				self.way_mission()				#call the main mission
				break
			else:
				continue

if __name__== '__main__':
	
	mission=Mission()							#make object of main mission class
	mission.main()								#call main function

