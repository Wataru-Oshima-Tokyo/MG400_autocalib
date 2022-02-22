#!/usr/bin/env python
# --*-- coding: utf-8 -*-

import cv2 as cv
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from camera_pkg.msg import Coordinate
import rospy
import cv_bridge
import numpy as np
from bringup.srv import MovJ, DO, EnableRobot, DisableRobot
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
import time
class MOVE:
	def __init__(self):
		print("start MG400")
		self.arm_move =rospy.ServiceProxy('/bringup/srv/MovJ',MovJ)
		self.arm_enable = rospy.ServiceProxy('/bringup/srv/EnableRobot',EnableRobot)
                self.arm_disable = rospy.ServiceProxy('/bringup/srv/DisableRobot',DisableRobot)
		self.suction = rospy.ServiceProxy('/bringup/srv/DO', DO)
		self.sub = rospy.Subscriber("/camera_pkg/coordinate", Coordinate, self.move_callback)
		self.pub_move = rospy.Publisher("/autocalib/move", Int16, queue_size=10)
		self.start_srv_ = rospy.Service('/autocalib_work/start', Empty, self.clbk_start_service)
		self.stop_srv_ = rospy.Service('/autocalib_work/stop', Empty, self.clbk_stop_service)
		self.hz = 20
                self.RUN = 0
                self.TIMEOUT = 0.5
		rate = rospy.Rate(self.hz)
		self.last_clb_time_ = rospy.get_time()
		self.pos_x =0
		self.pos_y =0
		while not rospy.is_shutdown():
			if self.RUN == 1:
				time_duration = rospy.get_time() - self.last_clb_time_
				if time_duration < self.TIMEOUT:
					#self.move_loop()
                                        pass
				rate.sleep()

	def move_callback(self, msg):
		#initial postion for MG400 in image coordinate is 566(x),145(y) and robot coordination is (300, 0)
		#from left to center, (332,145) and robot (300, 113)
		if msg.t =="L":
			self.pos_y= msg.x /3
			self.arm_move(self.pos_x, self.pos_y, 0, 0, 0, 0)
		elif msg.t =="R":
			self.pos_y-= -msg.x /3
			self.arm_move(self.pos_x, self.pos_y, 0, 0, 0, 0)
		elif msg.t =="M":
			self.pos_x=300
			self.pos_y=0
			self.arm_move(self.pos_x, self.pos_y, 0, 0, 0, 0)

		# self.arm_move(pos_x, pos_y, 0, 0, 0, 0)
        	time.sleep(0.1)
		self.last_clb_time_ = rospy.get_time()


	def clbk_start_service(self,req):
		print("start movement ")
		self.RUN = 1
                self.arm_enable()
		self.pos_x =300
		first_move = self.arm_move(self.pos_x, 0, 0, 0, 0, 0)
		time.sleep(1)
                self.last_clb_time_ = rospy.get_time()
		return EmptyResponse()


	def clbk_stop_service(self,req):
		print("stop movement")
		self.RUN = 0
                self.arm_disable()
		return EmptyResponse()
	
	def move_loop(self):
                init_pos_x = 200
                init_pos_y = 0
                for i in range(10):
                    pos_x = init_pos_x + 10*i
                    pos_y = init_pos_y + 10*i
                    self.arm_move(pos_x, pos_y, 0, 0, 0, 0)
                    time.sleep(1)
                self.last_clb_time_ = rospy.get_time()



if __name__ == "__main__":
	print("MG400_work start")
	rospy.init_node('MG400_work')
	mv = MOVE()
	rospy.spin()
