#!/usr/bin/env python
# --*-- coding: utf-8 -*-

import cv2 as cv
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from MG400_autocalib.msg import Coordinate
import rospy
import cv_bridge
import numpy as np
from bringup.srv import MovJ, DO
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

class MOVE:
	def __init__(self):
		print("start MG400")
		self.arm_move =rospy.ServiceProxy('/bringup/srv/MovJ',MovJ)
		self.suction = rospy.ServiceProxy('/bringup/srv/DO', DO)
		self.sub = rospy.Subscriber("/autocalib/coordinate", Coordinate, self.move_callback)
		self.pub_move = rospy.Publisher("/autocalib/move", Int16, queue_size=10)
		self.hz = 20
                self.RUN = 0
                self.TIMEOUT = 0.5
		rate = rospy.Rate(self.hz)
		self.last_clb_time_ = rospy.get_time()
		while not rospy.is_shutdown():
			if self.RUN == 1:
				time_duration = rospy.get_time() - self.last_clb_time_
				if time_duration < self.TIMEOUT:
					self.move_loop()

				rate.sleep()


	def move_callback(self, msg):
		print(msg.x, msg.y)


	def clbk_start_service(self,req):
		print("start movement ")
		self.RUN = 1
		first_move = self.arm_move(200, 0, 0, 0, 0, 0)
                self.setDefault()
		return EmptyResponse()


	def clbk_stop_service(self,req):
		print("stop movement")
		self.RUN = 0
		return EmptyResponse()
	
	def move_loop(self):
		self.last_clb_time_ = rospy.get_time()
		print("move_loop")


if __name__ == "__main__":
	print("MG400_work start")
	rospy.init_node('MG400_work')
	mv = MOVE()
	rospy.spin()
