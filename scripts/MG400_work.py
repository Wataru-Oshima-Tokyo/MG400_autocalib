#!/usr/bin/env python
# --*-- coding: utf-8 -*-

import cv2 as cv
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from camera_pkg.msg import Coordinate
import rospy
import cv_bridge
import numpy as np
from bringup.srv import MovJ, DO, EnableRobot, DisableRobot
from bringup.msg import ToolVectorActual
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from sklearn.linear_model import LinearRegression
import time
import math
class MOVE:
	def __init__(self):
		print("start MG400")
		self.filepath ="/home/woshima/catkin_ws/src/MG400_autocalib/calibration.txt"
		self.arm_move =rospy.ServiceProxy('/bringup/srv/MovJ',MovJ)
		self.arm_enable = rospy.ServiceProxy('/bringup/srv/EnableRobot',EnableRobot)
		self.robot_coordinate = rospy.Subscriber("/bringup/msg/ToolVectorActual", ToolVectorActual, self.robotCoordinate_callback)
                self.arm_disable = rospy.ServiceProxy('/bringup/srv/DisableRobot',DisableRobot)
		self.suction = rospy.ServiceProxy('/bringup/srv/DO', DO)
		self.sub = rospy.Subscriber("/camera_pkg/coordinate", Coordinate, self.image_callback)
		self.pub_move = rospy.Publisher("/autocalib/move", Int16, queue_size=10)
		self.work_start_srv_ = rospy.Service('/mg400_work/start', Empty, self.work_start_service)
		self.twist_pub = rospy.Subscriber('/cmd_vel', Twist, self.twist_callback)
		self.work_stop_srv_ = rospy.Service('/mg400_work/stop', Empty, self.work_stop_service)
		self.calib_start_srv = rospy.Service('/calibration/start', Empty, self.calib_start_service)
		self.calib_stop_srv = rospy.Service('/calibration/stop', Empty, self.calib_stop_service)
		self.sub_jointState = rospy.Subscriber('/bringup/srv/ok', Twist, self.twist_callback)
		self.camera_coordinate =np.array([[]])
		self.calib = False
		self.hz = 20
                self.RUN = 0
                self.TIMEOUT = 0.5
		rate = rospy.Rate(self.hz)
		self.last_clb_time_ = rospy.get_time()
		self.x_r =0
		self.y_r =0
		self.x_i =0
		self.y_i =0
		self.x_r_arr =[]
		self.y_r_arr =[]
		self.readCalibFile()

		while not rospy.is_shutdown():
			if self.RUN == 1:
				time_duration = rospy.get_time() - self.last_clb_time_
				if time_duration < self.TIMEOUT:
					#self.move_loop()
                                        pass
				rate.sleep()

	def readCalibFile(self):
		try:
			with open(self.filepath,"r") as file:	
				temp = file.read()
			temp = temp.split(",")
			print(temp)
			self.xx_coefficient = float(temp[0])
			self.xy_coefficient = float(temp[1])
			self.x_intercept = float(temp[2])
			self.yx_coefficient = float(temp[3])
			self.yy_coefficient = float(temp[4])
			self.y_intercept = float(temp[5])
		except:
			self.xx_coefficient = 0
			self.xy_coefficient = 0
			self.x_intercept = 0
			self.yx_coefficient = 0
			self.yy_coefficient = 0
			self.y_intercept = 0
			

	def robotCoordinate_callback(self, coordinate):
		self.x_r = coordinate.x
		self.y_r = coordinate.y

	def twist_callback(self, msg):
		if msg.angular.z>0:
			self.y_r += msg.linear.x
		else:
			self.y_r -= msg.linear.x
		self.y_r += msg.angular.z
		self.arm_move(self.x_r, self.y_r, 0, 0, 0, 0)

	def image_callback(self, msg):
		#initial postion for MG400 in image coordinate is 566(x),145(y) and robot coordination is (300, 0)
		#from left to center, (332,145) and robot (300, 113)
		if self.calib:
			if msg.t =="L":
				self.x_i = msg.x
				self.y_i = msg.y
				self.addCoordinate()
			elif msg.t =="R":
				self.cancelAppend()
			elif msg.t =="M":
				self.calibration()
				pass
		else:
			if msg.t =="L":
				x_a = msg.x*self.xx_coefficient + msg.y*self.xy_coefficient + self.x_intercept
				y_a = msg.x*self.yx_coefficient + msg.y*self.yy_coefficient + self.y_intercept
				self.arm_move(x_a,y_a, 0, 0, 0, 0)
			elif msg.t =="R":
				self.cancelAppend()
			elif msg.t =="M":
				self.calibration()
				pass
		

		self.last_clb_time_ = rospy.get_time()

	def cancelAppend(self):
		print("cancel")
		# self.x_i_arr.pop(-1)
		# self.y_i_arr.pop(-1)
		# self.x_r_arr.pop(-1)
		# self.y_r_arr.pop(-1)

	def addCoordinate(self):
		
		pose =np.array([[self.x_i, self.y_i]])
		if self.camera_coordinate.size ==0:
			self.camera_coordinate = pose
		else:
			self.camera_coordinate = np.append(self.camera_coordinate, pose, axis=0)
			print(self.camera_coordinate)
		self.x_r_arr.append(self.x_r)
		self.y_r_arr.append(self.y_r)

	def calibration(self):
		self.x_coefficient = LinearRegression().fit(self.camera_coordinate, self.x_r_arr)
		self.y_coefficient = LinearRegression().fit(self.camera_coordinate, self.y_r_arr)
		print(self.x_coefficient.coef_, self.x_coefficient.intercept_ , self.y_coefficient.coef_, self.y_coefficient.intercept_)
		with open(self.filepath,"w+") as file:
			file.write(str(self.x_coefficient.coef_[0])+',')
			file.write(str(self.x_coefficient.coef_[1])+',')
			file.write(str(self.x_coefficient.intercept_)+',')
			file.write(str(self.y_coefficient.coef_[0])+',')
			file.write(str(self.y_coefficient.coef_[1])+',')
			file.write(str(self.y_coefficient.intercept_))
		self.readCalibFile()
		



	def work_start_service(self,req):
		print("start movement ")
		self.RUN = 1
                self.arm_enable()
		self.pos_x =300
		first_move = self.arm_move(self.pos_x, 0, 0, 0, 0, 0)
		time.sleep(1)
                self.last_clb_time_ = rospy.get_time()
		return EmptyResponse()


	def work_stop_service(self,req):
		print("stop movement")
		self.RUN = 0
                self.arm_disable()
		return EmptyResponse()
	
	def calib_start_service(self,req):
		print("start calibration")
		self.calib = True
		return EmptyResponse()
	
	def calib_stop_service(self,req):
		print("stop calibration")
		self.calib = False
		return EmptyResponse()


if __name__ == "__main__":
	print("MG400_work start")
	rospy.init_node('MG400_work')
	mv = MOVE()
	rospy.spin()
