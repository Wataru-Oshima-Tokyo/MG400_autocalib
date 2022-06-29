#!/usr/bin/env python
# --*-- coding: utf-8 -*-
import os
import cv2 as cv
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from camera_pkg_msgs.msg import Coordinate
import rospy
import cv_bridge
import numpy as np
from mg400_bringup.srv import MovJ, DO, EnableRobot, DisableRobot,Sync, ClearError,JointMovJ
from mg400_bringup.msg import ToolVectorActual
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from sklearn.linear_model import LinearRegression
import time
import math
class MOVE:
	def __init__(self):
		print("start MG400")
                home = os.environ["HOME"]
		self.xy_filepath = home + "/catkin_ws/src/MG400_basic/xy_calibration.txt"
		self.z_filepath = home + "/catkin_ws/src/MG400_basic/z_calibration.txt"
		self.arm_move =rospy.ServiceProxy('/mg400_bringup/srv/MovJ',MovJ)
		self.suction =rospy.ServiceProxy('/mg400_bringup/srv/DOExecute',DO)
		self.sync =rospy.ServiceProxy('/mg400_bringup/srv/Sync',Sync)
		self.arm_enable = rospy.ServiceProxy('/mg400_bringup/srv/EnableRobot',EnableRobot)
		self.robot_coordinate = rospy.Subscriber("/mg400_bringup/msg/ToolVectorActual", ToolVectorActual, self.robotCoordinate_callback)
                self.arm_disable = rospy.ServiceProxy('/mg400_bringup/srv/DisableRobot',DisableRobot)
		self.clear_error = rospy.ServiceProxy('/mg400_bringup/srv/ClearError',ClearError)
		self.joint_move = rospy.ServiceProxy('/mg400_bringup/srv/ClearError',JointMovJ)
		self.sub = rospy.Subscriber("/camera_pkg/coordinate", Coordinate, self.image_callback)
		self.pub_move = rospy.Publisher("/autocalib/move", Int16, queue_size=10)
		self.work_start_srv_ = rospy.Service('/mg400_work/start', Empty, self.work_start_service)
		self.twist_pub = rospy.Subscriber('/cmd_vel', Twist, self.twist_callback)
		self.work_stop_srv_ = rospy.Service('/mg400_work/stop', Empty, self.work_stop_service)
		self.xy_calib_start_srv = rospy.Service('/xy_calibration/start', Empty, self.xy_calib_start_service)
		self.xy_calib_stop_srv = rospy.Service('/xy_calibration/stop', Empty, self.xy_calib_stop_service)
		self.z_calib_start_srv = rospy.Service('/z_calibration/start', Empty, self.z_calib_start_service)
		self.z_calib_stop_srv = rospy.Service('/z_calibration/stop', Empty, self.z_calib_stop_service)
		self.sub_jointState = rospy.Subscriber('/mg400_bringup/srv/ok', Twist, self.twist_callback)
		self.camera_coordinate =np.array([[]])
		self.xy_calib = False
		self.z_calib = False
		self.hz = 20
		self.camera_z = np.array([[]])
                self.RUN = 0
                self.TIMEOUT = 0.5
		self.count=0
		self.place_y=-352
		self.place_x=0
		rate = rospy.Rate(self.hz)
		self.last_clb_time_ = rospy.get_time()
		self.x_r =0
		self.y_r =0
		self.z_r =0
		self.pre_x_r=0
		self.pre_y_r=0
		self.pre_z_r=0
		 
		self.temp_x_r = 0
		self.temp_y_r = 0
		self.temp_z_r = 0
		self.x_i =0
		self.y_i =0
		self.z_i =0
		self.x_r_arr =[]
		self.y_r_arr =[]
		self.z_r_arr =[]
		self.x_r_intercept = 0
		self.y_r_intercept = 0
		self.z_r_coefficient = 0
		self.z_r_intercept = 0
		self.x_r_coefficient = [0,0,0]
		self.y_r_coefficient = [0,0,0]
		self.readCalibFile()
		self.initialize()

	def initialize(self):
		self.arm_disable()
		self.clear_error()
		self.arm_enable()
		self.joint_move(0,0,0,0)

	def readCalibFile(self):
		try:
			with open(self.xy_filepath,"r") as file:	
				line = file.read()
			line = line.split("\n")
			print(line)
			for i in range(len(line)):
				word = line[i].split(",")
				print(word)
				if i==0:
					for j in range(len(word)-1):
						self.x_r_coefficient[j] = float(word[j])
					self.x_r_intercept = float(word[-1])
				elif i ==1:
					for j in range(len(word)-1):
						self.y_r_coefficient[j] = float(word[j])
					self.y_r_intercept = float(word[-1])
				else:
					pass
			with open(self.z_filepath,"r") as file:
					line = file.read()
			line = line.split(",")
			print(line)
                        for i in range(len(line)):
				self.z_r_coefficient = float(line[0])
				self.z_r_intercept = float(line[1])
			print("done reading calib file ")
		except Exception as e:
			print(e)
			
	def getRobotCoordinate(self):
		self.x_r = self.temp_x_r
		self.y_r = self.temp_y_r
		self.z_r = self.temp_z_r

	def robotCoordinate_callback(self, coordinate):
		self.temp_x_r = coordinate.x
		self.temp_y_r = coordinate.y
		self.temp_z_r = coordinate.z

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
		if self.xy_calib or self.z_calib:
			if msg.t =="L":
                                #self.arm_enable()
				self.getRobotCoordinate()
				#self.arm_move(self.x_r, self.y_r, 0, 0, 0, 0)
				#self.arm_move(4.20, 250, 30, 0, 0, 0)
				self.pre_x_r=self.x_r
				self.pre_y_r=self.y_r
				self.pre_z_r=self.z_r
			elif msg.t =="R":
				self.x_i = msg.x
				self.y_i = msg.y
				self.z_i = msg.z
				self.addCoordinate()
				#self.arm_move(self.pre_x_r, self.pre_y_r, self.pre_z_r+20, 0, 0, 0)
                                #self.arm_disable()
			elif msg.t =="M":
# 				self.cancelAppend()
				self.calibration()
				pass
		else:
			if msg.t =="L":
				msgs = [msg.x, msg.y, msg.z]
				x_a, y_a =0,0
				for i in range(len(self.x_r_coefficient)):
					x_a += msgs[i]*self.x_r_coefficient[i]
					y_a += msgs[i]*self.y_r_coefficient[i]
				x_a += self.x_r_intercept
				y_a += self.y_r_intercept
				z_a = msg.z*self.z_r_coefficient + self.z_r_intercept
				z_move = 60
                                # z_a = -14
                                # x_a = msg.x*self.xx_coefficient + msg.y*self.xy_coefficient + msg.z*self.xz_coefficient +self.x_intercept
				# y_a = msg.x*self.yx_coefficient + msg.y*self.yy_coefficient + msg.z*self.yz_coefficient+self.y_intercept
				_r=0
				self.arm_move(x_a,y_a, z_move, _r, 0, 0)
				self.arm_move(x_a,y_a,z_a, _r, 0, 0)
				time.sleep(7)
				self.suction(1,1)
				time.sleep(0.5)
				self.arm_move(x_a,y_a,z_move, _r, 0, 0)
				self.arm_move(self.place_x,self.place_y,z_move, _r, 0, 0)
				self.arm_move(self.place_x,self.place_y,-150, _r, 0, 0)
				time.sleep(13)
				self.suction(1,0)
				time.sleep(0.5)
				self.suction(2,1)
				time.sleep(1)
				self.suction(2,0)
				time.sleep(1)
				self.arm_move(-4,-250,z_move, _r, 0, 0)
				time.sleep(3)

			elif msg.t =="R":
				self.xy_calib_start_service(Empty)
			elif msg.t =="M":
				self.z_calib_start_service(Empty)
			self.count+=1
			self.place_x += 40
			if self.count%3 ==0:
				self.place_y +=40
				self.place_x=0 
			

		self.last_clb_time_ = rospy.get_time()

	def cancelAppend(self):
		print("cancel")


	def addCoordinate(self):
		if self.x_r != 0  and self.y_r !=0 and self.z_r !=0:
			pose =np.array([[self.x_i, self.y_i, self.z_i]])
			#pose =np.array([[self.x_i, self.y_i]])
			z_coordiante = np.array([self.z_i])
			if self.camera_coordinate.size ==0:
				self.camera_coordinate = pose
				self.camera_z = z_coordiante
			else:
				self.camera_coordinate = np.append(self.camera_coordinate, pose, axis=0)
				self.camera_z = np.append(self.camera_z, z_coordiante)
			self.x_r_arr.append(self.x_r)
			self.y_r_arr.append(self.y_r)
			self.z_r_arr.append(self.z_r)
		else:
			pass
		
	def calibration(self):
		self.camera_z = self.camera_z.reshape(-1,1)
		print(self.camera_z)
		self.x_coefficient = LinearRegression().fit(self.camera_coordinate, self.x_r_arr)
		self.y_coefficient = LinearRegression().fit(self.camera_coordinate, self.y_r_arr)
		self.z_coefficient = LinearRegression().fit(self.camera_z, self.z_r_arr)
		print(self.x_coefficient.coef_, self.x_coefficient.intercept_ , self.y_coefficient.coef_, self.y_coefficient.intercept_, self.z_coefficient.coef_, self.z_coefficient.intercept_)
		if self.xy_calib:
			with open(self.xy_filepath,"w+") as file:
				for i in range(len(self.x_coefficient.coef_)):
					file.write(str(self.x_coefficient.coef_[i])+',')
				file.write(str(self.x_coefficient.intercept_)+'\n')
				for i in range(len(self.y_coefficient.coef_)):
					file.write(str(self.y_coefficient.coef_[i])+',')
				file.write(str(self.y_coefficient.intercept_)+'\n')
				file.write(str(self.z_coefficient.coef_[0])+',')
				file.write(str(self.z_coefficient.intercept_))
			self.xy_calib_stop_service(Empty)
		elif self.z_calib:
			with open(self.z_filepath,"w+") as file:
				file.write(str(self.z_coefficient.coef_[0])+',')
				file.write(str(self.z_coefficient.intercept_))	
			self.z_calib_stop_service(Empty)	
		self.readCalibFile()

		self.camera_coordinate =np.array([[]])
		self.camera_z = np.array([[]])
		self.x_r_arr =[]
		self.y_r_arr =[]
		self.z_r_arr =[]

		



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
	
	def xy_calib_start_service(self,req):
		print("start xy_alibration")
		self.xy_calib = True
		return EmptyResponse()
	
	def xy_calib_stop_service(self,req):
		print("stop xy_calibration")
		self.xy_calib = False
		self.camera_coordinate =np.array([[]])
		self.x_r_arr =[]
		self.y_r_arr =[]
		return EmptyResponse()
	
	def z_calib_start_service(self,req):
		print("start z_calibration")
		self.z_calib = True
		return EmptyResponse()
	
	def z_calib_stop_service(self,req):
		print("stop z_calibration")
		self.z_calib = False
		self.camera_z = np.array([[]])
		self.z_r_arr =[]
		return EmptyResponse()


if __name__ == "__main__":
	print("MG400_work start")
	rospy.init_node('MG400_work')
	mv = MOVE()
	rospy.spin()
# 	mv.work_stop_service(Empty)
