#!/usr/bin/env python
# --*-- coding: utf-8 -*-

import cv2 as cv
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from camera_pkg_msgs.msg import Coordinate
import rospy
import cv_bridge
import numpy as np
from mg400_bringup.srv import MovJ, DO, EnableRobot, DisableRobot, SpeedJ, AccJ, Sync, ClearError,JointMovJ
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
		self.set_SpeedJ =rospy.ServiceProxy('/mg400_bringup/srv/SpeedJ',SpeedJ)
		self.set_AccJ =rospy.ServiceProxy('/mg400_bringup/srv/AccJ',AccJ)
		self.arm_enable = rospy.ServiceProxy('/mg400_bringup/srv/EnableRobot',EnableRobot)
                self.linetrace_stop = rospy.ServiceProxy('/linetrace/stop',Empty)
                self.linetrace_start = rospy.ServiceProxy('/linetrace/start',Empty)
		self.robot_coordinate = rospy.Subscriber("/mg400_bringup/msg/ToolVectorActual", ToolVectorActual, self.robotCoordinate_callback)
                self.arm_disable = rospy.ServiceProxy('/mg400_bringup/srv/DisableRobot',DisableRobot)
		self.suction = rospy.ServiceProxy('/mg400_bringup/srv/DO', DO)
		self.clear_error = rospy.ServiceProxy('/mg400_bringup/srv/ClearError',ClearError)
		self.joint_move = rospy.ServiceProxy('/mg400_bringup/srv/JointMovJ',JointMovJ)
		self.sub = rospy.Subscriber("/objectdetection/coordinate", Coordinate, self.image_callback)
		self.pub_move = rospy.Publisher("/autocalib/move", Int16, queue_size=10)
		self.work_start_srv_ = rospy.Service('/mg400_work/start', Empty, self.work_start_service)
                self.twist_pub = rospy.Subscriber('/MG400/cmd_vel', Twist, self.twist_callback)
		self.work_stop_srv_ = rospy.Service('/mg400_work/stop', Empty, self.work_stop_service)
		self.calib_start_srv = rospy.Service('/calibration/start', Empty, self.calib_start_service)
		self.calib_stop_srv = rospy.Service('/calibration/stop', Empty, self.calib_stop_service)
		self.sub_jointState = rospy.Subscriber('/mg400_bringup/srv/ok', Twist, self.twist_callback)
		self.camera_coordinate =np.array([[]])
		self.now = time.time()
		self.end = time.time()
		self.calib = False
		self.hz = 20
                self.move_stopper= True
		self.camera_z = np.array([[]])
                self.RUN = 0
		self.place_y=-352
		self.place_x=0
                self.TIMEOUT = 0.5
		rate = rospy.Rate(self.hz)
		self.last_clb_time_ = rospy.get_time()
		self.x_r =0
		self.y_r =0
		self.z_r =0
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
		self.set_SpeedJ(60)
		self.set_AccJ(100)
		self.initialize()
		self.arm_move(4.20, -250, 30,0)

	def initialize(self):
		self.arm_disable()
		self.clear_error()
		self.arm_enable()
		time.sleep(0.5)
		# self.joint_move(0,0,0,0)


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
			

			

	def robotCoordinate_callback(self, coordinate):
		self.x_r = coordinate.x
		self.y_r = coordinate.y
		self.z_r = coordinate.z

	def twist_callback(self, msg):
		if msg.angular.z>0:
			self.y_r += msg.linear.x
		else:
			self.y_r -= msg.linear.x
		self.y_r += msg.angular.z
		self.arm_move(self.x_r, self.y_r, 0)

        def image_callback(self, msg):
# 		print("get the message")
		self.now = time.time()
# 		print("now ", self.now)
# 		print("end ", self.end)
		if  (msg.x !=0 and msg.y !=0):
			if self.end > self.now:
				try:
					self.linetrace_stop()
				except:
					pass
			else:
				try:
					self.linetrace_stop()
				except:
					pass
				self.end = time.time() +30
				self.initialize()
				self.arm_move(300, 0, 30, 0)
				msgs = [msg.x, msg.y, msg.z]
				x_a, y_a =0,0
				for i in range(len(self.x_r_coefficient)):
				    x_a += msgs[i]*self.x_r_coefficient[i]
				    y_a += msgs[i]*self.y_r_coefficient[i]
				x_a += self.x_r_intercept
				y_a += self.y_r_intercept
				z_a = msg.z*self.z_r_coefficient + self.z_r_intercept
				# z_a = -14
                                z_move=60
				_r =0
				self.arm_move(x_a,y_a,z_move , _r)
				self.arm_move(x_a,y_a,z_a, _r)
				time.sleep(7)
				self.suction(1,1)
				time.sleep(1)
				self.suction(1,0)
				self.arm_move(x_a,y_a,z_move, _r)
				self.arm_move(self.place_x,self.place_y,z_move, _r)
				self.arm_move(self.place_x,self.place_y,-150, _r)
				time.sleep(13)
				self.suction(2,1)
				time.sleep(1)
				self.suction(2,0)
				time.sleep(1)
				self.arm_move(-4,-250,z_move, _r)
				time.sleep(3)

		# self.last_clb_time_ = rospy.get_time()

	def cancelAppend(self):
		print("cancel")


	def addCoordinate(self):
		if self.x_r != 0  and self.y_r !=0 and self.z_r !=0:
			pose =np.array([[self.x_i, self.y_i, self.z_i]])
			# pose =np.array([[self.x_i, self.y_i]])
			z_coordiante = np.array([self.z_i])
			if self.camera_coordinate.size ==0:
				self.camera_coordinate = pose
				self.camera_z = z_coordiante
			else:
				self.camera_coordinate = np.append(self.camera_coordinate, pose, axis=0)
				self.camera_z = np.append(self.camera_z, z_coordiante)
				print(self.camera_coordinate)
				print(self.camera_z)
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
		with open(self.filepath,"w+") as file:
			for i in range(len(self.x_coefficient.coef_)):
				file.write(str(self.x_coefficient.coef_[i])+',')
			file.write(str(self.x_coefficient.intercept_)+'\n')
			for i in range(len(self.y_coefficient.coef_)):
				file.write(str(self.y_coefficient.coef_[i])+',')
			file.write(str(self.y_coefficient.intercept_)+'\n')
			file.write(str(self.z_coefficient.coef_[0])+',')
			file.write(str(self.z_coefficient.intercept_))
		self.readCalibFile()
		



	def work_start_service(self,req):
		print("start movement ")
		self.RUN = 1
                self.arm_enable()
		self.pos_x =300
		first_move = self.arm_move(self.pos_x, 0, 0, 0)
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
