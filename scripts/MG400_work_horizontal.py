#!/usr/bin/env python
# --*-- coding: utf-8 -*-
import os
import cv2 as cv
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from camera_pkg_msgs.msg import Coordinate
import rospy
import cv_bridge
import numpy as np
from mg400_bringup.srv import MovL, DO, EnableRobot, DisableRobot, SpeedL, AccL, Sync, ClearError,JointMovJ,SetCollisionLevel
from mg400_bringup.msg import ToolVectorActual, RobotStatus
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from sklearn.linear_model import LinearRegression
import time
import math
class MOVE:
	def __init__(self):
		print("start MG400")
                home = os.environ["HOME"]
		self.xy_filepath = home + "/catkin_ws/src/MG400_basic/files/xy_calibration_horizontal.txt"
		self.z_filepath = home + "/catkin_ws/src/MG400_basic//files/z_calibration_horizontal.txt"
		self.r_filepath = home + "/catkin_ws/src/MG400_basic//files/r_calibration_horizontal.txt"
		self.arm_move =rospy.ServiceProxy('/mg400_bringup/srv/MovL',MovL)
		self.collision_level =rospy.ServiceProxy('/mg400_bringup/srv/SetCollisionLevel',SetCollisionLevel)
		self.arm_reset =rospy.ServiceProxy('/arucodetect/reset',Empty)
		self.set_SpeedL =rospy.ServiceProxy('/mg400_bringup/srv/SpeedL',SpeedL)
		self.set_AccL =rospy.ServiceProxy('/mg400_bringup/srv/AccL',AccL)
		self.arm_enable = rospy.ServiceProxy('/mg400_bringup/srv/EnableRobot',EnableRobot)
		self.robot_coordinate = rospy.Subscriber("/mg400_bringup/msg/ToolVectorActual", ToolVectorActual, self.robotCoordinate_callback)
                self.arm_disable = rospy.ServiceProxy('/mg400_bringup/srv/DisableRobot',DisableRobot)
		self.suction = rospy.ServiceProxy('/mg400_bringup/srv/DO', DO)
		self.clear_error = rospy.ServiceProxy('/mg400_bringup/srv/ClearError',ClearError)
		#self.robot_sync = rospy.ServiceProxy('/mg400_bringup/srv/Sync',Sync)
		self.joint_move = rospy.ServiceProxy('/mg400_bringup/srv/JointMovJ',JointMovJ)
		self.sub = rospy.Subscriber("/outlet/coordinate", Coordinate, self.image_callback)
		self.mg400_dsth = rospy.Publisher("/mg400/working", Bool, queue_size=100)
		self.work_start_srv_ = rospy.Service('/mg400_work/start', Empty, self.work_start_service)
                self.twist_pub = rospy.Subscriber('/mg400/cmd_vel', Twist, self.twist_callback)
		self.robot_mode_sub = rospy.Subscriber('/mg400_bringup/msg/RobotStatus', RobotStatus, self.robotStatus_callback)
		self.work_stop_srv_ = rospy.Service('/mg400_work/stop', Empty, self.work_stop_service)
		self.xy_calib_start_srv = rospy.Service('/xy_calibration/start', Empty, self.xy_calib_start_service)
		self.xy_calib_stop_srv = rospy.Service('/xy_calibration/stop', Empty, self.xy_calib_stop_service)
		self.z_calib_start_srv = rospy.Service('/z_calibration/start', Empty, self.z_calib_start_service)
		self.z_calib_stop_srv = rospy.Service('/z_calibration/stop', Empty, self.z_calib_stop_service)
		self.sub_jointState = rospy.Subscriber('/mg400_bringup/srv/ok', Twist, self.twist_callback)
		self.camera_coordinate_x =np.array([[]])
		self.camera_coordinate_y =np.array([[]])
		self.camera_coordinate_z =np.array([[]])
		self.now = time.time()
		self.end = time.time()
		self.xy_calib = False
		self.z_calib = False
		self.robot_mode = 0
                self.robot_mode_prev = 0
		self.rate = rospy.Rate(20)
                self.move_stopper= True
		self.camera_z = np.array([[]])
                self.RUN = 0
		self.place_y=240
		self.place_x=154
                self.r_coordinate = 150
                self.TIMEOUT = 0.5
		self.ast_clb_time_ = rospy.get_time()
		self.x_a =0
		self.y_a =0
		self.z_a =0
		self.r_a =0
		self.x_r =0
		self.y_r =0
		self.z_r =0
		self.x_i =0
		self.y_i =0
		self.z_i =0
		self.r_i =0
		self.angle = 0
		self.coeficient =0.65
		self.Move = False
		self.x_r_arr =[]
		self.y_r_arr =[]
		self.z_r_arr =[]
		self.r_r_arr =[]
		self.x_r_intercept = 0
		self.y_r_intercept = 0
		self.z_r_coefficient = 0
		self.z_r_intercept = 0
		self.r_r_coefficient = 0
		self.r_r_intercept = 0
		self.x_r_coefficient = [0,0,0]
		self.y_r_coefficient = [0,0,0]
		self.readCalibFile()
		self.set_SpeedL(80)
		self.set_AccL(80)
		self.distance = 92#88
		time.sleep(2)
		self.arm_enable()
		time.sleep(2)
                while self.robot_mode == 0:
                    self.rate.sleep()
		
		self.initialize()
		time.sleep(2)
		self.arm_move(self.place_x ,self.place_y,60, self.r_coordinate)
		self.sync_robot()
		self.move_stopper = False

	def initValue(self):
		self.distance = 92#88
		self.r_coordinate = 150
		self.angle = 0

	def initialize(self):
		self.arm_disable()
		self.clear_error()
		self.arm_enable()
                time.sleep(2)
                while self.robot_mode !=5:
                    self.rate.sleep()
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
						print("coef", j,self.y_r_coefficient[j])
					print("")
					self.y_r_intercept = float(word[-1])
					print(self.y_r_intercept)
				else:
					pass
			with open(self.z_filepath,"r") as file:
					line = file.read()
			line = line.split(",")
			print(line)
                        for i in range(len(line)):
				self.z_r_coefficient = float(line[0])
				self.z_r_intercept = float(line[1])
			# for i in range(len(line)):
			# 	self.r_r_coefficient = float(line[0])
			# 	self.r_r_intercept = float(line[1])
			# print("done reading calib file ")
		except Exception as e:
			print(e)
			
	def robotStatus_callback(self, robot_status):
		"""
		    1:	"ROBOT_MODE_INIT",
			2:	"ROBOT_MODE_BRAKE_OPEN",
			3:	"",
			4:	"ROBOT_MODE_DISABLED",
			5:	"ROBOT_MODE_ENABLE",
			6:	"ROBOT_MODE_BACKDRIVE",
			7:	"ROBOT_MODE_RUNNING",
			8:	"ROBOT_MODE_RECORDING",
			9:	"ROBOT_MODE_ERROR",
			10:	"ROBOT_MODE_PAUSE",
			11:	"ROBOT_MODE_JOG"
		"""
		self.robot_mode = robot_status.robot_status

	def sync_robot(self):
			if self.robot_mode == 9:
				self.initialize()
			time.sleep(0.5)
			while self.robot_mode !=5:
				if self.robot_mode == 9:
					self.initialize()
					break
				self.rate.sleep()
			self.Move = False				
	
	
	def getRobotCoordinate(self):
		self.x_r = self.temp_x_r
		self.y_r = self.temp_y_r
		self.z_r = self.temp_z_r
		self.r_r = self.temp_r_r

	def robotCoordinate_callback(self, coordinate):
		self.temp_x_r = coordinate.x
		self.temp_y_r = coordinate.y
		self.temp_z_r = coordinate.z
		self.temp_r_r = coordinate.r

	def twist_callback(self, msg):
		x = self.temp_x_r + msg.linear.x
		y = self.temp_y_r + msg.linear.y
		z = self.temp_z_r + msg.linear.z
		r = self.r_coordinate
		if not self.Move:
			self.Move =True
			self.arm_move(x, y, z, r)
			self.sync_robot()

	def image_callback(self, msg):
		#initial postion for MG400 in image coordinate is 566(x),145(y) and robot coordination is (300, 0)
		#from left to center, (332,145) and robot (300, 113)
		print("the message is ", msg.t)
		"""
		robot_x -> camera_z
		robot_y -> camera_x
		robot_z -> camera_y 
		"""
		temp = msg.y
		msg.y = msg.x
		msg.x = msg.z
		msg.z = temp
		print("y:", msg.y)
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
				#z,y,x of image coordination correspons to x,y,z of robot coordination
				self.x_i = msg.x
				self.y_i = msg.y
				self.z_i = msg.z
				self.addCoordinate()
				#self.arm_move(self.pre_x_r, self.pre_y_r, self.pre_z_r+20, 0, 0, 0)
                                #self.arm_disable()
			elif msg.t =="M":
				print("here")
# 				self.cancelAppend()
				self.calibration()
				pass
		else:
			if msg.t =="L":
				if self.robot_mode == 9:
					self.initialize()
				msgs = [msg.x, msg.y, msg.z]  
				self.x_a, self.y_a =0,0
				# for i in range(len(self.x_r_coefficient)):
				# 	self.x_a += msgs[0]*self.x_r_coefficient[i]
				# for i in range(len(self.y_r_coefficient)):
				# 	self.y_a += msgs[1]*self.y_r_coefficient[i]
				self.x_a = msg.x*self.x_r_coefficient[0] + self.x_r_intercept
				self.y_a = msg.y*self.y_r_coefficient[0] + self.y_r_intercept
				self.z_a = msg.z*self.z_r_coefficient + self.z_r_intercept
				self._d = msg.z
				_coef =0.7
				# if self.x_a < 300:
				# 	_coef = 0.7
				self.arm_move(self.x_a*_coef,self.y_a, self.z_a-60, self.r_coordinate-msg.r)
                                # z_a = -14
                                # x_a = msg.x*self.xx_coefficient + msg.y*self.xy_coefficient + msg.z*self.xz_coefficient +self.x_intercept
				# y_a = msg.x*self.yx_coefficient + msg.y*self.yy_coefficient + msg.z*self.yz_coefficient+self.y_intercept
				
				self.sync_robot()
				# self.arm_move(x_a,y_a,z_a, _r)
				# self.sync_robot()
				self.mg400_dsth.publish(False)

			elif msg.t == "D":
				# added by the angle
				self.getRobotCoordinate()
				self.angle = msg.r * self.coeficient
				self.r_coordinate -= self.angle
				_y = self.distance * math.sin(math.radians(self.angle/self.coeficient))
				_x = self.distance -  self.distance * math.cos(math.radians(self.angle/self.coeficient)) -10
				self.arm_move(self.x_r+_x, self.y_r + _y, self.z_r, self.r_coordinate)
			elif msg.t == "A":
				self.getRobotCoordinate()
				self.angle += msg.r *self.coeficient
				self.r_coordinate -= msg.r *self.coeficient
				self.arm_move(self.x_r, self.y_r, self.z_r, self.r_coordinate)
			elif msg.t == "I":
                                self.initValue()
				self.arm_enable()
				# now_ = rospy.get_rostime()
				# time_ = rospy.get_rostime() + rospy.Time(2)
				rospy.sleep(2.)
				self.getRobotCoordinate()
				#rotate the endeffector to remove itself from the outlet
				_ex = 90
				if self.r_r >150:
					_ex *=-1
				
				self.arm_move(self.x_r- 40, self.y_r-_ex, self.z_r, self.r_r+_ex)
				self.sync_robot()
				self.arm_move(self.place_x ,self.place_y,60, self.r_coordinate)
				self.sync_robot()
			elif msg.t =="R":
				# self.xy_calib_start_service(Empty)
				self.xy_calib=True
			elif msg.t =="M":
				# self.z_calib_start_service(Empty)
				self.z_calib=True
			elif msg.t =="F":
				self.getRobotCoordinate()
				self.arm_move(self.x_r,self.y_r, self.z_r+8, self.r_coordinate)
				self.sync_robot()
				self.getRobotCoordinate()
					#115 is the dinstance from the camera to the object
				if abs(self.angle/self.coeficient) >20:
					# dis_coef = -0.00625*abs(self.angle/self.coeficient) + 1.075
					dis_coef = -0.0040625*abs(self.angle/self.coeficient) + 1.036875
					# angle_coef = -0.04041*abs(self.angle/self.coeficient) + 1.8783
					angle_coef = -0.049338*abs(self.angle/self.coeficient) + 2.1907
					print("dist_coef", dis_coef)
					print("angle_coef", angle_coef)
					d = self.distance/math.cos(math.radians(self.angle*angle_coef))
					# d = self.distance/math.cos(math.radians(self.angle))
					self.distance *= dis_coef
				else: 
					d = self.distance/math.cos(math.radians(self.angle))
				_x = self.distance* math.cos(math.radians(self.angle))
				_y = d* math.sin(math.radians(self.angle)) 
				if self.angle/self.coeficient <-30:
					self.distance *= 1.04
					# _y_coef = 0.02*(self.angle/self.coeficient) +1.2
					# _y *=_y_coef
					
				# _y = self.distance* math.sin(math.radians(self.angle)) 
				print("y_adjustment",_y)
				print("distance", self._d)
				print("angle", self.angle/self.coeficient)
				if self.x_r+self.distance <450:
					self.arm_move(self.x_r+self.distance,self.y_r-_y, self.z_r, self.r_coordinate)
				# self.arm_move(self.x_r+self.distance, self.y_r-_y, self.z_r, self.r_coordinate)
				else:
					print("out of range")
					self.arm_reset()

				self.sync_robot()
				self.arm_disable()

		self.last_clb_time_ = rospy.get_time()

	def cancelAppend(self):
		print("cancel")


	def addCoordinate(self):
		if self.x_r != 0  and self.y_r !=0 and self.z_r !=0:
			# pose =np.array([[self.x_i, self.y_i, self.z_i]])
			pose_x = np.array([[self.x_i]])
			pose_y = np.array([[self.y_i]])
			pose_z = np.array([[self.z_i]])
			# z_coordiante = np.array([self.y_i])
			if self.camera_coordinate_x.size == 0:
				self.camera_coordinate_x =pose_x
				self.camera_coordinate_y =pose_y
				self.camera_coordinate_z =pose_z
				# self.camera_coordinate = pose
				# self.camera_z = z_coordiante
			else:
				self.camera_coordinate_x = np.append(self.camera_coordinate_x, pose_x, axis=0)
				self.camera_coordinate_y = np.append(self.camera_coordinate_y, pose_y, axis=0)
				self.camera_coordinate_z = np.append(self.camera_coordinate_z, pose_z, axis=0)
				# self.camera_z = np.append(self.camera_z, z_coordiante)
			self.x_r_arr.append(self.x_r)
			self.y_r_arr.append(self.y_r)
			self.z_r_arr.append(self.z_r)
		else:
			pass
		
	def calibration(self):
		self.camera_z = self.camera_z.reshape(-1,1)
		print(self.camera_z)
		# self.x_coefficient = LinearRegression().fit(self.camera_coordinate, self.x_r_arr)
		# self.y_coefficient = LinearRegression().fit(self.camera_coordinate, self.y_r_arr)
		# self.z_coefficient = LinearRegression().fit(self.camera_z, self.z_r_arr)
		self.x_coefficient = LinearRegression().fit(self.camera_coordinate_x, self.x_r_arr)
		self.y_coefficient = LinearRegression().fit(self.camera_coordinate_y, self.y_r_arr)
		self.z_coefficient = LinearRegression().fit(self.camera_coordinate_z, self.z_r_arr)
		# self.z_coefficient = LinearRegression().fit(self.camera_z, self.z_r_arr)
		print(self.x_coefficient.coef_, self.x_coefficient.intercept_ , self.y_coefficient.coef_, self.y_coefficient.intercept_, self.z_coefficient.coef_, self.z_coefficient.intercept_)
		if self.xy_calib:
			with open(self.xy_filepath,"w+") as file:
				for i in range(len(self.x_coefficient.coef_)):
					file.write(str(self.x_coefficient.coef_[i])+',')
				file.write(str(self.x_coefficient.intercept_)+'\n')
				for i in range(len(self.y_coefficient.coef_)):
					file.write(str(self.y_coefficient.coef_[i])+',')
				file.write(str(self.y_coefficient.intercept_)+'\n')
			with open(self.z_filepath,"w+") as file:
				file.write(str(self.z_coefficient.coef_[0])+',')
				file.write(str(self.z_coefficient.intercept_))
			self.xy_calib_stop_service(Empty)
		elif self.z_calib:
			with open(self.z_filepath,"w+") as file:
				file.write(str(self.z_coefficient.coef_[0])+',')
				file.write(str(self.z_coefficient.intercept_))	
			self.z_calib_stop_service(Empty)	
		self.readCalibFile()

		self.camera_coordinate_x =np.array([[]])
		self.camera_coordinate_y =np.array([[]])
		self.camera_coordinate_z =np.array([[]])
		self.camera_z = np.array([[]])
		self.x_r_arr =[]
		self.y_r_arr =[]
		self.z_r_arr =[]

		




	def work_start_service(self,req):
		print("start movement ")
		self.RUN = 1
                self.arm_enable()
		self.pos_x =300
		first_move = self.arm_move(self.pos_x, self.r_coordinate)
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
		self.camera_coordinate_x =np.array([[]])
		self.camera_coordinate_y =np.array([[]])
		self.camera_coordinate_z =np.array([[]])
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
		self.camera_coordinate_x =np.array([[]])
		self.camera_coordinate_y =np.array([[]])
		self.camera_coordinate_z =np.array([[]])
		self.z_r_arr =[]
		return EmptyResponse()


if __name__ == "__main__":
	print("MG400_work start")
	rospy.init_node('MG400_work')
	mv = MOVE()
	rospy.spin()
# 	mv.work_stop_service(Empty)
