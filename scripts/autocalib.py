#!/usr/bin/env python
# --*-- coding: utf-8 -*-

import cv2 as cv
from sensor_msgs.msg import Image
import rospy
import cv_bridge
import numpy as np
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse


class AUTOCALIB:
	def __init__(self):
		print("__init__")
		self.bridge = cv_bridge.CvBridge()

		self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)   #Image型で画像トピックを購読し，コールバック関数を呼ぶ				
		self.start_srv_ = rospy.Service('/autocalib/start', Empty, self.clbk_start_service)
		self.stop_srv_ = rospy.Service('/autocalib/stop', Empty, self.clbk_stop_service)
	        self.hz = 20
                self.RUN = 0
                self.TIMEOUT = 0.5 
		rate = rospy.Rate(self.hz)
		self.last_clb_time_ = rospy.get_time()

		while not rospy.is_shutdown():
			if self.RUN == 1:
				
				time_duration = rospy.get_time() - self.last_clb_time_
				print(time_duration)
				if time_duration < self.TIMEOUT:
					self.start_autocalib()

				#self.pub_.publish(self.msg)
				rate.sleep()
	

	def start_autocalib(self):
    		# マニピュレータのアームに貼り付けた円形マークの座標を検出
		print("start autocalib")
 		cv.setMouseCallback("Dobot", self.mouseEvent)
		result_arm = self.getCircle(0)
		if isinstance(result_arm, np.ndarray):
			arm_x, arm_y, arm_r = result_arm
		
		# 円形の形状をしたピッキング・オブジェクトの座標を検出
		result_obj = self.getCircle(1)
		if isinstance(result_obj, np.ndarray):
			obj_x, obj_y, obj_r = result_obj
		
		if self.isSetArm:
			x, y, r = np.int16(np.around([arm_x, arm_y, arm_r]))
			self.drawMark(x, y, r)
		else:
			cv.putText(self.image, "L-Click : Mark of Arm",
						(20, 450), cv.FONT_HERSHEY_PLAIN, 1.5, (0,255,255), 2)
		
		if self.isSetObj:
			x, y, r = np.int16(np.around([obj_x, obj_y, obj_r]))
			self.drawMark(x, y, r)
		else:
			cv.putText(self.image, "R-Click : Mark of Target",
						(20, 470), cv.FONT_HERSHEY_PLAIN, 1.5, (0,255,255), 2)
		
		if self.isSetArm and self.isSetObj:
			cv.putText(self.image, "M-Click : Reset All Marks",
						(20, 470), cv.FONT_HERSHEY_PLAIN, 1.5, (0,255,255), 2)
		else:
			# 中央に点を描画する
			cv.circle(self.image, (self.SCREEN_WIDTH // 2, self.SCREEN_HEIGHT // 2), 3, (255, 255, 0), 2)

		cv.putText(self.image, "Esc Key -> Exit",
					(490, 20), cv.FONT_HERSHEY_PLAIN, 1.0, (255, 0, 255), 1)
		
		# キャプチャした画像にマークや使用法などを記述して描画
		cv.imshow("Dobot", self.image)
		
		# アームとピッキング・オブジェクトの両方が検出された場合
		if self.isSetArm == True and self.isSetObj == True:
			diff_x = self.arm_x - self.obj_x
			diff_y = self.arm_y - self.obj_y
			
			dist2 = diff_x * diff_x + diff_y * diff_y
			
			# アームがピッキング・オブジェクトに十分重なって見えた場合
			if dist2 < self.FIND_THRESHOLD:
				# 視差により位置ずれの修正とピッキング
				self.fineTuning(arm_x, arm_y, self.TUNING_SCALE_X, self.TUNING_SCALE_Y)
				
				# フラグを初期化
				self.isSetArm = False
				self.isSetObj = False
			else:
				# ある程度、近くまで移動できた場合
				if dist2 < self.FIND_THRESHOLD * 5:
					# 視差による位置ずれより、移動量を計算
					mx = np.clip(diff_x * self.MOVING_COEFF_0, -self.MOVING_MAX_0, self.MOVING_MAX_0)
					my = np.clip(diff_y * self.MOVING_COEFF_0, -self.MOVING_MAX_0, self.MOVING_MAX_0)
					
					# MOVL：Linear Movements [isQueued = True, 同期モード]
					# dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVLXYZINCMode, my, mx, 0, 0, True)
				# アームがピッキング・オブジェクトから遠い場合
				else:
					# 視差による位置ずれより、移動量を計算
					mx = np.clip(diff_x * self.MOVING_COEFF_1, - self.MOVING_MAX_1, self.MOVING_MAX_1)
					my = np.clip(diff_y * self.MOVING_COEFF_1, - self.MOVING_MAX_1, self.MOVING_MAX_1)
					
					# MOVJ：Joint Movements  [isQueued = False, 非同期モード]
					# dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVJXYZINCMode, my, mx, 0, 0, False)
		cv.waitKey(3)



	def clbk_start_service(self,req):
		print("start autocalib follow")
		self.RUN = 1
                self.setDefault()
		return EmptyResponse()


	def clbk_stop_service(self,req):
		print("stop autocalib follow")
		self.RUN = 0
		return EmptyResponse()


	def setDefault(self):
		self.SCREEN_HEIGHT, self.SCREEN_WIDTH = self.image.shape[:2]
		self.setOpenCVParams()

		self.AVERAGE_COUNT = 1

		self.arm_x = self.arm_y = self.arm_r = 0.0  # アームに付けた円形のマークの座標
		self.obj_x = self.obj_y = self.obj_r = 0.0  # ピッキングする円形のオブジェクトの座標

		self.isSetArm = False             # アームの色をピッキングしたら True
		self.isSetObj = False             # ピッキング・オブジェクトの色をピッキングしたら True

		# self.setDobotParams()

		#################################################################
		#  カメラの高さ = 530 mm  [台からカメラレンズの下側まで]        #
		#################################################################
		self.FIND_THRESHOLD  = 20.0    # アームとピッキング・オブジェクトの距離から
								# 近さを判定するための閾値  (20.0)

		self.MOVING_MAX_0    = 5.0     # 近いときの、１回での最大の移動量  (5.0)
		self.MOVING_COEFF_0  = 0.02    # 近いときの、カメラ座標でのズレを、
								# アームの移動量に換算するためのスケール  (0.02)

		self.MOVING_MAX_1    = 50.0    # 遠いときの、１回での最大の移動量  (50.0)
		self.MOVING_COEFF_1  = 0.20    # 遠いときの、カメラ座標でのズレを、
								# アームの移動量に換算するためのスケール  (0.20)

		self.TUNING_SCALE_X  = 0.25    # 視差により位置ずれの修正を行うためのＸ軸方向のスケール  (0.21)
		self.TUNING_SCALE_Y  = 0.28    # 視差により位置ずれの修正を行うためのＹ軸方向のスケール  (0.21)

		self.TUNING_DZ       = 16.0     # 十分近いづいた後に、アームを毎回下げる距離        (8.0)
		self.BOTTOM_Z        = -70.0   # アームの台座のＺ座標（ホームポジションが基準）   (-48.0)
		self.OBJ_HEIGHT      = 1.0    # ピッキングするオブジェクトの高さ（全て均一）     (10.0)
		#################################################################


	def setOpenCVParams(self):
        #################################################################
		#  カメラの高さ = 500 mm  [台からカメラレンズの下側まで]        #
		#################################################################
		self.HOUGH_1      = 30  # 手法依存の 1 番目のパラメータ．: CV_HOUGH_GRADIENT の場合は，
						# Canny() エッジ検出器に渡される2つの閾値の内，大きい方の閾値を表す
		self.HOUGH_2      = 10  # 円の中心を検出する際の投票数の閾値を表す。これが小さくなるほど，
						# より多くの誤検出が起こる可能性がある。
						# より多くの投票を獲得した円が，最初に出力される。
		
		self.ARM_SIZE_MIN = 12   # アームの円形のマークの最小半径  (5)
		self.ARM_SIZE_MAX = 25   # アームの円形のマークの最大半径  (20)
		
		self.OBJ_SIZE_MIN = 30  # 円形のピッキング・オブジェクトの最小半径  (20)
		self.OBJ_SIZE_MAX = 42  # 円形のピッキング・オブジェクトの最大半径  (50)
		#################################################################


		self.CIRCLE_PARAMS = []
		
		self.CIRCLE_PARAMS.append({  # Arm
			"HOUGH": [self.HOUGH_1, self.HOUGH_2, self.ARM_SIZE_MIN, self.ARM_SIZE_MAX],
			"MIN": [0, 0, 0],
			"MAX": [0, 0, 0]})
		
		self.CIRCLE_PARAMS.append({  # Obj
			"HOUGH": [self.HOUGH_1, self.HOUGH_2, self.OBJ_SIZE_MIN, self.OBJ_SIZE_MAX],
			"MIN": [0, 0, 0],
			"MAX": [0, 0, 0]})

	def setColorRange(self,index, x, y):
		print("\tPOS: ", x, y)
		bgr = self.image[y:y + 1, x:x + 1]
		bgr00 = bgr[0][0]
		
		print("\tRGB: ", bgr00)
		
		hsv00 = self.hsv[0][0]
		print("\tHSV: ", hsv00)
		
		#################################################################
		#  HSV 空間における、指定色から最小値と最大値                   #
		#################################################################
		MIN_DH, MIN_DS, MIN_DV = [15,  60, 60]
		MAX_DH, MAX_DS, MAX_DV = [15, 150, 60]
		#################################################################

		
		self.CIRCLE_PARAMS[index]["MIN"][0] = max([0, hsv00[0] - MIN_DH])
		self.CIRCLE_PARAMS[index]["MIN"][1] = max([0, hsv00[1] - MIN_DS])
		self.CIRCLE_PARAMS[index]["MIN"][2] = max([0, hsv00[2] - MIN_DV])
		print("\tMIN: ", np.array(self.CIRCLE_PARAMS[index]["MIN"]))
		
		self.CIRCLE_PARAMS[index]["MAX"][0] = min([255, hsv00[0] + MAX_DH])
		self.CIRCLE_PARAMS[index]["MAX"][1] = min([255, hsv00[1] + MAX_DS])
		self.CIRCLE_PARAMS[index]["MAX"][2] = min([255, hsv00[2] + MAX_DV])

		print("\tMAX: ", np.array(self.CIRCLE_PARAMS[index]["MAX"]))


	def getCircle(self,target_num):
		hsv_min = np.array(self.CIRCLE_PARAMS[target_num]["MIN"])
		hsv_max = np.array(self.CIRCLE_PARAMS[target_num]["MAX"])	
		# マスク画像を用いて元画像から指定した色を抽出
		mask = cv.inRange(self.hsv, hsv_min, hsv_max)
		if target_num == 0:
			cv.imshow("Arm", mask)
		else:
			cv.imshow("Target", mask)

		'''
		area = cv2.countNonZero(mask)
		print(label, ": ", area)
		radius = math.sqrt(area / math.pi)
		print("radius: ", radius)
		'''
		
		hough = self.CIRCLE_PARAMS[target_num]["HOUGH"]
		
		# ハフ変換を用いて，グレースケール画像から円を検出する。
		# 検出された円はベクトル形式で出力される。
		# 各ベクトルは，3要素の浮動小数点型ベクトル  [x, y, radius] としてエンコードされる。
		circles = cv.HoughCircles(
			mask,                   # 8ビット，シングルチャンネル，グレースケールの入力画像
			cv.HOUGH_GRADIENT,     # 現在のところ， CV_HOUGH_GRADIENT メソッドのみが実装されている。
			1,                      # dp - 画像分解能に対する投票分解能の比率の逆数
									# 例えば， dp=1 の場合は，投票空間は入力画像と同じ分解能を持つ。．
									# また dp=2 の場合は，投票空間の幅と高さは半分になる。
			150,                    # minDist ? 検出される円の中心同士の最小距離．
									# このパラメータが小さすぎると，正しい円の周辺に別の円が
									# 複数誤って検出されることになる。
									# 逆に大きすぎると，検出できない円がでてくる可能性がある。
			param1=hough[0],        # 手法依存の 1 番目のパラメータ．: CV_HOUGH_GRADIENT の場合は，
									# Canny() エッジ検出器に渡される2つの閾値の内，大きい方の閾値を表す。
									# （小さい閾値は，この値の半分になる。）．
			param2=hough[1],        # 手法依存の 2 番目のパラメータ．: CV_HOUGH_GRADIENT の場合は，
									# 円の中心を検出する際の投票数の閾値を表す。これが小さくなるほど，
									# より多くの誤検出が起こる可能性がある。
									# より多くの投票を獲得した円が，最初に出力される。
			minRadius=hough[2],     # 円の半径の最小値
			maxRadius=hough[3]      # 円の半径の最大値
		)
		
		try:
			cnt = len(circles[0])
		except:
			return None
		
		self.AVERAGE_LIST = []
		for i in range(len(self.CIRCLE_PARAMS)):
			self.AVERAGE_LIST.append(np.empty((0, 3), int))

		circles = np.uint16(np.around(circles))
		for circle in circles[0, :]:
			x, y, r = circle
			
			if len(self.AVERAGE_LIST[target_num]) < self.AVERAGE_COUNT:
				self.AVERAGE_LIST[target_num] = np.append(self.AVERAGE_LIST[target_num], np.array([[x, y, r]]), axis=0)
			else:
				self.AVERAGE_LIST[target_num] = np.append(self.AVERAGE_LIST[target_num], np.array([[x, y, r]]), axis=0)
				self.AVERAGE_LIST[target_num] = np.delete(self.AVERAGE_LIST[target_num], 0, axis=0)
		
		aveArray = np.mean(self.AVERAGE_LIST[target_num], axis=0)		
		return aveArray

	def mouseEvent(self,event, x, y, flags, param):
		if event == cv.EVENT_LBUTTONUP:
			print("\nL-Button: ARM")
			self.setColorRange(0, x, y)
			self.isSetArm = True
		
		if event == cv.EVENT_RBUTTONUP:
			print("\nR-Button: OBJ")
			self.setColorRange(1, x, y)
			self.isSetObj = True
		
		if event == cv.EVENT_MBUTTONUP:
			self.isSetArm = False
			self.isSetObj = False
			# dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVJXYZMode, 200.0, 0.0, -10.0, 0.0, True)


	def drawMark(self, x, y, radius):
        # 囲み線を描く
		cv.circle(self.image, (x, y), radius, (0, 0, 255), 2)
		
		# 中心点を描く
		cv.circle(self.image, (x, y), 3, (255, 0, 0), 3)


	def image_callback(self, msg):
		self.last_clb_time_ = rospy.get_time()
		#print("I will write down codes below")
		self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
		self.blur = cv.GaussianBlur(self.image, (33, 33), 1)
		# now click into the hsv img , and look at values:
		self.hsv = cv.cvtColor(self.blur, cv.COLOR_BGR2HSV)
		# self.gray =cv.cvtColor(self.image,cv.COLOR_BGR2GRAY)
		# self.result = self.getCircle(0, self.hsv)
		# lower_yellow=np.array([20,7,215])
		# upper_yellow=np.array([40,27,295])
		# self.mask=cv.inRange(self.hsv, lower_yellow, upper_yellow)
		# self.masked=cv.bitwise_and(self.image, self.image, mask=self.mask)
		# # thresh = 100
		# # ret,thresh_img = cv.threshold(self.gray, thresh, 255, cv.THRESH_BINARY)
		# #find contours
		# contours, hierarchy = cv.findContours(self.mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
		# # 小さい輪郭は誤検出として削除する
		# contours = list(filter(lambda x: cv.contourArea(x) > 100, contours))
		# cv.drawContours(self.image, contours,-1, (0,255,0), 3)
		# # circles = cv.HoughCircles(self.masked, cv.HOUGH_GRADIENT, dp=1, minDist=20, param1=100, param2=60, minRadius=0, maxRadius=0)
		# # circles = np.uint16(np.around(circles))
		# # for circle in circles[0, :]:
		# #     # 円周を描画する
		# # 	cv.circle(self.image, (circle[0], circle[1]), circle[2], (0, 165, 255), 5)
		# # 	# 中心点を描画する
		# # 	cv.circle(self.image, (circle[0], circle[1]), 2, (0, 0, 255), 3)    
		#cv.imshow("hsv",self.image)
		#cv.waitKey(3)


if __name__=="__main__":
	print("Start")
	rospy.init_node('follower')
	autocalib = AUTOCALIB()
#	follower.setOpenCVParams()
	rospy.spin()
