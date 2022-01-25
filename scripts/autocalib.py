#!/usr/bin/env python
# --*-- coding: utf-8 -*-

import cv2 as cv
from sensor_msgs.msg import Image
import rospy
import cv_bridge
import numpy as np


class AUTOCALIB:
	def __init__(self):
		print("__init__")
		self.bridge = cv_bridge.CvBridge()
# 		cv.namedWindow('BGR Image', 1)  #'BGR Image'という名前の画像表示のウィンドウを作成a
# 		cv.namedWindow('MASK', 1)   #'MASK'という名前の画像表示のウィンドウを作成
# 		cv.namedWindow('MASKED', 1) #'MASK'という名前の画像表示のウィンドウを作成
		self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)   #Image型で画像トピックを購読し，コールバック関数を呼ぶ
		
	def image_callback(self, msg):
		#print("I will write down codes below")
		self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
		# now click into the hsv img , and look at values:
		self.hsv = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)
		self.gray =cv.cvtColor(self.image,cv.COLOR_BGR2GRAY)
		lower_yellow=np.array([20,7,215])
		upper_yellow=np.array([40,27,295])
		self.mask=cv.inRange(self.hsv, lower_yellow, upper_yellow)
		self.masked=cv.bitwise_and(self.image, self.image, mask=self.mask)
		# thresh = 100
		# ret,thresh_img = cv.threshold(self.gray, thresh, 255, cv.THRESH_BINARY)
		#find contours
		contours, hierarchy = cv.findContours(self.mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
		# 小さい輪郭は誤検出として削除する
		contours = list(filter(lambda x: cv.contourArea(x) > 100, contours))
		cv.drawContours(self.image, contours,-1, (0,255,0), 3)
		# circles = cv.HoughCircles(self.masked, cv.HOUGH_GRADIENT, dp=1, minDist=20, param1=100, param2=60, minRadius=0, maxRadius=0)
		# circles = np.uint16(np.around(circles))
		# for circle in circles[0, :]:
		#     # 円周を描画する
		# 	cv.circle(self.image, (circle[0], circle[1]), circle[2], (0, 165, 255), 5)
		# 	# 中心点を描画する
		# 	cv.circle(self.image, (circle[0], circle[1]), 2, (0, 0, 255), 3)    
		cv.imshow("hsv",self.image)
		cv.waitKey(3)


if __name__=="__main__":
	print("Start")
	rospy.init_node('follower')
	autocalib = AUTOCALIB()
#	follower.setOpenCVParams()
	rospy.spin()
