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
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
		self.hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)  #色空間の変換(BGR→HSV)
		cv.namedWindow('hsv')
		cv.setMouseCallback('hsv', self.mouseEvent)
		# now click into the hsv img , and look at values:
		gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
		circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, dp=1, minDist=20, param1=100, param2=60, minRadius=0, maxRadius=0)
		circles = np.uint16(np.around(circles))
		for circle in circles[0, :]:
		    # 円周を描画する
			cv.circle(image, (circle[0], circle[1]), circle[2], (0, 165, 255), 5)
			# 中心点を描画する
			cv.circle(image, (circle[0], circle[1]), 2, (0, 0, 255), 3)    
		cv.imshow("hsv",self.hsv)
		cv.waitKey(3)

		
if __name__=="__main__":
	print("Start")
	rospy.init_node('follower')
	autocalib = AUTOCALIB()
#	follower.setOpenCVParams()
	rospy.spin()