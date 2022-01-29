#!/usr/bin/env python
# --*-- coding: utf-8 -*-

import cv2 as cv
from sensor_msgs.msg import Image, String
import rospy
import cv_bridge
import numpy as np
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

class MOVE:
    def __init__(self):
        print("start MG400")
        self.sub = rospy.Subscriber("/autocalib/coordinate", Float64MultiArray, self.move_callback)
        self.pub_move = rospy.Publisher("/autocalib/move", int, queue_size=10)

    def move_callback(self, msg):
        print(msg)
