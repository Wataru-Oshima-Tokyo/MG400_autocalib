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
        first_move = self.arm_move(200, 0, 0, 0, 0, 0)
        self.sub = rospy.Subscriber("/autocalib/coordinate", Coordinate, self.move_callback)
        self.pub_move = rospy.Publisher("/autocalib/move", Int16, queue_size=10)

    def move_callback(self, msg):
        print(msg.x, msg.y)

if __name__ == "__main__":
	print("MG400_work start")
	rospy.init_node('MG400_work')
	mv = MOVE()
	rospy.spin()
