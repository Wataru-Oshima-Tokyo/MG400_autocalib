#!/usr/bin/env python
# --*-- coding: utf-8 -*-
import rospy
from mg400_bringup.srv import InsertStatus

if __name__ == "__main__":
    rospy.init_node('MG400_srv_sample')
    insert_result_srvp_ =rospy.ServiceProxy('/insert_result',InsertStatus)

    rospy.sleep(2)

    insert_result_srvp_(1) 