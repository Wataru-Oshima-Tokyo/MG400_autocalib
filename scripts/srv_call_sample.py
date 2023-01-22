import rospy
from mg400_bringup.srv import InsertStatus


rospy.init_node('MG400_work')
insert_result_srvp_ =rospy.ServiceProxy('/insert_result',InsertStatus)

rospy.sleep(2)

insert_result_srvp_(1) 