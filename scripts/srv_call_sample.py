#!/usr/bin/env python
# --*-- coding: utf-8 -*-
import rospy
from mg400_bringup.srv import InsertStatus
import actionlib
from ros_central_server_action.msg import Send_commandAction, Send_commandGoal

action_server_status = ["PENDING", "ACTIVE", "PREEMPTED", "SUCCEEDED", "ABORTED", "REJECTED", "PREEMPTING","RECALLING", "RECALLED", "LOST"]
if __name__ == "__main__":
    rospy.init_node('MG400_srv_sample')
    msg_command_client= actionlib.SimpleActionClient("msg_command", Send_commandAction)
    rate = rospy.Rate(10)
    msg_cmd_goal = Send_commandGoal()
    msg_cmd_goal.command = "CHARGING"
    msg_cmd_goal.duration = 20
    msg_cmd_goal.to = "TK-MG400-001"
    msg_command_client.send_goal(msg_cmd_goal)
    msg_cmd_client_state = action_server_status[msg_command_client.get_state()]
    now = rospy.Time.now()
    while msg_cmd_client_state != action_server_status[3]:
      rospy.loginfo("waiting for the result")
      msg_cmd_client_state = action_server_status[msg_command_client.get_state()]
      if msg_cmd_client_state == action_server_status[2] or  msg_cmd_client_state == action_server_status[4] or (now+rospy.Duration(msg_cmd_goal.duration + 5) <rospy.Time.now()):
        rospy.loginfo("CHARGING Failed")
        break
      rate.sleep()
