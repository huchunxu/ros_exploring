#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# 初始化ROS节点，声明一个发布速度控制的Publisher
rospy.init_node('voice_teleop')
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
r = rospy.Rate(10)

# 接收到语音命令后发布速度指令
def get_voice(data):
    voice_text=data.data
    rospy.loginfo("I said:: %s",voice_text)
    twist = Twist()
    
    if voice_text == "go":
        twist.linear.x = 2 
    elif voice_text == "back":
        twist.linear.x = -2 
    elif voice_text == "left":
        twist.angular.z = 2 
    elif voice_text == "right":
        twist.angular.z = -2 
        
    pub.publish(twist)

# 订阅pocketsphinx语音识别的输出字符
def teleop():
    rospy.loginfo("Starting voice Teleop")
    rospy.Subscriber("/recognizer/output", String, get_voice)
    rospy.spin()

while not rospy.is_shutdown():
    teleop()

