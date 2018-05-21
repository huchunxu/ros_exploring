#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

# 初始化ROS节点，声明一个发布语音字符的Publisher
rospy.init_node('aiml_voice_recognizer')
pub = rospy.Publisher('voiceWords', String, queue_size=10)
r = rospy.Rate(1)

# 将pocketsphinx功能包识别输出的字符转换成aiml_voice_server需要的输入字符
def get_voice(data):
    voice_text=data.data
    rospy.loginfo("I said:: %s",voice_text)
    pub.publish(voice_text) 

# 订阅pocketsphinx语音识别的输出字符
def listener():
    rospy.loginfo("Starting voice recognizer")
    rospy.Subscriber("/recognizer/output", String, get_voice)
    rospy.spin()

while not rospy.is_shutdown():
    listener()

   
