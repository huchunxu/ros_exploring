#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import aiml
import os
import sys
from std_msgs.msg import String

# 初始化ROS节点，创建aiml.Kernel()对象
rospy.init_node('aiml_voice_server')
mybot = aiml.Kernel()
response_publisher = rospy.Publisher('response',String,queue_size=10)

# 加载aiml文件数据
def load_aiml(xml_file):
    data_path = rospy.get_param("aiml_path")
    print data_path
    os.chdir(data_path)
    if os.path.isfile("standard.brn"):
        mybot.bootstrap(brainFile = "standard.brn")
    else:
        mybot.bootstrap(learnFiles = xml_file, commands = "load aiml b")
        mybot.saveBrain("standard.brn")

# 解析输入字符串，匹配并发布应答字符串
def callback(data):
    input = data.data
    response = mybot.respond(input)
    
    rospy.loginfo("I heard:: %s",data.data)
    rospy.loginfo("I spoke:: %s",response)
    response_publisher.publish(response)

# 订阅用于语音识别的语音字符
def listener():
    rospy.loginfo("Starting ROS AIML voice Server")
    rospy.Subscriber("voiceWords", String, callback)
    rospy.spin()

if __name__ == '__main__':  
    load_aiml('startup.xml')
    listener()

