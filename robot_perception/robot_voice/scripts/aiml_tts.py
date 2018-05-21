#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, os, sys
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String

# 初始化ROS节点以及sound工具
rospy.init_node('aiml_tts', anonymous = True)

soundhandle = SoundClient()
rospy.sleep(1)
soundhandle.stopAll()
print 'Starting TTS'

# 获取应答字符，并且通过语音输出
def get_response(data):
    response = data.data
    rospy.loginfo("Response ::%s",response)
    soundhandle.say(response)

# 订阅语音识别后的应答字符
def listener():
    rospy.loginfo("Starting listening to response")
    rospy.Subscriber("response",String, get_response,queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    listener()
