#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros

# 定义状态Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'

# 定义状态Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome2'

# 主函数
def main():
    rospy.init_node('smach_example_state_machine')

    # 创建一个状态机
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # 打开状态机容器
    with sm:
        # 使用add方法添加状态到状态机容器当中
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 
                                            'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome2':'FOO'})
    
    # 创建并启动内部监测服务器
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()
    
    # 开始执行状态机
    outcome = sm.execute()
    
    # 等待退出
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
