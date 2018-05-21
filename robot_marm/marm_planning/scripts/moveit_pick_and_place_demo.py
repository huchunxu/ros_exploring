#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'

GRIPPER_FRAME = 'grasping_frame'

GRIPPER_OPEN = [0.004]
GRIPPER_CLOSED = [0.01]

REFERENCE_FRAME = 'base_link'

class MoveItPickAndPlaceDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_pick_and_place_demo')
        
        # 初始化场景对象
        scene = PlanningSceneInterface()
        
        # 创建一个发布场景变化信息的发布者
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)
        
        # 创建一个发布抓取姿态的发布者
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size=10)
        
        # 创建一个存储物体颜色的字典对象
        self.colors = dict()
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander(GROUP_NAME_ARM)
        
        # 初始化需要使用move group控制的机械臂中的gripper group
        gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
 
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.05)
        arm.set_goal_orientation_tolerance(0.1)

        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        arm.set_pose_reference_frame(REFERENCE_FRAME)
        
        # 设置每次运动规划的时间限制：5s
        arm.set_planning_time(5)
        
        # 设置pick和place阶段的最大尝试次数
        max_pick_attempts = 5
        max_place_attempts = 5
        rospy.sleep(2)

        # 设置场景物体的名称 
        table_id = 'table'
        box1_id = 'box1'
        box2_id = 'box2'
        target_id = 'target'
                
        # 移除场景中之前运行残留的物体
        scene.remove_world_object(table_id)
        scene.remove_world_object(box1_id)
        scene.remove_world_object(box2_id)
        scene.remove_world_object(target_id)
        
        # 移除场景中之前与机器臂绑定的物体
        scene.remove_attached_object(GRIPPER_FRAME, target_id)  
        rospy.sleep(1)
        
        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        
        # 控制夹爪张开
        gripper.set_joint_value_target(GRIPPER_OPEN)
        gripper.go()
        rospy.sleep(1)

        # 设置桌面的高度
        table_ground = 0.2
        
        # 设置table、box1和box2的三维尺寸[长, 宽, 高]
        table_size = [0.2, 0.7, 0.01]
        box1_size = [0.1, 0.05, 0.05]
        box2_size = [0.05, 0.05, 0.15]
        
        # 将三个物体加入场景当中
        table_pose = PoseStamped()
        table_pose.header.frame_id = REFERENCE_FRAME
        table_pose.pose.position.x = 0.35
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)
        
        box1_pose = PoseStamped()
        box1_pose.header.frame_id = REFERENCE_FRAME
        box1_pose.pose.position.x = 0.31
        box1_pose.pose.position.y = -0.1
        box1_pose.pose.position.z = table_ground + table_size[2] + box1_size[2] / 2.0
        box1_pose.pose.orientation.w = 1.0   
        scene.add_box(box1_id, box1_pose, box1_size)
        
        box2_pose = PoseStamped()
        box2_pose.header.frame_id = REFERENCE_FRAME
        box2_pose.pose.position.x = 0.29
        box2_pose.pose.position.y = 0.13
        box2_pose.pose.position.z = table_ground + table_size[2] + box2_size[2] / 2.0
        box2_pose.pose.orientation.w = 1.0   
        scene.add_box(box2_id, box2_pose, box2_size)       
                
        # 将桌子设置成红色，两个box设置成橙色
        self.setColor(table_id, 0.8, 0, 0, 1.0)
        self.setColor(box1_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box2_id, 0.8, 0.4, 0, 1.0)
        
        # 设置目标物体的尺寸
        target_size = [0.02, 0.01, 0.12]
        
        # 设置目标物体的位置，位于桌面之上两个盒子之间
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = 0.32
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
        target_pose.pose.orientation.w = 1.0
        
        # 将抓取的目标物体加入场景中
        scene.add_box(target_id, target_pose, target_size)
        
        # 将目标物体设置为黄色
        self.setColor(target_id, 0.9, 0.9, 0, 1.0)
        
        # 将场景中的颜色设置发布
        self.sendColors()

        # 设置支持的外观
        arm.set_support_surface_name(table_id)
        
        # 设置一个place阶段需要放置物体的目标位置
        place_pose = PoseStamped()
        place_pose.header.frame_id = REFERENCE_FRAME
        place_pose.pose.position.x = 0.32
        place_pose.pose.position.y = -0.2
        place_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
        place_pose.pose.orientation.w = 1.0

        # 将目标位置设置为机器人的抓取目标位置
        grasp_pose = target_pose
                
        # 生成抓取姿态
        grasps = self.make_grasps(grasp_pose, [target_id])

        # 将抓取姿态发布，可以在rviz中显示
        for grasp in grasps:
            self.gripper_pose_pub.publish(grasp.grasp_pose)
            rospy.sleep(0.2)
    
        # 追踪抓取成功与否，以及抓取的尝试次数
        result = None
        n_attempts = 0
        
        # 重复尝试抓取，直道成功或者超多最大尝试次数
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
            n_attempts += 1
            rospy.loginfo("Pick attempt: " +  str(n_attempts))
            result = arm.pick(target_id, grasps)
            rospy.sleep(0.2)
        
        # 如果pick成功，则进入place阶段 
        if result == MoveItErrorCodes.SUCCESS:
            result = None
            n_attempts = 0
            
            # 生成放置姿态
            places = self.make_places(place_pose)
            
            # 重复尝试放置，直道成功或者超多最大尝试次数
            while result != MoveItErrorCodes.SUCCESS and n_attempts < max_place_attempts:
                n_attempts += 1
                rospy.loginfo("Place attempt: " +  str(n_attempts))
                for place in places:
                    result = arm.place(target_id, place)
                    if result == MoveItErrorCodes.SUCCESS:
                        break
                rospy.sleep(0.2)
                
            if result != MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Place operation failed after " + str(n_attempts) + " attempts.")
        else:
            rospy.loginfo("Pick operation failed after " + str(n_attempts) + " attempts.")
                
        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()
        
        # 控制夹爪回到张开的状态
        gripper.set_joint_value_target(GRIPPER_OPEN)
        gripper.go()
        rospy.sleep(1)

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        
    # 创建夹爪的姿态数据JointTrajectory
    def make_gripper_posture(self, joint_positions):
        # 初始化夹爪的关节运动轨迹
        t = JointTrajectory()
        
        # 设置夹爪的关节名称
        t.joint_names = ['finger_joint1']
        
        # 初始化关节轨迹点
        tp = JointTrajectoryPoint()
        
        # 将输入的关节位置作为一个目标轨迹点
        tp.positions = joint_positions
        
        # 设置夹爪的力度
        tp.effort = [1.0]
        
        # 设置运动时间
        tp.time_from_start = rospy.Duration(1.0)
        
        # 将目标轨迹点加入到运动轨迹中
        t.points.append(tp)
        
        # 返回夹爪的关节运动轨迹
        return t
    
    # 使用给定向量创建夹爪的translation结构
    def make_gripper_translation(self, min_dist, desired, vector):
        # 初始化translation对象
        g = GripperTranslation()
        
        # 设置方向向量
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]
        
        # 设置参考坐标系
        g.direction.header.frame_id = GRIPPER_FRAME
        
        # 设置最小和期望的距离
        g.min_distance = min_dist
        g.desired_distance = desired
        
        return g

    # 创建一个允许的的抓取姿态列表
    def make_grasps(self, initial_pose_stamped, allowed_touch_objects):
        # 初始化抓取姿态对象
        g = Grasp()
        
        # 创建夹爪张开、闭合的姿态
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)
                
        # 设置期望的夹爪靠近、撤离目标的参数
        g.pre_grasp_approach = self.make_gripper_translation(0.01, 0.1, [1.0, 0.0, 0.0])
        g.post_grasp_retreat = self.make_gripper_translation(0.1, 0.15, [0.0, -1.0, 1.0])
        
        # 设置抓取姿态
        g.grasp_pose = initial_pose_stamped
    
        # 需要尝试改变姿态的数据列表
        pitch_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3]
        yaw_vals = [0]

        # 抓取姿态的列表
        grasps = []

        # 改变姿态，生成抓取动作
        for y in yaw_vals:
            for p in pitch_vals:
                # 欧拉角到四元数的转换
                q = quaternion_from_euler(0, p, y)
                
                # 设置抓取的姿态
                g.grasp_pose.pose.orientation.x = q[0]
                g.grasp_pose.pose.orientation.y = q[1]
                g.grasp_pose.pose.orientation.z = q[2]
                g.grasp_pose.pose.orientation.w = q[3]
                
                # 设置抓取的唯一id号
                g.id = str(len(grasps))
                
                # 设置允许接触的物体
                g.allowed_touch_objects = allowed_touch_objects
                
                # 将本次规划的抓取放入抓取列表中
                grasps.append(deepcopy(g))
                
        # 返回抓取列表
        return grasps
    
    # 创建一个允许的放置姿态列表
    def make_places(self, init_pose):
        # 初始化放置抓取物体的位置
        place = PoseStamped()
        
        # 设置放置抓取物体的位置
        place = init_pose
        
        # 定义x方向上用于尝试放置物体的偏移参数
        x_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]
        
        # 定义y方向上用于尝试放置物体的偏移参数
        y_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]
        
        pitch_vals = [0]
        
        # 定义用于尝试放置物体的偏航角参数
        yaw_vals = [0]

        # 定义放置物体的姿态列表
        places = []
        
        # 生成每一个角度和偏移方向上的抓取姿态
        for y in yaw_vals:
            for p in pitch_vals:
                for y in y_vals:
                    for x in x_vals:
                        place.pose.position.x = init_pose.pose.position.x + x
                        place.pose.position.y = init_pose.pose.position.y + y
                        
                        # 欧拉角到四元数的转换
                        q = quaternion_from_euler(0, p, y)
                        
                        # 欧拉角到四元数的转换
                        place.pose.orientation.x = q[0]
                        place.pose.orientation.y = q[1]
                        place.pose.orientation.z = q[2]
                        place.pose.orientation.w = q[3]
                        
                        # 将该放置姿态加入列表
                        places.append(deepcopy(place))
        
        # 返回放置物体的姿态列表
        return places
    
    # 设置场景物体的颜色
    def setColor(self, name, r, g, b, a = 0.9):
        # 初始化moveit颜色对象
        color = ObjectColor()
        
        # 设置颜色值
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # 更新颜色字典
        self.colors[name] = color

    # 将颜色设置发送并应用到moveit场景当中
    def sendColors(self):
        # 初始化规划场景对象
        p = PlanningScene()

        # 需要设置规划场景是否有差异     
        p.is_diff = True
        
        # 从颜色字典中取出颜色设置
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # 发布场景物体颜色设置
        self.scene_pub.publish(p)

if __name__ == "__main__":
    MoveItPickAndPlaceDemo()

    
