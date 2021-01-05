#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose


class MoveItIkDemo:
    # 初始化move_group的API
    moveit_commander.roscpp_initialize(sys.argv)
    # 初始化ROS节点
    rospy.init_node('moveit_ik_demo') 
    # 初始化需要使用move group控制的机械臂中的arm group
    arm = moveit_commander.MoveGroupCommander('manipulator')
    
    # 获取终端link的名称
    # end_effector_link = arm.get_end_effector_link()
    
    # 设置终端名称
    end_effector_link = 'wrist_3_link'
    # 设置目标位置所使用的参考坐标系
    # reference_frame = 'base_link'
    reference_frame = 'base'

    def __init__(self):
        print "set arm ..."
        # 设置参考坐标系
        self.arm.set_pose_reference_frame(self.reference_frame)
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.01)
        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)

    def gobackup(self):
        print "control arm to UP ..."
        # 控制机械臂先回到初始化位置
        self.arm.set_named_target('up')
        self.arm.go()

    def move(self, zuobiao, siyuanshu):
        print "go to user [ XYZ ] ..."
        print zuobiao
        print siyuanshu
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        # 设置坐标
        target_pose.pose.position.x = zuobiao[0]
        target_pose.pose.position.y = zuobiao[1]
        target_pose.pose.position.z = zuobiao[2]
        target_pose.pose.orientation.x = siyuanshu[0]
        target_pose.pose.orientation.y = siyuanshu[1]
        target_pose.pose.orientation.z = siyuanshu[2]
        target_pose.pose.orientation.w = siyuanshu[3]
        # target_pose.pose.orientation.x = -0.601
        # target_pose.pose.orientation.y = -0.366
        # target_pose.pose.orientation.z = 0.359
        # target_pose.pose.orientation.w = 0.613

        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()
        # 设置机械臂终端运动的目标位姿
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        # 规划运动路径
        traj = self.arm.plan()
        # 按照规划的运动路径控制机械臂运动
        self.arm.execute(traj)

    def closemove(self):
        print "close and exit MOVEIT and ROS ..."
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    zuobiao =  [-0.000, -0.109, 1.001]
    # siyuanshu = [0.760, 0.417, -0.236, -0.438] [-0.611, 0.313, -0.338, 0.644]
    siyuanshu = [-0.000, 0.000, -1.000, 0.000]
    gyh = MoveItIkDemo()
    # gyh.gobackup()
    gyh.move(zuobiao, siyuanshu)
    gyh.closemove()


    
    
