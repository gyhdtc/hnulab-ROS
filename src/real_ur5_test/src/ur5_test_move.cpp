/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <dh_hand_driver/start_ag95.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur5_test_move");
    ros::NodeHandle n;
    ros::ServiceClient ag95client =
      n.serviceClient<dh_hand_driver::start_ag95>("start_ag95");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("realarm");
     //获取终端link的名称
    std::string end_effector_link = arm.getEndEffectorLink();
    ROS_INFO("end_effector_link: %s",end_effector_link.c_str());
    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base";
    arm.setPoseReferenceFrame(reference_frame);

    //当运动规划失败后，允许重新规划
    arm.allowReplanning(true);
   
    arm.setGoalJointTolerance(0.001);

    arm.setMaxAccelerationScalingFactor(0.02);
    arm.setMaxVelocityScalingFactor(0.02);

    //控制机械臂先回到初始化位置A
    double targetPoseA[6] = {-0.11459428468813115, -2.0483997503863733, -1.090170685444967, -1.459487263356344, 1.514670491218567, -0.07102996507753545};
    std::vector<double> joint_group_positions(6);
    joint_group_positions[0] = targetPoseA[0];
    joint_group_positions[1] = targetPoseA[1];
    joint_group_positions[2] = targetPoseA[2];
    joint_group_positions[3] = targetPoseA[3];
    joint_group_positions[4] = targetPoseA[4];
    joint_group_positions[5] = targetPoseA[5];
    
    arm.setJointValueTarget(joint_group_positions);
    arm.move();
    ROS_INFO("arrive to A");
    sleep(1);

    
  dh_hand_driver::start_ag95 srv;
  srv.request.motorID = 1;
  srv.request.setpos = 10;
  srv.request.setforce = 20;
  if (ag95client.call(srv)) 
  {
    ROS_INFO("AG95_state: %s", srv.response.start_state.c_str());
  } 
  else 
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

     //控制机械臂先回到位置B
    double targetPoseB[6] = {-0.11451083818544561, -1.8543975988971155, -0.6480115095721644, -1.4594996611224573, 1.5146944522857666, -0.07109004655946904};
    joint_group_positions[0] = targetPoseB[0];
    joint_group_positions[1] = targetPoseB[1];
    joint_group_positions[2] = targetPoseB[2];
    joint_group_positions[3] = targetPoseB[3];
    joint_group_positions[4] = targetPoseB[4];
    joint_group_positions[5] = targetPoseB[5];

    arm.setJointValueTarget(joint_group_positions);
    arm.move();
    ROS_INFO("arrive to B");
    sleep(1);
    
     // 设置机器人终端的目标位置
    geometry_msgs::Pose target_pose;
    ////base_link
    // target_pose.orientation.x = 0.689;
    // target_pose.orientation.y = 0.723;
    // target_pose.orientation.z = -0.047;
    // target_pose.orientation.w = -0.015;

    // target_pose.position.x = -0.635;
    // target_pose.position.y = 0.077;
    // target_pose.position.z = 0.483;

    ////base
    target_pose.orientation.x = 0.723;
    target_pose.orientation.y = -0.689;
    target_pose.orientation.z = 0.015;
    target_pose.orientation.w = -0.046;

    target_pose.position.x = 0.635;
    target_pose.position.y = -0.076;
    target_pose.position.z = 0.482;


     // 设置机器臂当前的状态作为运动初始状态
    arm.setStartStateToCurrentState();

    arm.setPoseTarget(target_pose);

    // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);

    ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");   

    //让机械臂按照规划的轨迹开始运动。
    if(success)
    {
        arm.execute(plan); 
        sleep(1);
        ROS_INFO("arrive to C");
    }     
    else
      ROS_INFO("can't arrive to C");

  srv.request.motorID = 1;
  srv.request.setpos = 50;
  srv.request.setforce = 20;
  if (ag95client.call(srv)) 
  {
    ROS_INFO("AG95_state: %s", srv.response.start_state.c_str());
  } 
  else 
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("lift");
    arm.move();
    sleep(1);

    ros::shutdown(); 

    return 0;
}
