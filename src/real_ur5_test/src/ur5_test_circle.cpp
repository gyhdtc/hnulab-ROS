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
#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "ur5_test_circle_demo");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface arm("realarm");

  //获取终端link的名称
  std::string end_effector_link = arm.getEndEffectorLink();
  ROS_INFO("end_effector_link: %s", end_effector_link.c_str());

  //设置目标位置所使用的参考坐标系
  std::string reference_frame = "base_link";
  arm.setPoseReferenceFrame(reference_frame);

  //当运动规划失败后，允许重新规划
  arm.allowReplanning(true);

  //设置位置(单位：米)和姿态（单位：弧度）的允许误差
  arm.setGoalPositionTolerance(0.001);
  arm.setGoalOrientationTolerance(0.01);

  //设置允许的最大速度和加速度
  arm.setMaxAccelerationScalingFactor(0.02);
  arm.setMaxVelocityScalingFactor(0.02);

  //控制机械臂先回到原点
  double targetPoseA[6] = {-0.3152, -2.6356, -0.4054, -1.6554, 1.6013, 0};
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
  sleep(5);

  // 设置机器人终端的目标位置
  geometry_msgs::Pose target_pose;
  target_pose.orientation.x = 0.689;
  target_pose.orientation.y = 0.723;
  target_pose.orientation.z = -0.047;
  target_pose.orientation.w = -0.015;

  target_pose.position.x = -0.635;
  target_pose.position.y = 0.077;
  target_pose.position.z = 0.483;

  arm.setPoseTarget(target_pose);
  arm.move();
  ROS_INFO("arrive to center B");
  sleep(5);

  std::vector<geometry_msgs::Pose> waypoints;

  //将初始位姿加入路点列表
  waypoints.push_back(target_pose);

  double centerA = target_pose.position.x;
  double centerB = target_pose.position.y;
  double radius = 0.1;

  for (double th = 0.0; th < 12.56; th = th + 0.01) {
    target_pose.position.x = centerA + radius * cos(th);
    target_pose.position.y = centerB + radius * sin(th);
    waypoints.push_back(target_pose);
  }

  // 笛卡尔空间下的路径规划
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = 0.0;
  int maxtries = 100; //最大尝试规划次数
  int attempts = 0;   //已经尝试规划次数

  while (fraction < 1.0 && attempts < maxtries) {
    fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold,
                                        trajectory);
    attempts++;

    if (attempts % 10 == 0)
      ROS_INFO("Still trying after %d attempts...", attempts);
  }

  if (fraction == 1) {
    ROS_INFO("Path computed successfully. Moving the arm.");
    sleep(5);

    // 生成机械臂的运动规划数据
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    // // 执行运动
    // arm.execute(plan);
    // sleep(1);

    arm.asyncExecute(plan);
    // 获取机器人的起始位置
    ros::Duration(2).sleep();
    moveit::core::RobotStatePtr start_state(arm.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group =
        start_state->getJointModelGroup(arm.getName());

    std::vector<double> joint_positions(6),joint_velocities(6),joint_accelerations(6);
    start_state->copyJointGroupPositions(joint_model_group,
                                         joint_positions);
    start_state->copyJointGroupVelocities(joint_model_group, joint_velocities);
    start_state->copyJointGroupAccelerations(joint_model_group,
                                             joint_accelerations);
    ROS_INFO("joint_positions: %2.5f, %2.5f, %2.5f, %2.5f, %2.5f, %2.5f",
             joint_positions[0], joint_positions[1],
             joint_positions[2], joint_positions[3],
             joint_positions[4], joint_positions[5]);
    ROS_INFO("joint_velocities: %2.5f, %2.5f, %2.5f, %2.5f, %2.5f, %2.5f",
             joint_velocities[0], joint_velocities[1],
             joint_velocities[2], joint_velocities[3],
             joint_velocities[4], joint_velocities[5]);
    ROS_INFO("joint_accelerations: %2.5f, %2.5f, %2.5f, %2.5f, %2.5f, %2.5f",
             joint_accelerations[0], joint_accelerations[1],
             joint_accelerations[2], joint_accelerations[3],
             joint_accelerations[4], joint_accelerations[5]);
    

    start_state->copyJointGroupPositions(joint_model_group,
                                         joint_positions);
    start_state->copyJointGroupVelocities(joint_model_group, joint_velocities);
    start_state->copyJointGroupAccelerations(joint_model_group,
                                             joint_accelerations);
    ROS_INFO("joint_positions: %2.5f, %2.5f, %2.5f, %2.5f, %2.5f, %2.5f",
             joint_positions[0], joint_positions[1],
             joint_positions[2], joint_positions[3],
             joint_positions[4], joint_positions[5]);
    ROS_INFO("joint_velocities: %2.5f, %2.5f, %2.5f, %2.5f, %2.5f, %2.5f",
             joint_velocities[0], joint_velocities[1],
             joint_velocities[2], joint_velocities[3],
             joint_velocities[4], joint_velocities[5]);
    ROS_INFO("joint_accelerations: %2.5f, %2.5f, %2.5f, %2.5f, %2.5f, %2.5f",
             joint_accelerations[0], joint_accelerations[1],
             joint_accelerations[2], joint_accelerations[3],
             joint_accelerations[4], joint_accelerations[5]);
    ROS_INFO("The robot has completed the desired trajectory");

  } else {
    ROS_INFO("Path planning failed with only %0.6f success after %d attempts.",
             fraction, maxtries);
  }

  // 控制机械臂先回到初始化位置
  arm.setNamedTarget("start");
  arm.move();
  sleep(1);

  ros::shutdown();
  return 0;
}
