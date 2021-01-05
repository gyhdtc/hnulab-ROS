#ifndef UR5_GRASP
#define UR5_GRASP

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <dh_hand_driver/start_ag95.h>
#include <tf/transform_broadcaster.h>
#include <dh_hand_driver/start_ag95.h>

void cal_target_base_pose( tf::Vector3 target_camera_frame,tf::Quaternion taget_camera_orientation,geometry_msgs::Pose &target_base_pose);

#endif