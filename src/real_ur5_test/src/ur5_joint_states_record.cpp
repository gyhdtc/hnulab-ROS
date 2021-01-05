#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <fstream>

std::ofstream joint_states_position;

/**
 * This tutorial demonstrates simple receipt and record of messages over the ROS
 * system.
 */

void jointstatesCallback(const sensor_msgs::JointStateConstPtr &msg) {
  // joint_states_position.open( "/home/ipc/diansai_ws/src/real_ur5_test/joint_states_position.txt", std::ios::app | std::ios::out);
  // joint_states_position << msg->position[0] << "\t";
  // joint_states_position << msg->position[1] << "\t";
  // joint_states_position << msg->position[2] << "\t";
  // joint_states_position << msg->position[3] << "\t";
  // joint_states_position << msg->position[4] << "\t";
  // joint_states_position << msg->position[5] << "\n";
  // joint_states_position.close();

  //启动实体机械臂才有velocity,如果没有velocity,会报错Segmentation fault (core dumped)
    float pos[3],vel[3];
    pos[0]=msg->position[0];
    pos[1]=msg->position[1];
    pos[2]=msg->position[2];
    vel[0]=msg->velocity[0];
    vel[1]=msg->velocity[1];
    vel[2]=msg->velocity[2];
    ROS_INFO("I heard: [%f] [%f] [%f] [%f] [%f] [%f]",pos[0],pos[1],pos[2],vel[0],vel[1],vel[2]);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "ur5_joint_states_record");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/joint_states", 100, jointstatesCallback);

  ros::spin();

  return 0;
}
