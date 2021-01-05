
#include <real_ur5_test/ur5_grasp.h>

void cal_target_base_pose(tf::Vector3 target_camera_frame,
                          tf::Quaternion target_camera_orientation,
                          geometry_msgs::Pose &target_robot_pose) {
  tf::Quaternion camera_robot_q(-0.012306055482026292, 0.9998272267758495,
                                0.0030165711411203316, 0.013600657878865666);
  tf::Vector3 camera_robot_position(0.5714762683328084, 0.3185058084901818,
                                    0.9645603402525248);
  tf::Transform camera_to_robot(camera_robot_q, camera_robot_position);
  tf::StampedTransform camera_to_robot_(camera_to_robot, ros::Time::now(),
                                        "base", "kinect2_rgb_optical_frame");

  tf::Vector3 target_robot_frame;
  tf::Quaternion target_robot_orientation;

  target_robot_frame = camera_to_robot_ * target_camera_frame;
  target_robot_orientation = camera_to_robot_ * target_camera_orientation;

  target_robot_pose.position.x = target_robot_frame.getX();
  target_robot_pose.position.y = target_robot_frame.getY();
  target_robot_pose.position.z = target_robot_frame.getZ() + 0.3;

  target_robot_pose.orientation.x = target_robot_orientation.getAxis().getX();
  target_robot_pose.orientation.y = target_robot_orientation.getAxis().getY();
  target_robot_pose.orientation.z = target_robot_orientation.getAxis().getZ();
  target_robot_pose.orientation.w = target_robot_orientation.getW();

  ROS_INFO("POSITION:%0.5f %0.5f %0.5f", target_robot_pose.position.x,
           target_robot_pose.position.y, target_robot_pose.position.z);
  ROS_INFO("ORIENTATION:%0.5f %0.5f %0.5f %0.5f", target_robot_pose.orientation.x,
           target_robot_pose.orientation.y, target_robot_pose.orientation.z,
           target_robot_pose.orientation.w);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur5_grasp");
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
    arm.setMaxVelocityScalingFactor(0.01);

    geometry_msgs::Pose obj_robot_pose,above_obj_robot_pose;
     tf::Vector3 obj_camera_frame;
     tf::Quaternion obj_camera_orientation(-0,-0,6.123e-17,-1);
     obj_camera_frame.setX(-0.022756);
     obj_camera_frame.setY(0.0799366);
     obj_camera_frame.setZ( 0.917974);

     //cal_target_base_pose(obj_camera_frame,obj_camera_orientation,obj_robot_pose);
  std::vector<double> joint_group_positions(6);
    //控制机械臂先回到obj上面
    double targetPose2[6] = {-2.6271191279040735, -0.8607552687274378, 0.9596033096313477, -1.6507814566241663, -1.5826810042010706, -0.01015025774110967};
    joint_group_positions[0] = targetPose2[0];
    joint_group_positions[1] = targetPose2[1];
    joint_group_positions[2] = targetPose2[2];
    joint_group_positions[3] = targetPose2[3];
    joint_group_positions[4] = targetPose2[4];
    joint_group_positions[5] = targetPose2[5];
    arm.setJointValueTarget(joint_group_positions);
    arm.move();

       // //控制机械臂先回到obj
    double targetPose1[6] = {-2.6261001268969935, -0.8271778265582483, 1.0787382125854492, -1.8045147101031702, -1.582705322896139, -0.010784927998678029};
  
    joint_group_positions[0] = targetPose1[0];
    joint_group_positions[1] = targetPose1[1];
    joint_group_positions[2] = targetPose1[2];
    joint_group_positions[3] = targetPose1[3];
    joint_group_positions[4] = targetPose1[4];
    joint_group_positions[5] = targetPose1[5];
    arm.setJointValueTarget(joint_group_positions);
    arm.move();


  dh_hand_driver::start_ag95 srv;
  srv.request.motorID = 1;
  srv.request.setpos = 10;
  srv.request.setforce = 2;
  if (ag95client.call(srv)) 
  {
    ROS_INFO("AG95_state: %s", srv.response.start_state.c_str());
  } 
  else 
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

   
//      // //控制机械臂先回到obj上面
//     double targetPose2[6] = {-2.6271191279040735, -0.8607552687274378, 0.9596033096313477, -1.6507814566241663, -1.5826810042010706, -0.01015025774110967};
    joint_group_positions[0] = targetPose2[0];
    joint_group_positions[1] = targetPose2[1];
    joint_group_positions[2] = targetPose2[2];
    joint_group_positions[3] = targetPose2[3];
    joint_group_positions[4] = targetPose2[4];
    joint_group_positions[5] = targetPose2[5];
    arm.setJointValueTarget(joint_group_positions);
    arm.move();


    // //控制机械臂先回到box上面
    double targetPose3[6] = {-3.122756067906515, -1.327233616505758, 1.2717070579528809, -1.5204623381244105, -1.5826690832721155, -0.010605637227193654};
    joint_group_positions[0] = targetPose3[0];
    joint_group_positions[1] = targetPose3[1];
    joint_group_positions[2] = targetPose3[2];
    joint_group_positions[3] = targetPose3[3];
    joint_group_positions[4] = targetPose3[4];
    joint_group_positions[5] = targetPose3[5];
    
    arm.setJointValueTarget(joint_group_positions);
    arm.move();
    ROS_INFO("arrive to box above");

//  // 设置机器臂当前的状态作为运动初始状态
//     arm.setStartStateToCurrentState();
//     arm.setPoseTarget(obj_robot_pose);

//     // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);

//     ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");   

//     //让机械臂按照规划的轨迹开始运动。
//     if(success)
//     {
//         arm.execute(plan); 
//         //sleep(1);
//         ROS_INFO("arrive to obj");
//     }     
//     else
//       ROS_INFO("can't arrive to obj");

//      obj_robot_pose.position.z= obj_robot_pose.position.z-0.2


    // //控制机械臂先回到box
    double targetPoseA[6] = {-3.12276798883547, -1.3224242369281214, 1.4369120597839355, -1.6904991308795374, -1.582944695149557, -0.010138336812154591};
    joint_group_positions[0] = targetPoseA[0];
    joint_group_positions[1] = targetPoseA[1];
    joint_group_positions[2] = targetPoseA[2];
    joint_group_positions[3] = targetPoseA[3];
    joint_group_positions[4] = targetPoseA[4];
    joint_group_positions[5] = targetPoseA[5];
    
    arm.setJointValueTarget(joint_group_positions);
    arm.move();
    ROS_INFO("arrive to box");
    sleep(1);

   
  srv.request.motorID = 1;
  srv.request.setpos = 50;
  srv.request.setforce = 30;
  if (ag95client.call(srv)) 
  {
    ROS_INFO("AG95_state: %s", srv.response.start_state.c_str());
  } 
  else 
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }
    
  // dh_hand_driver::start_ag95 srv;
  // srv.request.motorID = 1;
  // srv.request.setpos = 10;
  // srv.request.setforce = 20;
  // if (ag95client.call(srv)) 
  // {
  //   ROS_INFO("AG95_state: %s", srv.response.start_state.c_str());
  // } 
  // else 
  // {
  //   ROS_ERROR("Failed to call service");
  //   return 1;
  // }

  //    //控制机械臂先回到位置B
  //   double targetPoseB[6] = {-0.11451083818544561, -1.8543975988971155, -0.6480115095721644, -1.4594996611224573, 1.5146944522857666, -0.07109004655946904};
  //   joint_group_positions[0] = targetPoseB[0];
  //   joint_group_positions[1] = targetPoseB[1];
  //   joint_group_positions[2] = targetPoseB[2];
  //   joint_group_positions[3] = targetPoseB[3];
  //   joint_group_positions[4] = targetPoseB[4];
  //   joint_group_positions[5] = targetPoseB[5];

  //   arm.setJointValueTarget(joint_group_positions);
  //   arm.move();
  //   ROS_INFO("arrive to B");
  //   sleep(1);
    
  //    // 设置机器人终端的目标位置
  //   geometry_msgs::Pose target_pose;
  //   ////base_link
  //   // target_pose.orientation.x = 0.689;
  //   // target_pose.orientation.y = 0.723;
  //   // target_pose.orientation.z = -0.047;
  //   // target_pose.orientation.w = -0.015;

  //   // target_pose.position.x = -0.635;
  //   // target_pose.position.y = 0.077;
  //   // target_pose.position.z = 0.483;

  //   ////base
  //   target_pose.orientation.x = 0.723;
  //   target_pose.orientation.y = -0.689;
  //   target_pose.orientation.z = 0.015;
  //   target_pose.orientation.w = -0.046;

  //   target_pose.position.x = 0.635;
  //   target_pose.position.y = -0.076;
  //   target_pose.position.z = 0.482;


  //    // 设置机器臂当前的状态作为运动初始状态
  //   arm.setStartStateToCurrentState();

  //   arm.setPoseTarget(target_pose);

  //   // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
  //   moveit::planning_interface::MoveGroupInterface::Plan plan;
  //   moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);

  //   ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");   

  //   //让机械臂按照规划的轨迹开始运动。
  //   if(success)
  //   {
  //       arm.execute(plan); 
  //       sleep(1);
  //       ROS_INFO("arrive to C");
  //   }     
  //   else
  //     ROS_INFO("can't arrive to C");

  // srv.request.motorID = 1;
  // srv.request.setpos = 50;
  // srv.request.setforce = 20;
  // if (ag95client.call(srv)) 
  // {
  //   ROS_INFO("AG95_state: %s", srv.response.start_state.c_str());
  // } 
  // else 
  // {
  //   ROS_ERROR("Failed to call service");
  //   return 1;
  // }

    // // 控制机械臂先回到初始化位置
    // arm.setNamedTarget("lift");
    // arm.move();
    // sleep(1);

    ros::shutdown(); 

    return 0;
}
