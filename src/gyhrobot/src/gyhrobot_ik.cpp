#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gyhrobot_ik");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    arm.allowReplanning(true);

    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.01);

    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);

    arm.setNamedTarget("up");
    arm.move();
    sleep(5);

    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = 0.70692;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.70729;

    target_pose.position.x = 0.2593;
    target_pose.position.y = 0.0636;
    target_pose.position.z = 0.1787;

    arm.setStartStateToCurrentState();

    arm.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);

    ROS_INFO("plan (pose goal) %s", success ? "":"FAILED");

    if (success)
        arm.execute(plan);

    sleep(5);

    ros::shutdown();

    return 0;
}