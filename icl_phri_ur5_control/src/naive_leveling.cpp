#include <ros/ros.h>

#include "icl_phri_ur5_control/naive_leveling.h"

namespace ur5_control
{
NaiveLeveling::NaiveLeveling(ros::NodeHandle &nh):
    group_name_("manipilator"),
    move_group_(new moveit::planning_interface::MoveGroupInterface(group_name_))
{
    const robot_state::JointModelGroup * joint_model_group =
        move_group_.getCurrentState()->getJointModelGroup(group_name_);
}

bool NaiveLeveling::planMove(const geometry_msgs::Pose::ConstPtr &target_pose)
{
    move_group_.setPoseTarget(*target_pose);
    return (move_group_.plan(my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("Leveling", "Plan (pose goal) %s", success ? "" : "FAILED");
}

bool NaiveLeveling::planMove(double x, double y, double z, double qw)
{
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = qw;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    move_group.setPoseTarget(target_pose);
    return (move_group_.plan(my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("Leveling", "Plan (pose goal) %s", success ? "" : "FAILED");
}

bool NaiveLeveling::planMove(double x, double y, double z, double r, double p, double y)
{
    geometry_msgs::Pose target_pose;
    target_pose.orientation = tf::createQuaternionFromRPY(r, p, y);
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    move_group.setPoseTarget(target_pose);
    return (move_group_.plan(my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("Leveling", "Plan (pose goal) %s", success ? "" : "FAILED");
}
} // namespace ur5_control