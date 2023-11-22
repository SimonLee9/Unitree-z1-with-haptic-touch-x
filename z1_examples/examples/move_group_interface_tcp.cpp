#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

geometry_msgs::Pose haptic_pose_msg;

void poseCallback(const geometry_msgs::Pose::ConstPtr &msg) {
    haptic_pose_msg.orientation.x = msg->orientation.x;
    haptic_pose_msg.orientation.y = msg->orientation.y;
    haptic_pose_msg.orientation.z = msg->orientation.z;
    haptic_pose_msg.orientation.w = msg->orientation.w;
    haptic_pose_msg.position.x = msg->position.x;
    haptic_pose_msg.position.y = msg->position.y;
    haptic_pose_msg.position.z = msg->position.z;

    haptic_pose_msg.position = msg->position;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(0);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  move_group_interface.setMaxVelocityScalingFactor(0.15);
  move_group_interface.setMaxAccelerationScalingFactor(0.5);

  const moveit::core::JointModelGroup* jmg = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  ros::Subscriber pose_sub = nh.subscribe("haptic_pose", 1,::poseCallback);

  /* Move to init position */
  
  geometry_msgs::Pose target_pose1;
  /*
  target_pose1.orientation.w = 0;
  target_pose1.position.x = 0.3; // origin : 0.3
  target_pose1.position.y = 0; // origin : 0
  target_pose1.position.z = 0.4; // origin : 0.4
  */

  //move_group_interface.setPoseTarget(target_pose1);
  move_group_interface.setPoseTarget(haptic_pose_msg);

  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  bool success = (move_group_interface.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

  if(success) {
    move_group_interface.execute(plan1);
  }

  // Cartesinan Paths
  // ^^^^^^^^^^^^^^^^
  std::vector<geometry_msgs::Pose> waypoints;
  //waypoints.push_back(target_pose1);
  waypoints.push_back(haptic_pose_msg);


  //geometry_msgs::Pose target_pose2 = target_pose1;
  geometry_msgs::Pose target_pose2 = haptic_pose_msg;

  target_pose2.position.x += 0.1;
  waypoints.push_back(target_pose2); // forward

  target_pose2.position.z -= 0.1;
  waypoints.push_back(target_pose2); // up

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threhold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threhold, trajectory);

  moveit::planning_interface::MoveGroupInterface::Plan plan2;
  plan2.trajectory_ = trajectory;
  move_group_interface.execute(plan2);
  

  //ros::shutdown();
  //ros::spin();
  ros::waitForShutdown();

  return 0;
}
