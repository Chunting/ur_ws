#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lesson_move_group");


  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("manipulator");

  geometry_msgs::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = 0.5;
  pose.position.z = 0.5;
  group.setPoseTarget(pose);
  group.move();

}
