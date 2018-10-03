#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lesson_move_group2");

  // start a background "spinner", so our node can process ROS messages
  //  - this lets us know when the move is completed
  ros::AsyncSpinner spinner(1);
  spinner.start();


  std::map<std::string, double> joints;
  joints["right_shoulder_pan_joint"] = 0;
  joints["right_shoulder_lift_joint"] =  0;
  joints["right_elbow_joint"] =  0.7854;
  joints["right_wrist_1_joint"] =  -1.5708;
  joints["right_wrist_2_joint"] =  -1.5708;
  joints["right_wrist_3_joint"] = 0;

  moveit::planning_interface::MoveGroup group2("right_manipulator");

  group2.setJointValueTarget(joints);
  group2.move();
}
