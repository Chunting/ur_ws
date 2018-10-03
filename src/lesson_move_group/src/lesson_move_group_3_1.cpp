#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lesson_move_group");

  // start a background "spinner", so our node can process ROS messages
  //  - this lets us know when the move is completed
  ros::AsyncSpinner spinner(1);
  spinner.start();


  std::map<std::string, double> joints;
  joints["sim_shoulder_pan_joint"] = -0.4;
  joints["sim_shoulder_lift_joint"] =  -2;
  joints["sim_elbow_joint"] =  1.7;
  joints["sim_wrist_1_joint"] =  0.1;
  joints["sim_wrist_2_joint"] =  0.75;
  joints["sim_wrist_3_joint"] = -0.2;

  moveit::planning_interface::MoveGroup group("sim_manipulator");
  //group.setPlannerId("sim_move_gruop");
  //group.getDefaultPlannerId("manipulator");
  group.setJointValueTarget(joints);
  group.move();
}
