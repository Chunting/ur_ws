#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <fstream>
#include <ctime>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lesson_move_group");

  // start a background "spinner", so our node can process ROS messages
  //  - this lets us know when the move is completed
  ros::AsyncSpinner spinner(1);
  spinner.start();

  clock_t start = clock();

  std::map<std::string, double> joints;
  joints["left_shoulder_pan_joint"] = 1.5708;
  joints["left_shoulder_lift_joint"] =  -3.14159;
  joints["left_elbow_joint"] =  0;
 //joints["left_elbow_joint"] =  0;
  joints["left_wrist_1_joint"] =  0;
  joints["left_wrist_2_joint"] =  1.5708;
  //joints["left_wrist_2_joint"] =  3.14159;
  joints["left_wrist_3_joint"] = 0;
// 0.345975,3.250664,5.139516,0.327431,1.351234,2.709953

  // joints["left_shoulder_pan_joint"] = 0.345975;
  // joints["left_shoulder_lift_joint"] =  3.250664;
  // joints["left_elbow_joint"] =  5.139516;
  // joints["left_wrist_1_joint"] =  0.327431;
  // joints["left_wrist_2_joint"] =  1.351234;
  // joints["left_wrist_3_joint"] = 2.709953;
  moveit::planning_interface::MoveGroup group("left_manipulator");
  clock_t ends = clock();
  ofstream time;
  time.open("time.txt");
  time << (double)(ends-start)/CLOCKS_PER_SEC << "\n" << endl;
  time.close();
  // move_group_interface::MoveGroup::Plan plan;
  // group.setJointValueTarget(joints);
  // if (!group.plan(plan))
  // {
  //   ROS_FATAL("Unable to create motion plan.  Aborting.");
  //   exit(-1);
  // }
  // group.asyncExecute(plan);

  group.setJointValueTarget(joints);
  group.move();
}
