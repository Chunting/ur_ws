#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/MultiArrayDimension.h"

double a[3];

void chatterCallback(const std_msgs::Float64MultiArray& msg)
{

     ROS_INFO("I heard: [%f,%f,%f]", msg.data[0],msg.data[1],msg.data[2]);
     a[0]=msg.data[0];
     a[1]=msg.data[1];
     a[2]=msg.data[2];
}
      
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lesson_move_group");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("manipulator");
  geometry_msgs::Pose pose;
  move_group_interface::MoveGroup::Plan plan;

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::Rate r(4);

  while (n.ok())
  {
     //a=0;
     ROS_INFO("I heard: [%f %f %f]", a[0],a[1],a[2]);
     ros::spinOnce();
     //ROS_INFO("I heard(2): [%f]", a);
     if(a[0]==0&&a[1]==0&&a[2]==0)
     {
       r.sleep();
       continue;
     }

     pose=group.getCurrentPose().pose;
     pose.position.x=pose.position.x+a[0];
     pose.position.y=pose.position.y+a[1];
     pose.position.z=pose.position.z+a[2];
     group.setPoseTarget(pose);

     if (!group.plan(plan))
     {
       ROS_FATAL("Unable to create motion plan.  Aborting.");
       exit(-1);
     }

  // do non-blocking move request
     group.asyncExecute(plan);

  // cancel motion after fixed time
     //sleep(1.0);
     //group.stop();
     //sleep(1.0);  // wait for stop command to be received
     a[0]=0;
     a[1]=0;
     a[2]=0;
     r.sleep();
  }

     group.stop();
     sleep(1.0);  // wait for stop command to be received
     return 0;
}
