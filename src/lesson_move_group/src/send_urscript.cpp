#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

double current_joint4 = 0;

void Callback(const sensor_msgs::JointState& msg)
{
   current_joint4 = msg.position[4];
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  std::vector<double> current_joint_values;

  ros::Publisher UrScript_pub = n.advertise<std_msgs::String>("/ur_driver/URScript", 1000);

  ros::Publisher command_pub = n.advertise<std_msgs::Float64>("/command", 1000);

  ros::Subscriber UrStates_sub = n.subscribe("/joint_states", 1000, Callback);

  ros::Rate loop_rate(10);
  
  int count = 0;

    std_msgs::Float64 command;

    command.data = 0;

  while (ros::ok())
  {
    loop_rate.sleep();

    std_msgs::String msg;

    std::stringstream ss;

    double pos = 0.1*sin(0.1*count*0.1);

    command.data = pos;

    ss << "servoj([0,-1.5708,0,0," << pos << ",0],0.1,0.1,500)";

    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    UrScript_pub.publish(msg);

    command_pub.publish(command);

    ros::spinOnce();

    ROS_INFO("the current_joint4 is: %f", current_joint4*180/3.14159);

    ros::spinOnce();


    ++count;
  }


  return 0;
}
