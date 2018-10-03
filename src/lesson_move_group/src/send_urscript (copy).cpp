#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher UrScript_pub = n.advertise<std_msgs::String>("/ur_driver/URScript", 1000);

  ros::Rate loop_rate(0.5);
  
  int count = 0;

  int minus_one = -1 ;

  int sign_v = 1;

  float v = 0.1 ;

  while (ros::ok() && count<6)
  {
    loop_rate.sleep();

    std_msgs::String msg;

    std::stringstream ss;
    
    sign_v = minus_one*sign_v ;

    float vel =sign_v*v;

    ss << "speedj([0,0,0,0," << vel << ",0],0.2,2)";

    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    ros::spinOnce();

    UrScript_pub.publish(msg);

    ros::spinOnce();


    ++count;
  }


  return 0;
}
