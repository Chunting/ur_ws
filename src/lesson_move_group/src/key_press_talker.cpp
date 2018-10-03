#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <sstream>
#include <linux/input.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include "std_msgs/String.h"

#define KEY_EVENT_DEV3_NAME "/dev/input/event3"

std_msgs::Float64MultiArray m;

int sysKeyScan(ros::Publisher *key_press_pub);

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher key_press_pub = n.advertise<std_msgs::Float64MultiArray>("chatter", 1000);

    m.layout.dim.push_back(std_msgs::MultiArrayDimension());
    m.layout.dim[0].size=3;
    m.layout.dim[0].stride=3;
    m.layout.dim[0].label="con";
    m.data.resize(3);
    m.data[0]=0;
    m.data[1]=0;
    m.data[2]=0;
    key_press_pub.publish(m);
    ros::spinOnce();

   sysKeyScan(&key_press_pub);


  return 0;
}

int sysKeyScan(ros::Publisher *key_press_pub)
{
  int l_ret=-1;
  int i=0;
  int key_fd=0;
  struct input_event key_event={0};

  key_fd=open(KEY_EVENT_DEV3_NAME, O_RDONLY);
  if(key_fd<=0)
  {
    printf("open error! %d\n", key_fd);
    return l_ret;
  }
  while(1)
  {
    l_ret = lseek(key_fd,0,SEEK_SET);
    l_ret = read(key_fd, &key_event, sizeof(key_event));
    if(l_ret)
    {
       if(key_event.type==EV_KEY&&(key_event.value==0||key_event.value==1))
       {
          printf("key %d %s\n",key_event.code,(key_event.value)?"pressed":"released");

         switch(key_event.code)
         {
           case 16:
           {
              m.data[0]=-0.1;
              m.data[1]=0;
              m.data[2]=0;
              printf("%f,%f,%f\n",m.data[0],m.data[1],m.data[2]);
              key_press_pub->publish(m);
              ros::spinOnce();
              //printf("key %d q\n",key_event.code);
              break;
           }
           case 17:
           {
              m.data[0]=0;
              m.data[1]=0;
              m.data[2]=0.1;
              printf("%f,%f,%f\n",m.data[0],m.data[1],m.data[2]);
              key_press_pub->publish(m);
              ros::spinOnce();
              //printf("key %d w\n",key_event.code);
              break;
           }
           case 30:
           {
              m.data[0]=0;
              m.data[1]=-0.1;
              m.data[2]=0;
              printf("%f,%f,%f\n",m.data[0],m.data[1],m.data[2]);
              key_press_pub->publish(m);
              ros::spinOnce();
              //printf("key %d a\n",key_event.code);
              break;
           }
           case 31:
           {
              m.data[0]=0;
              m.data[1]=0;
              m.data[2]=-0.1;
              printf("%f,%f,%f\n",m.data[0],m.data[1],m.data[2]);
              key_press_pub->publish(m);
              ros::spinOnce();
              //printf("key %d s\n",key_event.code);
              break;
           }
           case 32:
           {
              m.data[0]=0;
              m.data[1]=0.1;
              m.data[2]=0;
              printf("%f,%f,%f\n",m.data[0],m.data[1],m.data[2]);
              key_press_pub->publish(m);
              ros::spinOnce();
              //printf("key %d d\n",key_event.code);
              break;
           }
           case 45:
           {
              m.data[0]=0.1;
              m.data[1]=0;
              m.data[2]=0;
              printf("%f,%f,%f\n",m.data[0],m.data[1],m.data[2]);
              key_press_pub->publish(m);
              ros::spinOnce();
              //printf("key %d x\n",key_event.code);
              break;
           }

         }

         if(key_event.code==KEY_ESC)
         {
           break;
         }

       }
    }
  }
  close(key_fd);
  l_ret = 1;
  return l_ret;
}

