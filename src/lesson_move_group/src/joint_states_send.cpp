 #include <stdio.h>
#include <strings.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
//#include <linux/in.h>
#include <stdlib.h>
#include <memory.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h> //添加信号处理 防止向已断开的连接通信

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#define PORT 11910
#define Buflen 1024

double a[6];

void sig_pipe(int signo); //用户注册的信号函数,接收的是信号值
int s; //全局变量 ， 存储套接字描述符

void chatterCallback(const sensor_msgs::JointState& msg )
{
     ROS_INFO("I heard: [%f,%f,%f,%f,%f,%f]", msg.position[0],msg.position[1],msg.position[2],msg.position[3],msg.position[4],msg.position[5]);
     a[0]=msg.position[0];
     a[1]=msg.position[1];
     a[2]=msg.position[2];
     a[3]=msg.position[3];
     a[4]=msg.position[4];
     a[5]=msg.position[5];
     write(s,a,48);
}

int main(int argc,char **argv)
{
  sockaddr_in server_addr;
  int err;
  sighandler_t ret;

  /********************socket()*********************/
  s= socket(AF_INET,SOCK_STREAM,0);
  if(s<0)
  {
     printf("client : create socket error\n");
     return 1;
  }
     printf("client : socket fd = %d\n", s);
//信号处理函数 SIGINT 是当用户按一个 Ctrl-C 建时发送的信号
     ret = signal(SIGTSTP,sig_pipe);
     if(SIG_ERR == ret)
     {
        printf("信号挂接失败\n");
        return -1;
     }
     else
     printf("信号挂接成功\n") ;
/*******************connect()*********************/
//设置服务器地址结构，准备连接到服务器
    memset(&server_addr,0,sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

   server_addr.sin_addr.s_addr = inet_addr("192.168.1.101");
   err = connect(s,(struct sockaddr *)&server_addr,sizeof(sockaddr));
   if(err == 0)
   {
      printf("client : connect to server\n");
   }
   else
   {
      printf("client : connect error\n");
     return -1;
}

  ros::init(argc, argv, "joint_states_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("joint_states", 1000, chatterCallback);
  ros::spin();
  close(s);
  return 0;
}

void sig_pipe(int signo) //传入套接字描述符
{
   printf("Catch a signal\n");
   if(signo == SIGTSTP)
   {
     printf("接收到 SIGTSTP 信号\n");
     int ret = close(s);
     if(ret == 0)
     printf("成功 : 关闭套接字\n");
     else if(ret ==-1 )
     printf("失败 : 未关闭套接字\n");
      exit(1);
    }
} 
