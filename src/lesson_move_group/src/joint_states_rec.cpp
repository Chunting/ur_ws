#include <stdio.h>    
#include <stdlib.h>    
#include <strings.h>    
#include <sys/types.h>    
#include <sys/socket.h>    
#include <memory.h>    
#include <unistd.h>    
//#include <linux/in.h>    
#include <netinet/in.h>    
//#include <linux/inet_diag.h>    
#include <arpa/inet.h>    
    
#include <signal.h>    


#include "ros/ros.h"
#include <sstream>
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/MultiArrayDimension.h"
    
#define PORT    11910   //定义通信端口    
#define BACKLOG 5       //定义侦听队列长度    
#define buflen  1024    
    
void process_conn_server(int s,ros::Publisher *t);    
void sig_pipe(int signo);    
    
int ss,sc;  //ss为服务器socket描述符，sc为某一客户端通信socket描述符
   
std_msgs::Float64MultiArray joint_states_rec;

int main(int argc,char *argv[])    
{    
    ros::init(argc, argv, "joint_states_talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("joint_states_rec", 1000);

    joint_states_rec.layout.dim.push_back(std_msgs::MultiArrayDimension());
    joint_states_rec.layout.dim[0].size=6;
    joint_states_rec.layout.dim[0].stride=6;
    joint_states_rec.layout.dim[0].label="con";
    joint_states_rec.data.resize(6);
    joint_states_rec.data[0]=0;
    joint_states_rec.data[1]=0;
    joint_states_rec.data[2]=0;
    joint_states_rec.data[3]=0;
    joint_states_rec.data[4]=0;
    joint_states_rec.data[5]=0;
    chatter_pub.publish(joint_states_rec);
    ros::spinOnce();
    struct sockaddr_in server_addr; //存储服务器端socket地址结构    
    struct sockaddr_in client_addr; //存储客户端 socket地址结构    
    
    int err;    //返回值    
    pid_t pid;  //分叉进行的ID    
    
    /*****************socket()***************/    
    ss = socket(AF_INET,SOCK_STREAM,0); //建立一个序列化的，可靠的，双向连接的的字节流    
    if(ss<0)    
    {    
        printf("server : server socket create error\n");    
        return -1;    
    }    
    //注册信号    
    sighandler_t ret;    
    ret = signal(SIGTSTP,sig_pipe);    
    if(SIG_ERR == ret)    
    {    
        printf("信号挂接失败\n");    
        return -1;    
    }    
    else    
        printf("信号挂接成功\n");    
    
    
    /******************bind()****************/    
    //初始化地址结构    
    memset(&server_addr,0,sizeof(server_addr));    
    server_addr.sin_family = AF_INET;           //协议族    
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);   //本地地址    
    server_addr.sin_port = htons(PORT);    
    
    err = bind(ss,(struct sockaddr *)&server_addr,sizeof(sockaddr));    
    if(err<0)    
    {    
        printf("server : bind error\n");    
        return -1;    
    }    
    
    /*****************listen()***************/    
    err = listen(ss,BACKLOG);   //设置监听的队列大小    
    if(err < 0)    
    {    
        printf("server : listen error\n");    
        return -1;    
    }    
    

    
    while(ros::ok())    
    {    
        socklen_t addrlen = sizeof(client_addr);    
        //accept返回客户端套接字描述符    
        sc = accept(ss,(struct sockaddr *)&client_addr,&addrlen);  //注，此处为了获取返回值使用 指针做参数    
        if(sc < 0)  //出错    
        {    
            continue;   //结束此次循环    
        }    
        else    
        {    
            printf("server : connected\n");    
        }    
    
   
            close(ss); 

            process_conn_server(sc,&chatter_pub);
  
            close(sc);    
   
    }
   return 0;    
}    
    
/**  
  服务器对客户端连接处理过程；先读取从客户端发送来的数据，  
  然后将接收到的数据的字节的个数发送到客户端  
  */    
    
//通过套接字 s 与客户端进行通信    
void process_conn_server(int s,ros::Publisher *t)    
{    
    ssize_t size = 0;    
    char buffer[buflen];  //定义数据缓冲区 
 
    while(ros::ok())     
    {    
        //等待读    
        for(size = 0;size == 0 ;size = read(s,buffer,buflen));    
        //输出从客户端接收到的数据    
        printf("%s",buffer);    
        double control[6]={0};
        memcpy((char*)(&control),buffer,48);

        joint_states_rec.data[0]=control[0];
        joint_states_rec.data[1]=control[1];
        joint_states_rec.data[2]=control[2];
        joint_states_rec.data[3]=control[3];
        joint_states_rec.data[4]=control[4];
        joint_states_rec.data[5]=control[5];
        printf("%f,%f,%f,%f,%f,%f\n",joint_states_rec.data[0],joint_states_rec.data[1],joint_states_rec.data[2],joint_states_rec.data[3],joint_states_rec.data[4],joint_states_rec.data[5]);

        
        t->publish(joint_states_rec);
        ros::spinOnce();
        if(strcmp(buffer,"quit") == 0)    
        {    
            close(s);   //成功返回0，失败返回-1    
            return ;    
        }   
   
    }    
}    
void sig_pipe(int signo)    
{    
    printf("catch a signal\n");    
    if(signo == SIGTSTP)    
    {    
        printf("接收到 SIGTSTP 信号\n");    
        int ret1 = close(ss);    
        int ret2 = close(sc);    
        int ret = ret1>ret2?ret1:ret2;    
        if(ret == 0)    
            printf("成功 : 关闭套接字\n");    
        else if(ret ==-1 )    
            printf("失败 : 未关闭套接字\n");    
    
        exit(1);    
    }    
}  
