#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "robotiq_force_torque_sensor/ft_sensor.h"
#include "robotiq_force_torque_sensor/sensor_accessor.h"
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
const double PI=M_PI;
const double ZERO_THRESH = 0.00000001;
//std::vector<double> current_joint_values;
double* current_joint_values =new double[6];
double current_force_z =0;

struct Ur5Interface{
  //#define UR5_PARAMS
  const double d1 =  0.089159;
  const double a2 = -0.42500;
  const double a3 = -0.39225;
  const double d4 =  0.10915;
  const double d5 =  0.09465;
  const double d6 =  0.0823;
  
  tf::TransformListener tfListener;  // tf listener to data from external tf broadcaster
  tf::TransformBroadcaster tfBroadcaster;  // broadcasts goal transform to tf to visualize it in rviz

  int SIGN(double x) {
      return (x > 0) - (x < 0);
  }

  // @param q       The 6 joint values 
  // @param T       The 4x4 end effector pose in row-major ordering
  void forward(const double* q, double* T) {
    double s1 = sin(*q), c1 = cos(*q); q++;
    double q234 = *q, s2 = sin(*q), c2 = cos(*q); q++;
    double s3 = sin(*q), c3 = cos(*q); q234 += *q; q++;
    q234 += *q; q++;
    double s5 = sin(*q), c5 = cos(*q); q++;
    double s6 = sin(*q), c6 = cos(*q); 
    double s234 = sin(q234), c234 = cos(q234);
    *T = ((c1*c234-s1*s234)*s5)/2.0 - c5*s1 + ((c1*c234+s1*s234)*s5)/2.0; T++;
    *T = (c6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0) - 
          (s6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0); T++;
    *T = (-(c6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0 - 
          s6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0)); T++;
    *T = ((d5*(s1*c234-c1*s234))/2.0 - (d5*(s1*c234+c1*s234))/2.0 - 
          d4*s1 + (d6*(c1*c234-s1*s234)*s5)/2.0 + (d6*(c1*c234+s1*s234)*s5)/2.0 - 
          a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3); T++;
    *T = c1*c5 + ((s1*c234+c1*s234)*s5)/2.0 + ((s1*c234-c1*s234)*s5)/2.0; T++;
    *T = (c6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0) + 
          s6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0)); T++;
    *T = (c6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0) - 
          s6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0)); T++;
    *T = ((d5*(c1*c234-s1*s234))/2.0 - (d5*(c1*c234+s1*s234))/2.0 + d4*c1 + 
          (d6*(s1*c234+c1*s234)*s5)/2.0 + (d6*(s1*c234-c1*s234)*s5)/2.0 + d6*c1*c5 - 
          a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3); T++;
    *T = ((c234*c5-s234*s5)/2.0 - (c234*c5+s234*s5)/2.0); T++;
    *T = ((s234*c6-c234*s6)/2.0 - (s234*c6+c234*s6)/2.0 - s234*c5*c6); T++;
    *T = (s234*c5*s6 - (c234*c6+s234*s6)/2.0 - (c234*c6-s234*s6)/2.0); T++;
    *T = (d1 + (d6*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3) + a2*s2 - 
         (d6*(c234*c5+s234*s5))/2.0 - d5*c234); T++;
    *T = 0.0; T++; *T = 0.0; T++; *T = 0.0; T++; *T = 1.0;
  }
  
  // @param q       The 6 joint values 
  // @param Ti      The 4x4 link i pose in row-major ordering. If NULL, nothing is stored.
  void forward_all(const double* q, double* T1, double* T2, double* T3,   double* T4, double* T5, double* T6) {
      double s1 = sin(*q), c1 = cos(*q); q++; // q1
      double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q); q++; // q2
      double s3 = sin(*q), c3 = cos(*q); q23 += *q; q234 += *q; q++; // q3
      q234 += *q; q++; // q4
      double s5 = sin(*q), c5 = cos(*q); q++; // q5
      double s6 = sin(*q), c6 = cos(*q); // q6
      double s23 = sin(q23), c23 = cos(q23);
      double s234 = sin(q234), c234 = cos(q234);

      if(T1 != NULL) {
        *T1 = c1; T1++;
        *T1 = 0; T1++;
        *T1 = s1; T1++;
        *T1 = 0; T1++;
        *T1 = s1; T1++;
        *T1 = 0; T1++;
        *T1 = -c1; T1++;
        *T1 = 0; T1++;
        *T1 =       0; T1++;
        *T1 = 1; T1++;
        *T1 = 0; T1++;
        *T1 =d1; T1++;
        *T1 =       0; T1++;
        *T1 = 0; T1++;
        *T1 = 0; T1++;
        *T1 = 1; T1++;
      }

      if(T2 != NULL) {
        *T2 = c1*c2; T2++;
        *T2 = -c1*s2; T2++;
        *T2 = s1; T2++;
        *T2 =a2*c1*c2; T2++;
        *T2 = c2*s1; T2++;
        *T2 = -s1*s2; T2++;
        *T2 = -c1; T2++;
        *T2 =a2*c2*s1; T2++;
        *T2 =         s2; T2++;
        *T2 = c2; T2++;
        *T2 = 0; T2++;
        *T2 =   d1 + a2*s2; T2++;
        *T2 =               0; T2++;
        *T2 = 0; T2++;
        *T2 = 0; T2++;
        *T2 =                 1; T2++;
      }

      if(T3 != NULL) {
        *T3 = c23*c1; T3++;
        *T3 = -s23*c1; T3++;
        *T3 = s1; T3++;
        *T3 =c1*(a3*c23 + a2*c2); T3++;
        *T3 = c23*s1; T3++;
        *T3 = -s23*s1; T3++;
        *T3 = -c1; T3++;
        *T3 =s1*(a3*c23 + a2*c2); T3++;
        *T3 =         s23; T3++;
        *T3 = c23; T3++;
        *T3 = 0; T3++;
        *T3 =     d1 + a3*s23 + a2*s2; T3++;
        *T3 =                    0; T3++;
        *T3 = 0; T3++;
        *T3 = 0; T3++;
        *T3 =                                     1; T3++;
      }

      if(T4 != NULL) {
        *T4 = c234*c1; T4++;
        *T4 = s1; T4++;
        *T4 = s234*c1; T4++;
        *T4 =c1*(a3*c23 + a2*c2) + d4*s1; T4++;
        *T4 = c234*s1; T4++;
        *T4 = -c1; T4++;
        *T4 = s234*s1; T4++;
        *T4 =s1*(a3*c23 + a2*c2) - d4*c1; T4++;
        *T4 =         s234; T4++;
        *T4 = 0; T4++;
        *T4 = -c234; T4++;
        *T4 =                  d1 + a3*s23 + a2*s2; T4++;
        *T4 =                         0; T4++;
        *T4 = 0; T4++;
        *T4 = 0; T4++;
        *T4 =                                                  1; T4++;
      }

      if(T5 != NULL) {
        *T5 = s1*s5 + c234*c1*c5; T5++;
        *T5 = -s234*c1; T5++;
        *T5 = c5*s1 - c234*c1*s5; T5++;
        *T5 =c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1; T5++;
        *T5 = c234*c5*s1 - c1*s5; T5++;
        *T5 = -s234*s1; T5++;
        *T5 = - c1*c5 - c234*s1*s5; T5++;
        *T5 =s1*(a3*c23 + a2*c2) - d4*c1 + d5*s234*s1; T5++;
        *T5 =                           s234*c5; T5++;
        *T5 = c234; T5++;
        *T5 = -s234*s5; T5++;
        *T5 =                          d1 + a3*s23 + a2*s2 - d5*c234; T5++;
        *T5 =                                                   0; T5++;
        *T5 = 0; T5++;
        *T5 = 0; T5++;
        *T5 =                                                                                 1; T5++;
      }

      if(T6 != NULL) {
        *T6 =   c6*(s1*s5 + c234*c1*c5) - s234*c1*s6; T6++;
        *T6 = - s6*(s1*s5 + c234*c1*c5) - s234*c1*c6; T6++;
        *T6 = c5*s1 - c234*c1*s5; T6++;
        *T6 =d6*(c5*s1 - c234*c1*s5) + c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1; T6++;
        *T6 = - c6*(c1*s5 - c234*c5*s1) - s234*s1*s6; T6++;
        *T6 = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1; T6++;
        *T6 = - c1*c5 - c234*s1*s5; T6++;
        *T6 =s1*(a3*c23 + a2*c2) - d4*c1 - d6*(c1*c5 + c234*s1*s5) + d5*s234*s1; T6++;
        *T6 =                                       c234*s6 + s234*c5*c6; T6++;
        *T6 = c234*c6 - s234*c5*s6; T6++;
        *T6 = -s234*s5; T6++;
        *T6 =                                                      d1 + a3*s23 + a2*s2 - d5*c234 - d6*s234*s5; T6++;
        *T6 =                                                                                                   0; T6++;
        *T6 = 0; T6++;
        *T6 = 0; T6++;
        *T6 =                                                                                                                                            1; T6++;
      }
  }

  // @param T       The 4x4 end effector pose in row-major ordering
    // @param q_sols  An 8x6 array of doubles returned, all angles should be in [0,2*PI)
    // @param q6_des  An optional parameter which designates what the q6 value should take
    //                in case of an infinite solution on that joint.
    // @return        Number of solutions found (maximum of 8)
  int inverse(const double* T, double* q_sols, double q6_des) {
    int num_sols = 0;
    double T02 = -*T; T++; double T00 =  *T; T++; double T01 =  *T; T++; double T03 = -*T; T++; 
    double T12 = -*T; T++; double T10 =  *T; T++; double T11 =  *T; T++; double T13 = -*T; T++; 
    double T22 =  *T; T++; double T20 = -*T; T++; double T21 = -*T; T++; double T23 =  *T;

    ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
    double q1[2];
    {
      double A = d6*T12 - T13;
      double B = d6*T02 - T03;
      double R = A*A + B*B;
      if(fabs(A) < ZERO_THRESH) {
        double div;
        if(fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
          div = -SIGN(d4)*SIGN(B);
        else
          div = -d4/B;
        double arcsin = asin(div);
        if(fabs(arcsin) < ZERO_THRESH)
          arcsin = 0.0;
        if(arcsin < 0.0)
          q1[0] = arcsin + 2.0*PI;
        else
          q1[0] = arcsin;
        q1[1] = PI - arcsin;
      }
      else if(fabs(B) < ZERO_THRESH) {
        double div;
        if(fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
          div = SIGN(d4)*SIGN(A);
        else
          div = d4/A;
        double arccos = acos(div);
        q1[0] = arccos;
        q1[1] = 2.0*PI - arccos;
      }
      else if(d4*d4 > R) {
        return num_sols;
      }
      else {
        double arccos = acos(d4 / sqrt(R)) ;
        double arctan = atan2(-B, A);
        double pos = arccos + arctan;
        double neg = -arccos + arctan;
        if(fabs(pos) < ZERO_THRESH)
          pos = 0.0;
        if(fabs(neg) < ZERO_THRESH)
          neg = 0.0;
        if(pos >= 0.0)
          q1[0] = pos;
        else
          q1[0] = 2.0*PI + pos;
        if(neg >= 0.0)
          q1[1] = neg; 
        else
          q1[1] = 2.0*PI + neg;
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
    double q5[2][2];
    {
      for(int i=0;i<2;i++) {
        double numer = (T03*sin(q1[i]) - T13*cos(q1[i])-d4);
        double div;
        if(fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
          div = SIGN(numer) * SIGN(d6);
        else
          div = numer / d6;
        double arccos = acos(div);
        q5[i][0] = arccos;
        q5[i][1] = 2.0*PI - arccos;
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    {
      for(int i=0;i<2;i++) {
        for(int j=0;j<2;j++) {
          double c1 = cos(q1[i]), s1 = sin(q1[i]);
          double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
          double q6;
          ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
          if(fabs(s5) < ZERO_THRESH)
            q6 = q6_des;
          else {
            q6 = atan2(SIGN(s5)*-(T01*s1 - T11*c1), 
                      SIGN(s5)*(T00*s1 - T10*c1));
            if(fabs(q6) < ZERO_THRESH)
              q6 = 0.0;
            if(q6 < 0.0)
              q6 += 2.0*PI;
          }
          ////////////////////////////////////////////////////////////////////////////////

          double q2[2], q3[2], q4[2];
          ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
          double c6 = cos(q6), s6 = sin(q6);
          double x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
          double x04y = c5*(T20*c6 - T21*s6) - T22*s5;
          double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) + 
                        T03*c1 + T13*s1;
          double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);

          double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
          if(fabs(fabs(c3) - 1.0) < ZERO_THRESH)
            c3 = SIGN(c3);
          else if(fabs(c3) > 1.0) {
            // TODO NO SOLUTION
            continue;
          }
          double arccos = acos(c3);
          q3[0] = arccos;
          q3[1] = 2.0*PI - arccos;
          double denom = a2*a2 + a3*a3 + 2*a2*a3*c3;
          double s3 = sin(arccos);
          double A = (a2 + a3*c3), B = a3*s3;
          q2[0] = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
          q2[1] = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
          double c23_0 = cos(q2[0]+q3[0]);
          double s23_0 = sin(q2[0]+q3[0]);
          double c23_1 = cos(q2[1]+q3[1]);
          double s23_1 = sin(q2[1]+q3[1]);
          q4[0] = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
          q4[1] = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);
          ////////////////////////////////////////////////////////////////////////////////
          for(int k=0;k<2;k++) {
            if(fabs(q2[k]) < ZERO_THRESH)
              q2[k] = 0.0;
            else if(q2[k] < 0.0) q2[k] += 2.0*PI;
            if(fabs(q4[k]) < ZERO_THRESH)
              q4[k] = 0.0;
            else if(q4[k] < 0.0) q4[k] += 2.0*PI;
            q_sols[num_sols*6+0] = q1[i];    q_sols[num_sols*6+1] = q2[k]; 
            q_sols[num_sols*6+2] = q3[k];    q_sols[num_sols*6+3] = q4[k]; 
            q_sols[num_sols*6+4] = q5[i][j]; q_sols[num_sols*6+5] = q6; 
            num_sols++;
          }

        }
      }
    }
    return num_sols;
  }

  int findClosestIK(const double* T, double* q_sol, double q6_des, double* current_joint_values){
    double q_sols[8*6];
    int a=inverse(T, q_sols, q6_des);
    if(a<4){
      printf("no enough solution can be found");
      return -1;
    }

    for (int i=0; i<a;i++){
      for (int j=0; j<6;j++){
          while (q_sols[i*6+j]-current_joint_values[j]>PI){
              q_sols[i*6+j]=q_sols[i*6+j]-2*PI;
          }
          while (current_joint_values[j]-q_sols[i*6+j]>PI){
              q_sols[i*6+j]=q_sols[i*6+j]+2*PI;
          }     
      }
    }

    double d[a];
    for(int i=0;i<a;i++){
      d[i]=0;
    }

    for (int i=0; i<a;i++){
      for (int j=0; j<6;j++){
        d[i]=d[i]+(q_sols[i*6+j]-current_joint_values[j])*(q_sols[i*6+j]-current_joint_values[j]);
      }
    }
    
    int index=0;
    for(int i=1;i<a;i++){
      if(d[i]<d[index]){
        index=i;
      }
    }

    for (int i=0; i<6; i++){
      q_sol[i]=q_sols[index*6+i];
    }
    return 0;
  }

};

void Callback(const sensor_msgs::JointState& msg)
{
   for (int i=0; i<6;i++){
     current_joint_values[i]=msg.position[i];
   }
}

void reCallback(const robotiq_force_torque_sensor::ft_sensor& msg)
{
  current_force_z = msg.Fz;
	ROS_INFO("I heard: FX[%f] FY[%f] FZ[%f] MX[%f] MY[%f] MZ[%f]", msg.Fx,msg.Fy,msg.Fz,msg.Mx,msg.My,msg.Mz);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  //std::vector<double> current_joint_values;

  ros::Publisher UrScript_pub = n.advertise<std_msgs::String>("/ur_driver/URScript", 1000);

  ros::Publisher command_pub = n.advertise<std_msgs::Float64>("/command", 1000);

  ros::Subscriber UrStates_sub = n.subscribe("/joint_states", 1000, Callback);

  ros::ServiceClient client = n.serviceClient<robotiq_force_torque_sensor::sensor_accessor>("robotiq_force_torque_sensor_acc");
	ros::Subscriber sub1 = n.subscribe("robotiq_force_torque_sensor",100,reCallback);
  robotiq_force_torque_sensor::sensor_accessor srv;

  ros::Rate loop_rate(125);
  
  int count = 0;

  std_msgs::Float64 command;

  command.data = 0;

  // double pos=0;

   double vel=0;

   double Gain=1;

   double K=800;

  srv.request.command_id = srv.request.COMMAND_SET_ZERO;

  if(client.call(srv)){
    ROS_INFO("ret: %s", srv.response.res.c_str());
  }

  while (ros::ok())
  {
    loop_rate.sleep();

    std_msgs::String msg;

    std::stringstream ss;

    struct Ur5Interface ur5;

    double* T = new double[16];
    double* q_sol =new double[6];
    double* j_vel =new double[6];
    double* end_pos =new double[3];

    ros::spinOnce();

    for (int i=0; i<6;i++){
     q_sol[i] = current_joint_values[i];
    }

    ur5.forward(q_sol, T);

    ROS_INFO("the current end position is: %f,%f,%f", T[3],T[7],T[11]);

    tf::Vector3 dp(T[0],T[4],T[8]);
    double dl=0;
    if (current_force_z>3 || current_force_z<-3){
      dl=-1*current_force_z/K;
    }

    dp.operator*=(dl);

    T[3]=T[3]-dp.x();
    T[7]=T[7]-dp.y();
    T[11]=T[11]-dp.z();

    ROS_INFO("the desired end position is: %f,%f,%f", T[3],T[7],T[11]);

    int success = -1;
    success = ur5.findClosestIK( T,  q_sol, 0.0 , current_joint_values); 
      
    if (success == -1)
    {
      continue;
    }

    for (int i=1;i<6;i++){
      j_vel[i]=Gain*(q_sol[i] - current_joint_values[i]);
    }

    command.data = j_vel[1];

    ss << "speedj([" << j_vel[0] <<"," << j_vel[1] <<"," << j_vel[2] <<"," << j_vel[3] <<"," << j_vel[4] <<"," << j_vel[5] << "],10,0.008)";

    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    UrScript_pub.publish(msg);

    command_pub.publish(command);

    // if(count == 10){
    //   srv.request.command_id = srv.request.COMMAND_SET_ZERO;

    //   if(client.call(srv)){
    //     ROS_INFO("ret: %s", srv.response.res.c_str());
    //   }
    //   count = 0;
    // }

    ros::spinOnce();


    ++count;
  }


  return 0;
}
