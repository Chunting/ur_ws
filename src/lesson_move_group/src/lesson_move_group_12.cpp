#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/MultiArrayDimension.h"

//
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <string>

#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <ctime>

using namespace std;

const double PI=M_PI;
const double ZERO_THRESH = 0.00000001;
int control_mode=1;

int pushIntoTransformArray(std::string transOrigin,
                           std::string transDestination,
                           std::vector<tf::Transform>* transArray,
                           tf::TransformListener* tfListener) {
  tf::StampedTransform transform;
  try {
    tfListener->waitForTransform(transOrigin, transDestination, ros::Time(0),ros::Duration(3.0));
    tfListener->lookupTransform(transOrigin, transDestination, ros::Time(0),
                                transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
    return -1;
  }
  transArray->push_back(
      tf::Transform(transform.getRotation(), transform.getOrigin()));
  return 0;
}

void MinusPI2PI(double *Joints, int n){
    for (int i=0;i<n;i++){
      while (Joints[i]>PI){
        Joints[i]=Joints[i]-2*PI;
      }
      while (Joints[i]<=-1*PI){
        Joints[i]=Joints[i]+2*PI;
      }
    }
}

void matrix2XZYEuler(double* T, double* euler){
    double R00=*T; T++;
    double R01=*T; T++;
    double R02=*T; T++;
    double R10=*T; T++;
    double R11=*T; T++;
    double R12=*T; T++;
    double R20=*T; T++;
    double R21=*T; T++;
    double R22=*T; 
    euler[1]=-1*asin(R01);
    if (fabs(R01)<0.9999){
      euler[0]=atan2(R21,R11);
      euler[2]=atan2(R02,R00);
    }else {
      euler[0]=atan2(-1*R12,R22);
      euler[2]=0;
    }
}

int ik_first_two_joints (tf::Vector3 Pos, double *Joints1, double *Joints2){
    tf::Vector3 x(1.0,0.0,0.0);
    tf::Vector3 y(0.0,1.0,0.0);
    tf::Vector3 z(0.0,0.0,1.0);
    if(fabs(Pos.getX())<ZERO_THRESH && fabs(Pos.getY())<ZERO_THRESH)
    {
      if(Pos.getZ()>0)
      {
        Joints1[0]=0;
        Joints1[1]=-1*PI/2;
        Joints2[0]=0;
        Joints2[1]=-1*PI/2;
      }else{
        Joints1[0]=0;
        Joints1[1]=PI/2;
        Joints2[0]=0;
        Joints2[1]=PI/2;
      }

    }else{
      double b2=acos(Pos.dot(z)/Pos.length());
      double b1=PI/2-b2;
      tf::Vector3 l=z.cross(Pos);
      double b3=acos(-1*(l.dot(y))/l.length());

      if(Pos.getY()<=0){
        Joints1[0]=b3;
        Joints1[1]=-1*b1;
        Joints2[0]=b3+PI;
        Joints2[1]=-1*PI+b1;
      }else if(Pos.getY()>0){
        Joints1[0]=-1*b3;
        Joints1[1]=-1*b1;
        Joints2[0]=-1*b3+PI;
        Joints2[1]=-1*PI+b1;
      }
      MinusPI2PI(Joints1, 2);
      MinusPI2PI(Joints2, 2);
    }
    return 0;
}


struct tfPercNeuronInterface {
  
  std::vector<std::string> bodyJointsLeftArm; 
  std::vector<std::string> bodyJointsLeftHand;   

  tf::Transform transWorldLeftHumanBaseLink;
  tf::Transform transLeftHumanBaseLinkLeftHumanBase;
  tf::Transform transWorldLeftBaseLink;
  tf::Transform transWorldLeftBase;
  tf::Transform transLeftBaseLinkLeftHand;
  tf::Transform transLeftGoalPosition;
  tf::Transform transLeftBaseLinkLeftArm;

  tf::TransformListener tfListener;  // tf listener to data from external tf broadcaster
  tf::TransformBroadcaster tfBroadcaster;  // broadcasts goal transform to tf to visualize it in rviz

  tf::StampedTransform transform;

  int side=1;//1 for left; 2 for right;

  /* int gettransBaseLinkArm(int side) {
    
    std::vector<tf::Transform> transformArray;
    int success = 0;  
    // std::vector<std::string> bodyJointsArm; 

    // Calculate transformation matrix from ur# base_link to RightHand
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    if (side==1){
      success += pushIntoTransformArray("/Neck",  "/LeftBaseLink", &transformArray,
                                      &tfListener);

      if (success != 0) {
        //printf("\nNo tf Transformation found from left_base_link to Neck.");
        return -1;
      }

      for (int i = 0; i < bodyJointsLeftArm.size() - 1; i++) {
        success += pushIntoTransformArray(
            bodyJointsLeftArm.at(i), bodyJointsLeftArm.at(i + 1),
            &transformArray, &tfListener);  // returns 0 for success -1 for failure
        if (success != 0) {
          //printf("\nNo tf transformation found from Neck to LeftArm");
          return -1;
        }
      }

      transLeftBaseLinkLeftArm = transformArray.at(0).inverse();
      transLeftBaseLinkLeftArm = transLeftBaseLinkLeftArm.operator*=(transformArray.at(1));
      transLeftBaseLinkLeftArm = transLeftBaseLinkLeftArm.operator*=(transformArray.at(2));
    
      // tfBroadcaster.sendTransform(tf::StampedTransform(
      //     transLeftBaseLinkLeftArm, ros::Time::now(), "/LeftBaseLink", "/LeftBaseLinkToLeftArm"));
      // printf("\nsuccess to broadcast tf Transformation from left_base_link to leftarm.");
      transformArray.clear();
    }
       
    return 0;
  } */

  int gettransBaseLinkHand(int side) {
    
    std::vector<tf::Transform> transformArray;
    int success = 0;  

    // Calculate transformation matrix from ur# base_link to RightHand
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    if (side==1){
      success += pushIntoTransformArray("/left_human_base_link",  "/left_human_base", &transformArray,
                                      &tfListener);

      if (success != 0) {
        //printf("\nNo tf Transformation found from left_base_link to Neck.");
        return -1;
      }

      for (int i = 0; i < bodyJointsLeftArm.size() - 1; i++) {
        success += pushIntoTransformArray(
            bodyJointsLeftArm.at(i), bodyJointsLeftArm.at(i + 1),
            &transformArray, &tfListener);  // returns 0 for success -1 for failure
        if (success != 0) {
          //printf("\nNo tf transformation found from Neck to LeftHand");
          return -1;
        }
      }

      transLeftBaseLinkLeftHand = transformArray.at(0);
      tf::Transform transLeftHumanBaseLeftShoulder;
      transLeftHumanBaseLeftShoulder.setBasis(transformArray.at(1).getBasis());
      transLeftHumanBaseLeftShoulder.setOrigin(tf::Vector3(0,0,0));  
      transLeftBaseLinkLeftHand = transLeftBaseLinkLeftHand.operator*=(transLeftHumanBaseLeftShoulder);
      tfBroadcaster.sendTransform(tf::StampedTransform(transLeftHumanBaseLeftShoulder, ros::Time::now(), "/left_human_base", "/left_human_shoulder"));
      for(int i = 2; i < transformArray.size(); i++){
        transLeftBaseLinkLeftHand = transLeftBaseLinkLeftHand.operator*=(transformArray.at(i));
      }
      tfBroadcaster.sendTransform(tf::StampedTransform(transLeftBaseLinkLeftHand, ros::Time::now(), "/left_human_base_link", "/left_human_forearm"));
    
      transformArray.clear();

      transLeftBaseLinkLeftHand = transWorldLeftBaseLink.inverse().operator*(transLeftBaseLinkLeftHand);
      tf::Vector3 Pos= transLeftBaseLinkLeftHand.getOrigin();
      transLeftGoalPosition.setBasis(transLeftBaseLinkLeftHand.getBasis());
      Pos.operator*=(1.0/0.7);
      transLeftGoalPosition.setOrigin(Pos);

      tfBroadcaster.sendTransform(tf::StampedTransform(transLeftGoalPosition, ros::Time::now(), "/left_base_link", "/left_goal"));

      // printf("\nsuccess to broadcast tf Transformation from left_base_link to LeftGoalPosition.");
    }
    
        
    return 0;
  }

  /* int gettransBaseLinkHand2(int side, double* human_data) {
    
    std::vector<tf::Transform> transformArray;
    int success = 0;  
    std::vector<std::string> bodyJointsArm; 
    tf::Transform RotNeckShoulder;
    tf::Transform RotShoulderArm;
    tf::Transform RotArmForearm;
    tf::Transform RotForearmHand;
    tf::Transform transNeckShoulder;
    tf::Transform transShoulderArm;
    tf::Transform transArmForearm;
    tf::Transform transForearmHand;
    tf::Vector3 arm;
    tf::Vector3 forearm;
    tf::Vector3 hand;
    tf::Vector3 endPos;
    tf::Matrix3x3 wrist_rot;
    double roll; double pitch; double yaw;
    tf::Quaternion Quat;

    // Calculate transformation matrix from ur# base_link to RightHand
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    if (side==1){
      clock_t start = clock();

      success += pushIntoTransformArray("/Neck",  "/LeftBaseLink", &transformArray,
                                      &tfListener);

      if (success != 0) {
        printf("\nNo tf Transformation found from left_base_link to Neck.");
        return -1;
      }

      // for (int i = 0; i < bodyJointsLeftHand.size() - 1; i++) {
      //   success += pushIntoTransformArray(
      //       bodyJointsLeftHand.at(i), bodyJointsLeftHand.at(i + 1),
      //       &transformArray, &tfListener);  // returns 0 for success -1 for failure
      //   if (success != 0) {
      //     printf("\nNo tf transformation found from Neck to LeftHand");
      //     return -1;
      //   }
      // }

      try {
        tfListener.waitForTransform("/Neck", "/LeftShoulder", ros::Time(0),ros::Duration(3.0));
        tfListener.lookupTransform("/Neck", "/LeftShoulder", ros::Time(0),
                                    transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return -1;
      }
      RotNeckShoulder.setBasis(transform.getBasis());
      RotNeckShoulder.setOrigin(tf::Vector3(0,0,0));
      transNeckShoulder.setBasis(transform.getBasis());
      transNeckShoulder.setOrigin(transform.getOrigin());

      try {
        tfListener.waitForTransform("/LeftShoulder", "/LeftArm", ros::Time(0),ros::Duration(3.0));
        tfListener.lookupTransform("/LeftShoulder","/LeftArm",  ros::Time(0),
                                    transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return -1;
      }
      arm=transform.getOrigin();
      RotShoulderArm.setBasis(transform.getBasis());
      RotShoulderArm.setOrigin(tf::Vector3(0,0,0));  
      transShoulderArm.setBasis(transform.getBasis());
      transShoulderArm.setOrigin(transform.getOrigin());

      try {
        tfListener.waitForTransform("/LeftArm", "/LeftForeArm", ros::Time(0),ros::Duration(3.0));
        tfListener.lookupTransform("/LeftArm",  "/LeftForeArm",  ros::Time(0),
                                    transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return -1;
      }
      forearm=transform.getOrigin();
      RotArmForearm.setBasis(transform.getBasis());
      RotArmForearm.setOrigin(tf::Vector3(0,0,0)); 
      transArmForearm.setBasis(transform.getBasis());
      transArmForearm.setOrigin(transform.getOrigin());

      try {
      tfListener.waitForTransform("/LeftForeArm", "/LeftHand", ros::Time(0),ros::Duration(3.0));
      tfListener.lookupTransform("/LeftForeArm", "/LeftHand", ros::Time(0),
                                  transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return -1;
      }
      hand=transform.getOrigin();
      RotForearmHand.setBasis(transform.getBasis());
      RotForearmHand.setOrigin(tf::Vector3(0,0,0)); 
      transForearmHand.setBasis(transform.getBasis());
      transForearmHand.setOrigin(transform.getOrigin());

      tf::Transform transLeftUrWorldLeftHandWorld;
      Quat.setRPY(PI/2, 0, -PI/2);
      transLeftUrWorldLeftHandWorld.setRotation(Quat);
      transLeftUrWorldLeftHandWorld.setOrigin(tf::Vector3(0, 0, 0));
      arm=transLeftUrWorldLeftHandWorld*(RotNeckShoulder*(arm));
      forearm=transLeftUrWorldLeftHandWorld*(RotNeckShoulder*(RotShoulderArm*(forearm)));
      hand=transLeftUrWorldLeftHandWorld*(RotNeckShoulder*(RotShoulderArm*(RotArmForearm*(hand))));
      wrist_rot=transLeftUrWorldLeftHandWorld.operator*(RotNeckShoulder.operator*(RotShoulderArm.operator*(RotArmForearm.operator*(RotForearmHand)))).getBasis();
      wrist_rot.getRPY(roll,pitch,yaw);
      
      int human_data_count=0;
      human_data[human_data_count]=arm.getX(); human_data_count++;
      human_data[human_data_count]=arm.getY(); human_data_count++;
      human_data[human_data_count]=arm.getZ(); human_data_count++;
      human_data[human_data_count]=forearm.getX(); human_data_count++;
      human_data[human_data_count]=forearm.getY(); human_data_count++;
      human_data[human_data_count]=forearm.getZ(); human_data_count++;
      endPos=arm.operator+=(forearm);
      endPos.operator+=(hand);
      human_data[human_data_count]=endPos.getX(); human_data_count++;
      human_data[human_data_count]=endPos.getY(); human_data_count++;
      human_data[human_data_count]=endPos.getZ(); human_data_count++;
      human_data[human_data_count]=roll; human_data_count++;
      human_data[human_data_count]=pitch; human_data_count++;
      human_data[human_data_count]=yaw;

      transLeftBaseLinkLeftHand = transformArray.at(0).inverse();
      transLeftBaseLinkLeftHand = transLeftBaseLinkLeftHand.operator*=(transNeckShoulder);
      transLeftBaseLinkLeftHand = transLeftBaseLinkLeftHand.operator*=(transShoulderArm);
      transLeftBaseLinkLeftHand = transLeftBaseLinkLeftHand.operator*=(transArmForearm);
      transLeftBaseLinkLeftHand = transLeftBaseLinkLeftHand.operator*=(transForearmHand);

      // for(int i = 1; i < transformArray.size(); i++){
      //   transLeftBaseLinkLeftHand = transLeftBaseLinkLeftHand.operator*=(transformArray.at(i));
      // }
    
      tfBroadcaster.sendTransform(tf::StampedTransform(
          transLeftBaseLinkLeftHand, ros::Time::now(), "/LeftBaseLink", "/LeftBaseLinkToLeftHand"));
      printf("\nsuccess to broadcast tf Transformation from left_base_link to leftHand.");
      transformArray.clear();

      tf::Matrix3x3 Rot=transLeftBaseLinkLeftHand.getBasis();
      tf::Vector3 Pos= transLeftBaseLinkLeftHand.getOrigin();
      transLeftGoalPosition.setBasis(Rot);
      Pos.operator*=(1.0/0.7);
      transLeftGoalPosition.setOrigin(Pos);
      tfBroadcaster.sendTransform(tf::StampedTransform(
              transLeftGoalPosition, ros::Time::now(), "/left_base_link", "/LeftGoalPosition"));
      printf("\nsuccess to broadcast tf Transformation from left_base_link to LeftGoalPosition.");

      clock_t ends = clock();
      cout << "the time is " << (double)(ends-start)/CLOCKS_PER_SEC << "\n" << endl;
    }
    
    if (side==2){
      success += pushIntoTransformArray("/Neck",  "/RightBaseLink", &transformArray,
                                      &tfListener);

      if (success != 0) {
        printf("\nNo tf Transformation found from right_base_link to Neck.");
        return -1;
      }

      for (int i = 0; i < bodyJointsRightHand.size() - 1; i++) {
        success += pushIntoTransformArray(
            bodyJointsRightHand.at(i), bodyJointsRightHand.at(i + 1),
            &transformArray, &tfListener);  // returns 0 for success -1 for failure
        if (success != 0) {
          printf("\nNo tf transformation found from Neck to RightHand");
          return -1;
        }
      }     
      
      transRightBaseLinkRightHand = transformArray.at(0).inverse();
      for(int i = 1; i < transformArray.size(); i++){
        transRightBaseLinkRightHand = transRightBaseLinkRightHand.operator*=(transformArray.at(i));
      }
    
      tfBroadcaster.sendTransform(tf::StampedTransform(
          transRightBaseLinkRightHand, ros::Time::now(), "/RightBaseLink", "/RightBaseLinkToRightHand"));
      printf("\nsuccess to broadcast tf Transformation from right_base_link to rightHand.");
      transformArray.clear();

    }
    
    return 0;
  } */

  /* int gettransBaseLinkHand3(int side, double* human_data) {
    
    std::vector<tf::Transform> transformArray;
    int success = 0;  
    std::vector<std::string> bodyJointsArm; 
    tf::Transform RotNeckShoulder;
    tf::Transform RotShoulderArm;
    tf::Transform RotArmForearm;
    tf::Transform RotForearmHand;
    tf::Transform transNeckShoulder;
    tf::Transform transShoulderArm;
    tf::Transform transArmForearm;
    tf::Transform transForearmHand;
    tf::Vector3 arm;
    tf::Vector3 forearm;
    tf::Vector3 hand;
    tf::Vector3 endPos;
    tf::Matrix3x3 wrist_rot;
    double roll; double pitch; double yaw;
    tf::Quaternion Quat;

    // Calculate transformation matrix from ur# base_link to RightHand
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    if (side==1){

      success += pushIntoTransformArray("/Neck",  "/LeftBaseLink", &transformArray,
                                      &tfListener);

      if (success != 0) {
        // printf("\nNo tf Transformation found from left_base_link to Neck.");
        return -1;
      }

      // for (int i = 0; i < bodyJointsLeftHand.size() - 1; i++) {
      //   success += pushIntoTransformArray(
      //       bodyJointsLeftHand.at(i), bodyJointsLeftHand.at(i + 1),
      //       &transformArray, &tfListener);  // returns 0 for success -1 for failure
      //   if (success != 0) {
      //     printf("\nNo tf transformation found from Neck to LeftHand");
      //     return -1;
      //   }
      // }

      for (int i = 0; i < bodyJointsLeftArm.size() - 1; i++) {
        success += pushIntoTransformArray(
            bodyJointsLeftArm.at(i), bodyJointsLeftArm.at(i + 1),
            &transformArray, &tfListener);  // returns 0 for success -1 for failure
        if (success != 0) {
          return -1;
        }
      }

      RotNeckShoulder.setBasis(transformArray.at(1).getBasis());
      RotNeckShoulder.setOrigin(tf::Vector3(0,0,0));

      arm=transformArray.at(2).getOrigin();
      RotShoulderArm.setBasis(transformArray.at(2).getBasis());
      RotShoulderArm.setOrigin(tf::Vector3(0,0,0));  

      forearm=transformArray.at(3).getOrigin();
      RotArmForearm.setBasis(transformArray.at(3).getBasis());
      RotArmForearm.setOrigin(tf::Vector3(0,0,0)); 

      // hand=transform.getOrigin();
      // RotForearmHand.setBasis(transform.getBasis());
      // RotForearmHand.setOrigin(tf::Vector3(0,0,0)); 
      // transForearmHand.setBasis(transform.getBasis());
      // transForearmHand.setOrigin(transform.getOrigin());

      tf::Transform transLeftUrWorldLeftHandWorld;
      Quat.setRPY(PI/2, 0, -PI/2);
      transLeftUrWorldLeftHandWorld.setRotation(Quat);
      transLeftUrWorldLeftHandWorld.setOrigin(tf::Vector3(0, 0, 0));
      arm=transLeftUrWorldLeftHandWorld*(RotNeckShoulder*(arm));
      forearm=transLeftUrWorldLeftHandWorld*(RotNeckShoulder*(RotShoulderArm*(forearm)));
      // hand=transLeftUrWorldLeftHandWorld*(RotNeckShoulder*(RotShoulderArm*(RotArmForearm*(hand))));
      // wrist_rot=transLeftUrWorldLeftHandWorld.operator*(RotNeckShoulder.operator*(RotShoulderArm.operator*(RotArmForearm.operator*(RotForearmHand)))).getBasis();
      wrist_rot=transLeftUrWorldLeftHandWorld.operator*(RotNeckShoulder.operator*(RotShoulderArm.operator*(RotArmForearm))).getBasis();
      wrist_rot.getRPY(roll,pitch,yaw);
      
      int human_data_count=0;
      human_data[human_data_count]=arm.getX(); human_data_count++;
      human_data[human_data_count]=arm.getY(); human_data_count++;
      human_data[human_data_count]=arm.getZ(); human_data_count++;
      human_data[human_data_count]=forearm.getX(); human_data_count++;
      human_data[human_data_count]=forearm.getY(); human_data_count++;
      human_data[human_data_count]=forearm.getZ(); human_data_count++;
      endPos=arm.operator+=(forearm);
      // endPos.operator+=(hand);
      human_data[human_data_count]=endPos.getX(); human_data_count++;
      human_data[human_data_count]=endPos.getY(); human_data_count++;
      human_data[human_data_count]=endPos.getZ(); human_data_count++;
      human_data[human_data_count]=roll; human_data_count++;
      human_data[human_data_count]=pitch; human_data_count++;
      human_data[human_data_count]=yaw;

      transLeftBaseLinkLeftHand = transformArray.at(0).inverse();
      for(int i = 1; i < transformArray.size(); i++){
        transLeftBaseLinkLeftHand = transLeftBaseLinkLeftHand.operator*=(transformArray.at(i));
      }
      transformArray.clear();

      tf::Matrix3x3 Rot=transLeftBaseLinkLeftHand.getBasis();
      tf::Vector3 Pos= transLeftBaseLinkLeftHand.getOrigin();
      transLeftGoalPosition.setBasis(Rot);
      Pos.operator*=(1.0/0.7);
      transLeftGoalPosition.setOrigin(Pos);

    }
    
    // if (side==2){
    //   success += pushIntoTransformArray("/Neck",  "/RightBaseLink", &transformArray,
    //                                   &tfListener);

    //   if (success != 0) {
    //     printf("\nNo tf Transformation found from right_base_link to Neck.");
    //     return -1;
    //   }

    //   for (int i = 0; i < bodyJointsRightHand.size() - 1; i++) {
    //     success += pushIntoTransformArray(
    //         bodyJointsRightHand.at(i), bodyJointsRightHand.at(i + 1),
    //         &transformArray, &tfListener);  // returns 0 for success -1 for failure
    //     if (success != 0) {
    //       printf("\nNo tf transformation found from Neck to RightHand");
    //       return -1;
    //     }
    //   }     
      
    //   transRightBaseLinkRightHand = transformArray.at(0).inverse();
    //   for(int i = 1; i < transformArray.size(); i++){
    //     transRightBaseLinkRightHand = transRightBaseLinkRightHand.operator*=(transformArray.at(i));
    //   }
    
    //   tfBroadcaster.sendTransform(tf::StampedTransform(
    //       transRightBaseLinkRightHand, ros::Time::now(), "/RightBaseLink", "/RightBaseLinkToRightHand"));
    //   printf("\nsuccess to broadcast tf Transformation from right_base_link to rightHand.");
    //   transformArray.clear();

    // }
    
    return 0;
  } */

  int ik_newmap(int side, std::vector<double> current_joint_values,  std::map<std::string, double>& joints){
    tf::Vector3 ArmPos;
    double* Joints1=new double[2];
    double* Joints2=new double[2];
    tf::Quaternion Quat;
    double d1=0;
    double d2=0;
    double d=0;

    tf::Transform TransformOG;
    tf::Transform TransformOR;
    tf::Transform TransformGR;

    if(side==1){

      std::vector<tf::Transform> transformArray;
      int success = 0;  

      success += pushIntoTransformArray("/left_human_base_link",  "/left_human_base", &transformArray, &tfListener);

      if (success != 0) {
        //printf("\nNo tf Transformation found from left_base_link to Neck.");
        return -1;
      }

      for (int i = 0; i < bodyJointsLeftArm.size() - 1; i++) {
        success += pushIntoTransformArray(
            bodyJointsLeftArm.at(i), bodyJointsLeftArm.at(i + 1),
            &transformArray, &tfListener);  // returns 0 for success -1 for failure
        if (success != 0) {
          //printf("\nNo tf transformation found from Neck to LeftArm");
          return -1;
        }
      }

      transLeftBaseLinkLeftArm = transformArray.at(0);

      tf::Transform transLeftHumanBaseLeftShoulder;
      transLeftHumanBaseLeftShoulder.setBasis(transformArray.at(1).getBasis());
      transLeftHumanBaseLeftShoulder.setOrigin(tf::Vector3(0,0,0));  
      transLeftBaseLinkLeftArm = transLeftBaseLinkLeftArm.operator*=(transLeftHumanBaseLeftShoulder);
      tfBroadcaster.sendTransform(tf::StampedTransform(transLeftHumanBaseLeftShoulder, ros::Time::now(), "/left_human_base", "/left_human_shoulder"));

      transLeftBaseLinkLeftArm = transLeftBaseLinkLeftArm.operator*=(transformArray.at(2));
      tfBroadcaster.sendTransform(tf::StampedTransform(transLeftBaseLinkLeftArm, ros::Time::now(), "/left_human_base_link", "/left_human_arm"));

      transLeftBaseLinkLeftArm = transWorldLeftBaseLink.inverse().operator*(transLeftBaseLinkLeftArm);
      //compute the left ur5's joint values and execute
      ArmPos=transLeftBaseLinkLeftArm.getOrigin();
      ArmPos.setX(-1*ArmPos.getX());
      ArmPos.setY(-1*ArmPos.getY());
      int a= ik_first_two_joints (ArmPos,Joints1,Joints2);

      for(int j=0;j<2;j++){
          d=Joints2[j]-current_joint_values.at(j);
          if(d>PI){
            Joints2[j]=Joints2[j]-2*PI;
          }
          if(d<-1*PI){
            Joints2[j]=Joints2[j]+2*PI;
          }
      }
      joints["left_shoulder_pan_joint"] = Joints2[0];
      joints["left_shoulder_lift_joint"] =  Joints2[1];


      tf::Vector3 origin_shoulder_arm = transformArray.at(2).getOrigin();
      tf::Matrix3x3 rot_shoulder_arm = transformArray.at(2).getBasis();

      tf::Vector3 origin_arm_forearm = transformArray.at(3).getOrigin();

      
      tf::Transform tf0;
      tf0.setBasis(rot_shoulder_arm);
      tf0.setOrigin(tf::Vector3(0.0,0.0,0.0));
      origin_arm_forearm = tf0*(origin_arm_forearm);
      double elbow_angle = -1*(origin_shoulder_arm.angle(origin_arm_forearm));
      joints["left_elbow_joint"] = elbow_angle;

      tf::Transform transShoulderElbow;
      Quat.setRPY(0.0,elbow_angle,0.0);
      transShoulderElbow.setRotation(Quat);
      transShoulderElbow.setOrigin(tf::Vector3(0, 0, 0));

      tf::Transform transElbowArm = transShoulderElbow.inverseTimes(transformArray.at(2));
      tf::Transform transElbowWist = transElbowArm.operator*(transformArray.at(3));

      Quat.setRPY(PI,0,PI/2);
      TransformOR.setRotation(Quat);
      TransformOR.setOrigin(tf::Vector3(0, 0, 0));

      transElbowWist.setOrigin(tf::Vector3(0, 0, 0));


      TransformGR=TransformOR.operator*(transElbowWist);
      TransformGR=TransformGR.operator*(TransformOR.inverse());

      double* T = new double[9];
      double* euler = new double[3];
      T[0]=TransformGR.getBasis().getRow(0).getX();
      T[1]=TransformGR.getBasis().getRow(0).getY();
      T[2]=TransformGR.getBasis().getRow(0).getZ();


      T[3]=TransformGR.getBasis().getRow(1).getX();
      T[4]=TransformGR.getBasis().getRow(1).getY();
      T[5]=TransformGR.getBasis().getRow(1).getZ();

      T[6]=TransformGR.getBasis().getRow(2).getX();
      T[7]=TransformGR.getBasis().getRow(2).getY();
      T[8]=TransformGR.getBasis().getRow(2).getZ();

      matrix2XZYEuler(T, euler);

      joints["left_wrist_1_joint"] =  euler[0];
      joints["left_wrist_2_joint"] =  PI/2+euler[1]+PI/8;
      joints["left_wrist_3_joint"] = euler[2]+PI/18;//-0.8;

      transformArray.clear();

    }

    return 0;
  }

  int joint2joint_map(int side, std::vector<double> current_joint_values,  std::map<std::string, double>& joints){
    tf::Vector3 ArmPos;
    double* Joints1=new double[2];
    double* Joints2=new double[2];
    tf::Quaternion Quat;
    double d1=0;
    double d2=0;
    double d=0;

    tf::Transform TransformOG;
    tf::Transform TransformOR;
    tf::Transform TransformGR;

    if(side==1){

      std::vector<tf::Transform> transformArray;
      int success = 0;  

      success += pushIntoTransformArray("/left_human_base_link",  "/left_human_base", &transformArray, &tfListener);

      if (success != 0) {
        //printf("\nNo tf Transformation found from left_base_link to Neck.");
        return -1;
      }

      for (int i = 0; i < bodyJointsLeftArm.size() - 1; i++) {
        success += pushIntoTransformArray(
            bodyJointsLeftArm.at(i), bodyJointsLeftArm.at(i + 1),
            &transformArray, &tfListener);  // returns 0 for success -1 for failure
        if (success != 0) {
          //printf("\nNo tf transformation found from Neck to LeftArm");
          return -1;
        }
      }

      transLeftBaseLinkLeftArm = transformArray.at(0);

      tf::Transform transLeftHumanBaseLeftShoulder;
      transLeftHumanBaseLeftShoulder.setBasis(transformArray.at(1).getBasis());
      transLeftHumanBaseLeftShoulder.setOrigin(tf::Vector3(0,0,0));  
      transLeftBaseLinkLeftArm = transLeftBaseLinkLeftArm.operator*=(transLeftHumanBaseLeftShoulder);
      tfBroadcaster.sendTransform(tf::StampedTransform(transLeftHumanBaseLeftShoulder, ros::Time::now(), "/left_human_base", "/left_human_shoulder"));

      transLeftBaseLinkLeftArm = transLeftBaseLinkLeftArm.operator*=(transformArray.at(2));
      tfBroadcaster.sendTransform(tf::StampedTransform(transLeftBaseLinkLeftArm, ros::Time::now(), "/left_human_base_link", "/left_human_arm"));

      transLeftBaseLinkLeftArm = transWorldLeftBaseLink.inverse().operator*(transLeftBaseLinkLeftArm);
      //compute the left ur5's joint values and execute
      ArmPos=transLeftBaseLinkLeftArm.getOrigin();
      ArmPos.setX(-1*ArmPos.getX());
      ArmPos.setY(-1*ArmPos.getY());
      int a= ik_first_two_joints (ArmPos,Joints1,Joints2);

      for(int j=0;j<2;j++){
          d=Joints2[j]-current_joint_values.at(j);
          if(d>PI){
            Joints2[j]=Joints2[j]-2*PI;
          }
          if(d<-1*PI){
            Joints2[j]=Joints2[j]+2*PI;
          }
      }
      joints["left_shoulder_pan_joint"] = Joints2[0];
      joints["left_shoulder_lift_joint"] =  Joints2[1];


      tf::Vector3 origin_shoulder_arm = transformArray.at(2).getOrigin();
      tf::Matrix3x3 rot_shoulder_arm = transformArray.at(2).getBasis();

      tf::Vector3 origin_arm_forearm = transformArray.at(3).getOrigin();

      
      tf::Transform tf0;
      tf0.setBasis(rot_shoulder_arm);
      tf0.setOrigin(tf::Vector3(0.0,0.0,0.0));
      origin_arm_forearm = tf0*(origin_arm_forearm);
      double elbow_angle = -1*(origin_shoulder_arm.angle(origin_arm_forearm));
      joints["left_elbow_joint"] = elbow_angle;

      tf::Transform transShoulderElbow;
      Quat.setRPY(0.0,elbow_angle,0.0);
      transShoulderElbow.setRotation(Quat);
      transShoulderElbow.setOrigin(tf::Vector3(0, 0, 0));

      tf::Transform transElbowArm = transShoulderElbow.inverseTimes(transformArray.at(2));
      tf::Transform transElbowWist = transElbowArm.operator*(transformArray.at(3));

      // Quat.setRPY(PI,0,PI/2);
      // TransformOR.setRotation(Quat);
      // TransformOR.setOrigin(tf::Vector3(0, 0, 0));

      // transElbowWist.setOrigin(tf::Vector3(0, 0, 0));


      // TransformGR=TransformOR.operator*(transElbowWist);
      // TransformGR=TransformGR.operator*(TransformOR.inverse());

      double* T = new double[9];
      double* euler = new double[3];
      T[0]=transElbowWist.getBasis().getRow(0).getX();
      T[1]=transElbowWist.getBasis().getRow(0).getY();
      T[2]=transElbowWist.getBasis().getRow(0).getZ();


      T[3]=transElbowWist.getBasis().getRow(1).getX();
      T[4]=transElbowWist.getBasis().getRow(1).getY();
      T[5]=transElbowWist.getBasis().getRow(1).getZ();

      T[6]=transElbowWist.getBasis().getRow(2).getX();
      T[7]=transElbowWist.getBasis().getRow(2).getY();
      T[8]=transElbowWist.getBasis().getRow(2).getZ();

      matrix2XZYEuler(T, euler);

      joints["left_wrist_1_joint"] =  euler[2];
      joints["left_wrist_2_joint"] =  PI/2-euler[1]+PI/8;
      joints["left_wrist_3_joint"] = euler[0]+PI/18;//-0.8;

      transformArray.clear();

    }

    return 0;
  }

  /* int ik_newmap2(int side, std::vector<double> current_joint_values,  tfPercNeuronInterface& tfPNIntf, std::map<std::string, double>& joints, double* human_data){
    tf::Vector3 ArmPos;
    double* Joints1=new double[2];
    double* Joints2=new double[2];
    tf::Quaternion Quat;
    double d1=0;
    double d2=0;
    double d=0;
    tf::Transform TransformOG;
    tf::Transform TransformOR;
    tf::Transform TransformGR;

    if(side==1){

      std::vector<tf::Transform> transformArray;
      int success = 0;  

      success += pushIntoTransformArray("/Neck",  "/LeftBaseLink", &transformArray,
                                      &tfListener);

      if (success != 0) {
        return -1;
      }

      for (int i = 0; i < bodyJointsLeftArm.size() - 1; i++) {
        success += pushIntoTransformArray(
            bodyJointsLeftArm.at(i), bodyJointsLeftArm.at(i + 1),
            &transformArray, &tfListener);  // returns 0 for success -1 for failure
        if (success != 0) {
          //printf("\nNo tf transformation found from Neck to LeftArm");
          return -1;
        }
      }

      transLeftBaseLinkLeftArm = transformArray.at(0).inverse();
      transLeftBaseLinkLeftArm = transLeftBaseLinkLeftArm.operator*=(transformArray.at(1));
      transLeftBaseLinkLeftArm = transLeftBaseLinkLeftArm.operator*=(transformArray.at(2));

      //compute the left ur5's joint values and execute
      ArmPos=transLeftBaseLinkLeftArm.getOrigin();
      ArmPos.setX(-1*ArmPos.getX());
      ArmPos.setY(-1*ArmPos.getY());
      int a= ik_first_two_joints (ArmPos,Joints1,Joints2);

      for(int j=0;j<2;j++){
          d=Joints2[j]-current_joint_values.at(j);
          if(d>PI){
            Joints2[j]=Joints2[j]-2*PI;
          }
          if(d<-1*PI){
            Joints2[j]=Joints2[j]+2*PI;
          }
      }
      joints["left_shoulder_pan_joint"] = Joints2[0];
      joints["left_shoulder_lift_joint"] =  Joints2[1];

      tf::Vector3 origin_shoulder_arm = transformArray.at(2).getOrigin();
      tf::Matrix3x3 rot_shoulder_arm = transformArray.at(2).getBasis();

      // tf::Transform transArmForearm;
      tf::Vector3 origin_arm_forearm = transformArray.at(3).getOrigin();
     
      tf::Transform tf0;
      tf0.setBasis(rot_shoulder_arm);
      tf0.setOrigin(tf::Vector3(0.0,0.0,0.0));
      origin_arm_forearm = tf0*(origin_arm_forearm);
      double elbow_angle = -1*(origin_shoulder_arm.angle(origin_arm_forearm));
      joints["left_elbow_joint"] = elbow_angle;

      tf::Transform transShoulderElbow;
      Quat.setRPY(0.0,elbow_angle,0.0);
      transShoulderElbow.setRotation(Quat);
      transShoulderElbow.setOrigin(tf::Vector3(0, 0, 0));

      tf::Transform transElbowArm = transShoulderElbow.inverseTimes(transformArray.at(2));
      tf::Transform transElbowWist = transElbowArm.operator*(transformArray.at(3));

      Quat.setRPY(PI,0,PI/2);
      TransformOR.setRotation(Quat);
      TransformOR.setOrigin(tf::Vector3(0, 0, 0));

      transElbowWist.setOrigin(tf::Vector3(0, 0, 0));


      TransformGR=TransformOR.operator*(transElbowWist);
      TransformGR=TransformGR.operator*(TransformOR.inverse());

      double* T = new double[9];
      double* euler = new double[3];
      T[0]=TransformGR.getBasis().getRow(0).getX();
      T[1]=TransformGR.getBasis().getRow(0).getY();
      T[2]=TransformGR.getBasis().getRow(0).getZ();


      T[3]=TransformGR.getBasis().getRow(1).getX();
      T[4]=TransformGR.getBasis().getRow(1).getY();
      T[5]=TransformGR.getBasis().getRow(1).getZ();

      T[6]=TransformGR.getBasis().getRow(2).getX();
      T[7]=TransformGR.getBasis().getRow(2).getY();
      T[8]=TransformGR.getBasis().getRow(2).getZ();

      matrix2XZYEuler(T, euler);

      joints["left_wrist_1_joint"] =  euler[0];
      joints["left_wrist_2_joint"] =  PI/2+euler[1];
      joints["left_wrist_3_joint"] = euler[2];//-0.8;

      

      // record the human data
      tf::Transform RotNeckShoulder;
      tf::Transform RotShoulderArm;
      tf::Transform RotArmForearm;
      tf::Matrix3x3 wrist_rot;
      double roll,pitch,yaw;
      tf::Vector3 arm = transformArray.at(2).getOrigin();
      tf::Vector3 forearm = transformArray.at(3).getOrigin();
      tf::Vector3 endPos;

      RotNeckShoulder.setBasis(transformArray.at(1).getBasis());
      RotNeckShoulder.setOrigin(tf::Vector3(0,0,0));
      RotShoulderArm.setBasis(transformArray.at(2).getBasis());
      RotShoulderArm.setOrigin(tf::Vector3(0,0,0));
      RotArmForearm.setBasis(transformArray.at(3).getBasis());
      RotArmForearm.setOrigin(tf::Vector3(0,0,0));

      tf::Transform transLeftUrWorldLeftHandWorld;
      Quat.setRPY(PI/2, 0, -PI/2);
      transLeftUrWorldLeftHandWorld.setRotation(Quat);
      transLeftUrWorldLeftHandWorld.setOrigin(tf::Vector3(0, 0, 0));
      arm=transLeftUrWorldLeftHandWorld*(RotNeckShoulder*arm);
      forearm=transLeftUrWorldLeftHandWorld*(RotNeckShoulder*(RotShoulderArm*(forearm)));
      // hand=transLeftUrWorldLeftHandWorld*(RotNeckShoulder*(RotShoulderArm*(RotArmForearm*(hand))));
      wrist_rot=transLeftUrWorldLeftHandWorld.operator*(RotNeckShoulder.operator*(RotShoulderArm.operator*(RotArmForearm))).getBasis();
      wrist_rot.getRPY(roll,pitch,yaw);

      int human_data_count=0;
      human_data[human_data_count]=arm.getX(); human_data_count++;
      human_data[human_data_count]=arm.getY(); human_data_count++;
      human_data[human_data_count]=arm.getZ(); human_data_count++;
      human_data[human_data_count]=forearm.getX(); human_data_count++;
      human_data[human_data_count]=forearm.getY(); human_data_count++;
      human_data[human_data_count]=forearm.getZ(); human_data_count++;
      endPos=arm.operator+=(forearm);
      human_data[human_data_count]=endPos.getX(); human_data_count++;
      human_data[human_data_count]=endPos.getY(); human_data_count++;
      human_data[human_data_count]=endPos.getZ(); human_data_count++;
      human_data[human_data_count]=roll; human_data_count++;
      human_data[human_data_count]=pitch; human_data_count++;
      human_data[human_data_count]=yaw;


      transformArray.clear();

    }

    return 0;
  } */

};

struct Ur5Interface{
  //#define UR5_PARAMS
  const double d1 =  0.089159;
  const double a2 = -0.42500;
  const double a3 = -0.39225;
  const double d4 =  0.10915;
  const double d5 =  0.09465;
  const double d6 =  0.0823;

  void Zero2PI(double *Joints, int n){
    for (int i=0;i<n;i++){
      while (Joints[i]>2*PI){
        Joints[i]=Joints[i]-2*PI;
      }
      while (Joints[i]<=0){
        Joints[i]=Joints[i]+2*PI;
      }
    }
  }

  
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

  int findClosestIK(const double* T, double* q_sol, double q6_des, std::vector<double> current_joint_values){
    double q_sols[8*6];
    int a=inverse(T, q_sols, q6_des);
    if(a<4){
      printf("no enough solution can be found");
      return -1;
    }

    // for(int i=0;i<a;i++){
    //   printf("\n before normalization: %1.6f,%1.6f,%1.6f,%1.6f,%1.6f,%1.6f\n",
    //       q_sols[i*6+0],q_sols[i*6+1],q_sols[i*6+2],q_sols[i*6+3],q_sols[i*6+4],q_sols[i*6+5] );
    // }
    // printf("\na= %d\n",a);

    for (int i=0; i<a;i++){
      for (int j=0; j<6;j++){
        
          // while (q_sols[i*6+j]>PI){
          //   q_sols[i*6+j]=q_sols[i*6+j]-2*PI;
          // }
          // while (q_sols[i*6+j]<-PI){
          //   q_sols[i*6+j]=q_sols[i*6+j]+2*PI;
          // }
          while (q_sols[i*6+j]-current_joint_values.at(j)>PI){
              q_sols[i*6+j]=q_sols[i*6+j]-2*PI;
          }
          while (current_joint_values.at(j)-q_sols[i*6+j]>PI){
              q_sols[i*6+j]=q_sols[i*6+j]+2*PI;
          }     
      }
    }

    // for(int i=0;i<a;i++){
    //   printf("\n after normalization: %1.6f,%1.6f,%1.6f,%1.6f,%1.6f,%1.6f\n",
    //       q_sols[i*6+0],q_sols[i*6+1],q_sols[i*6+2],q_sols[i*6+3],q_sols[i*6+4],q_sols[i*6+5] );
    // }

    // printf("\n the current states are: %1.6f,%1.6f,%1.6f,%1.6f,%1.6f,%1.6f\n",
    //           current_joint_values.at(0),current_joint_values.at(1),current_joint_values.at(2),current_joint_values.at(3),current_joint_values.at(4),current_joint_values.at(5) );

    double d[a];
    for(int i=0;i<a;i++){
      d[i]=0;
    }

    for (int i=0; i<a;i++){
      for (int j=0; j<6;j++){
        d[i]=d[i]+(q_sols[i*6+j]-current_joint_values.at(j))*(q_sols[i*6+j]-current_joint_values.at(j));
      }
    }
    
     int index=0;
    for(int i=1;i<a;i++){
      if(d[i]<d[index]){
        index=i;
      }
    }
    //
    // if (a>4){
    //   index=5;
    // }
    
    // index=5;
    // printf("\n%d\n",index);
    // printf("\n the choosen joints are: %1.6f,%1.6f,%1.6f,%1.6f,%1.6f,%1.6f\n",
    //           q_sols[index*6+0],q_sols[index*6+1],q_sols[index*6+2],q_sols[index*6+3],q_sols[index*6+4],q_sols[index*6+5] );

    

    for (int i=0; i<6; i++){
      q_sol[i]=q_sols[index*6+i];
    }
    // for(int j=0; j<6; j++){
    //   if (q_sols[index*6+j]-current_joint_values.at(j)>PI){
    //       q_sol[j]=q_sols[index*6+j]-2*PI;
    //   }else if (current_joint_values.at(j)-q_sols[index*6+j]>PI){
    //       q_sol[j]=q_sols[index*6+j]+2*PI;
    //   }else{
    //     q_sol[j]=q_sols[index*6+j];
    //   }   
    // }

    // printf("\n the solution is :%1.6f,%1.6f,%1.6f,%1.6f,%1.6f,%1.6f\n",
    //           q_sol[0],q_sol[1],q_sol[2],q_sol[3],q_sol[4],q_sol[5] );

    // printf("ok solution can be found");
    return 0;
  }

  void array2tf(double* T, tf::Transform& transform){
    double T00=*T; T++; 
    double T01=*T; T++;
    double T02=*T; T++;
    double T03=*T; T++;
    double T10=*T; T++;
    double T11=*T; T++;
    double T12=*T; T++;
    double T13=*T; T++;
    double T20=*T; T++;
    double T21=*T; T++;
    double T22=*T; T++;
    double T23=*T; T++;
    double T30=*T; T++;
    double T31=*T; T++;
    double T32=*T; T++;
    double T33=*T;
    tf::Matrix3x3 rot(T00,T01,T02,T10,T11,T12,T20,T21,T22);
    tf::Vector3 ori(T03,T13,T23);
    transform.setBasis(rot);
    transform.setOrigin(ori);
  }
   
};


      
int main(int argc, char **argv)
{
  printf("\n Please control mode. \n");
  cin>>control_mode;
  if (control_mode!=1 && control_mode!=2  && control_mode!=3 && control_mode!=4){
    return 0;
  }else{
    cout<<"the choosen control mode is:"<<control_mode<<endl;
  }

  ros::init(argc, argv, "follow_arm_node");
  ros::NodeHandle node;
  ros::NodeHandle& nodeRef = node;
  int success; 

  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveGroup group1("left_manipulator");

  move_group_interface::MoveGroup::Plan plan1;

  std::map<std::string, double> joints1;
  std::map<std::string, double>& left_ur_joints = joints1;

  double framesPerSecPercNeuron = 10;
  ros::Rate rate(framesPerSecPercNeuron);
  

  // initial the left ur robot
  joints1["left_shoulder_pan_joint"] = PI/2;
  joints1["left_shoulder_lift_joint"] =  -PI;
  joints1["left_elbow_joint"] =  0;
  joints1["left_wrist_1_joint"] =  0;
  joints1["left_wrist_2_joint"] =  PI/2;
  joints1["left_wrist_3_joint"] = 0;
  group1.setJointValueTarget(joints1);
  if (!group1.plan(plan1))
  {
    ROS_FATAL("Unable to create motion plan.  Aborting.");
    exit(-1);
  }
  group1.asyncExecute(plan1);


  // Initialize Perception Neuron Interface
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  struct tfPercNeuronInterface tfPNIntf;  
  struct tfPercNeuronInterface& tfPNIntfRef = tfPNIntf;
  struct Ur5Interface ur;
  tf::Quaternion q;
  tfPNIntf.bodyJointsLeftArm = {"Neck","LeftShoulder", "LeftArm","LeftForeArm"};
  tfPNIntf.bodyJointsLeftHand = {"Neck","LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand"}; 

  // Initialize Transform from /world to /left_human_base_link and from /left_human_base_link to /left_human_base
  tfPNIntf.transWorldLeftHumanBaseLink = tf::Transform(
      tf::Quaternion(0, 0, 0, 1),
      tf::Vector3(0, -0.2, 0.5));  

  q.setRPY(PI/2, 0, -PI/2);
  tfPNIntf.transLeftHumanBaseLinkLeftHumanBase.setRotation(q);
  tfPNIntf.transLeftHumanBaseLinkLeftHumanBase.setOrigin(tf::Vector3(0, 0, 0));

  q.setRPY(0, PI/2, -PI/4);
  tfPNIntf.transWorldLeftBaseLink.setRotation(q);
  tfPNIntf.transWorldLeftBaseLink.setOrigin(tf::Vector3(0, 0, 0));

  // q.setRPY(0, -PI/2, 3*PI/4);
  // tfPNIntf.transWorldLeftBase.setRotation(q);
  // tfPNIntf.transWorldLeftBase.setOrigin(tf::Vector3(0, 0, 0));
  
  tfPNIntf.tfBroadcaster.sendTransform(tf::StampedTransform(tfPNIntf.transWorldLeftHumanBaseLink, ros::Time::now(), "/world", "/left_human_base_link")); 
  tfPNIntf.tfBroadcaster.sendTransform(tf::StampedTransform(tfPNIntf.transLeftHumanBaseLinkLeftHumanBase, ros::Time::now(), "/left_human_base_link", "/left_human_base"));

  
  // Start the loop to get tf position from Perception Neuron and publish it to the
  // robot / controller
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  printf("\nYou can start moving slowly now.");
  std::vector<double> current_joint_values;
  int side=0;
  ofstream human;
  human.open("human.txt");
  ofstream robot;
  robot.open("robot.txt");
  ofstream time;
  time.open("time.txt");

  int count_robot=0;
  int count_human=0;

  while (node.ok()) {
    //joint_controlled_mode
    if (control_mode==1){
       //left
        // side = 1;
        // current_joint_values= group1.getCurrentJointValues();
        // printf("\n getCurrentJointValues" );
        // printf("\n %1.6f,%1.6f\n",current_joint_values.at(0),current_joint_values.at(1) );
        // ur.ik_newmap2(side, current_joint_values,tfPNIntfRef,left_ur_joints);
        // joints1["left_shoulder_pan_joint"] = joints1["left_shoulder_pan_joint"]+PI/2;
        // group1.setJointValueTarget(joints1);
        // if (!group1.plan(plan1))
        // {
        //   ROS_FATAL("Unable to create motion plan.  Aborting.");
        //   exit(-1);
        // }
        // group1.asyncExecute(plan1);

    }

    // position_controlled_mode
    if (control_mode==2){

      //  record start time       
      double* T = new double[16];
      double* q_sol =new double[6];
      //left
      side = 1;
      success = -1;
      double* human_data = new double[12];

      tfPNIntf.tfBroadcaster.sendTransform(tf::StampedTransform(tfPNIntf.transWorldLeftHumanBaseLink, ros::Time::now(), "/world", "/left_human_base_link")); 
      tfPNIntf.tfBroadcaster.sendTransform(tf::StampedTransform(tfPNIntf.transLeftHumanBaseLinkLeftHumanBase, ros::Time::now(), "/left_human_base_link", "/left_human_base"));
     
      clock_t start = clock();
      while (success == -1) {
        //success = tfPNIntf.gettransBaseLinkHand3(side, human_data);
        success = tfPNIntf.gettransBaseLinkHand(side);
      }

      int i = 0;
      tf::Matrix3x3 RotGoal;
      RotGoal=tfPNIntf.transLeftGoalPosition.getBasis();
      tf::Matrix3x3 RotAdj;
      RotAdj.setValue(1,0,0,0,-1,0,0,0,-1);
      RotGoal=RotGoal.operator*=(RotAdj);

      T[0]=RotGoal.getRow(i).getX(),
      T[1]=RotGoal.getRow(i).getY(),
      T[2]=RotGoal.getRow(i).getZ(),
      T[3]=tfPNIntf.transLeftGoalPosition.getOrigin().getX();
      i = 1;
      T[4]=RotGoal.getRow(i).getX(),
      T[5]=RotGoal.getRow(i).getY(),
      T[6]=RotGoal.getRow(i).getZ(),
      T[7]=tfPNIntf.transLeftGoalPosition.getOrigin().getY();
      i = 2;
      T[8]=RotGoal.getRow(i).getX(),
      T[9]=RotGoal.getRow(i).getY(),
      T[10]=RotGoal.getRow(i).getZ(),
      T[11]=tfPNIntf.transLeftGoalPosition.getOrigin().getZ();
      T[12]=0;
      T[13]=0;
      T[14]=0;
      T[15]=1;
      current_joint_values= group1.getCurrentJointValues();
      // printf("\n getCurrentJointValues" );
      // printf("\n %1.6f,%1.6f\n",current_joint_values.at(0),current_joint_values.at(1) );
      success = -1;
      success = ur.findClosestIK( T,  q_sol, 0.0 , current_joint_values);
 
      
      if (success == -1)
      {
        continue;
      }
      // record end time
      // clock_t ends = clock();
      // time << (double)(ends-start)/CLOCKS_PER_SEC << "\n" << endl;
      clock_t ends = clock();
      time << (double)(ends-start)/CLOCKS_PER_SEC << " "<< "\n" << endl;

      joints1["left_shoulder_pan_joint"] = q_sol[0];
      joints1["left_shoulder_lift_joint"] =  q_sol[1];
      joints1["left_elbow_joint"] = q_sol[2];
      joints1["left_wrist_1_joint"] =  q_sol[3];
      joints1["left_wrist_2_joint"] =  q_sol[4];
      joints1["left_wrist_3_joint"] = q_sol[5];
      group1.setJointValueTarget(joints1);

      // clock_t start_1 = clock();
      if (!group1.plan(plan1))
      {
        ROS_FATAL("Unable to create motion plan.  Aborting.");
        exit(-1);
      } 
      // ends = clock();
      // time << (double)(ends-start_1)/CLOCKS_PER_SEC << " ";
      
       group1.asyncExecute(plan1); 
      // clock_t start_2 = clock();
      // group1.execute(plan1);
      // ends = clock();
      // time << (double)(ends-start_2)/CLOCKS_PER_SEC << " ";

      // ends = clock();
      // time << (double)(ends-start)/CLOCKS_PER_SEC << "\n" << endl;

      //
      count_human++; human << count_human<< " ";
      for (int j=0;j<12;j++){
        human << human_data[j] << " ";
      }
      human << "\n" << endl;

      double q[6];
      double T1[16];
      double T2[16];
      double T3[16];
      double T4[16];
      double T5[16];
      double T6[16];
      tf::Transform UR_F1;  
      tf::Transform& UR_F1Ref = UR_F1;
      tf::Transform UR_F2;  
      tf::Transform& UR_F2Ref = UR_F2;
      tf::Transform UR_F3;  
      tf::Transform& UR_F3Ref = UR_F3;
      tf::Transform UR_F4;  
      tf::Transform& UR_F4Ref = UR_F4;
      tf::Transform UR_F5;  
      tf::Transform& UR_F5Ref = UR_F5;
      tf::Transform UR_F6;  
      tf::Transform& UR_F6Ref = UR_F6;
      double roll; double pitch; double yaw;

      q[0]=joints1["left_shoulder_pan_joint"];
      q[1]=joints1["left_shoulder_lift_joint"];
      q[2]=joints1["left_elbow_joint"] ;
      q[3]=joints1["left_wrist_1_joint"] ;
      q[4]=joints1["left_wrist_2_joint"] ;
      q[5]=joints1["left_wrist_3_joint"] ;

      ur.forward_all(q,T1,T2,T3, T4, T5, T6);
      ur.array2tf(T1,UR_F1Ref);
      ur.array2tf(T2,UR_F2Ref);
      ur.array2tf(T3,UR_F3Ref);
      ur.array2tf(T6,UR_F6Ref);

      tf::Matrix3x3 left_ur_base_in_world(0,-0.7071067811865475,0.7071067811865475,0,-0.7071067811865475,-0.7071067811865475,1,0,0);
      tf::Transform World2LeftUrBase;
      World2LeftUrBase.setBasis(left_ur_base_in_world);
      World2LeftUrBase.setOrigin(tf::Vector3(0,0,0));

      tf::Vector3 ur_link2=UR_F2.getOrigin();
      ur_link2.operator-=(UR_F1.getOrigin());
      tf::Vector3 ur_link3=UR_F3.getOrigin();
      ur_link3.operator-=(UR_F2.getOrigin());
      tf::Vector3 ur_end=UR_F6.getOrigin();
      ur_link2=World2LeftUrBase*(ur_link2);
      ur_link3=World2LeftUrBase*(ur_link3);
      ur_end=World2LeftUrBase*(ur_end);
      UR_F6Ref=World2LeftUrBase.operator*(UR_F6Ref);
      tf::Matrix3x3 rot_ur_end_human_end(0,1,0,0,0,1,1,0,0);
      tf::Transform trans_ur_end_human_end;
      trans_ur_end_human_end.setBasis(rot_ur_end_human_end);
      trans_ur_end_human_end.setOrigin(tf::Vector3(0,0,0));
      UR_F6Ref=UR_F6Ref.operator*(trans_ur_end_human_end);
      tf::Matrix3x3 end_rot=UR_F6Ref.getBasis();        
      end_rot.getRPY(roll,pitch,yaw);
      count_robot++; robot << count_robot<<" ";
      robot << ur_link2.getX()<<" "<<ur_link2.getY()<<" "<<ur_link2.getZ()<<" ";
      robot << ur_link3.getX()<<" "<<ur_link3.getY()<<" "<<ur_link3.getZ()<<" ";
      robot << ur_end.getX()<<" "<<ur_end.getY()<<" "<<ur_end.getZ()<<" ";
      robot << roll <<" "<< pitch<<" "<< yaw <<" ";
      robot << q[0] <<" "<< q[1] <<" "<< q[2] <<" "<< q[3] <<" "<< q[4] <<" "<< q[5] <<" ";
      robot << "\n" << endl;


    }

    if (control_mode==3){
              
      side = 1;
      current_joint_values= group1.getCurrentJointValues();
      double* human_data = new double[12];

      tfPNIntf.tfBroadcaster.sendTransform(tf::StampedTransform(tfPNIntf.transWorldLeftHumanBaseLink, ros::Time::now(), "/world", "/left_human_base_link")); 
      tfPNIntf.tfBroadcaster.sendTransform(tf::StampedTransform(tfPNIntf.transLeftHumanBaseLinkLeftHumanBase, ros::Time::now(), "/left_human_base_link", "/left_human_base"));

      clock_t start = clock();
      tfPNIntf.ik_newmap(side, current_joint_values, left_ur_joints);
      //tfPNIntf.ik_newmap2(side, current_joint_values,tfPNIntfRef,left_ur_joints, human_data);

      // record end time
      clock_t ends = clock();
      time << (double)(ends-start)/CLOCKS_PER_SEC << " "<< "\n" << endl;

      // joints1["left_shoulder_pan_joint"] = joints1["left_shoulder_pan_joint"];
      group1.setJointValueTarget(joints1);

      // clock_t start_1 = clock();
       if (!group1.plan(plan1))
      {
        ROS_FATAL("Unable to create motion plan.  Aborting.");
        exit(-1);
      } 
      // ends = clock();
      // time << (double)(ends-start_1)/CLOCKS_PER_SEC << " ";

       group1.asyncExecute(plan1); 

      // clock_t start_2 = clock();
      // group1.execute(plan1);
      // ends = clock();
      // time << (double)(ends-start_2)/CLOCKS_PER_SEC << " ";

      // ends = clock();
      // time << (double)(ends-start)/CLOCKS_PER_SEC << "\n" << endl;

      //
      count_human++; human << count_human<< " ";
      for (int j=0;j<12;j++){
        human << human_data[j] << " ";
      }
      human << "\n" << endl;

      double q[6];
      double T1[16];
      double T2[16];
      double T3[16];
      double T4[16];
      double T5[16];
      double T6[16];
      tf::Transform UR_F1;  
      tf::Transform& UR_F1Ref = UR_F1;
      tf::Transform UR_F2;  
      tf::Transform& UR_F2Ref = UR_F2;
      tf::Transform UR_F3;  
      tf::Transform& UR_F3Ref = UR_F3;
      tf::Transform UR_F4;  
      tf::Transform& UR_F4Ref = UR_F4;
      tf::Transform UR_F5;  
      tf::Transform& UR_F5Ref = UR_F5;
      tf::Transform UR_F6;  
      tf::Transform& UR_F6Ref = UR_F6;
      double roll; double pitch; double yaw;
      q[0]=joints1["left_shoulder_pan_joint"];
      q[1]=joints1["left_shoulder_lift_joint"];
      q[2]=joints1["left_elbow_joint"] ;
      q[3]=joints1["left_wrist_1_joint"] ;
      q[4]=joints1["left_wrist_2_joint"] ;
      q[5]=joints1["left_wrist_3_joint"] ;

      ur.forward_all(q,T1,T2,T3, T4, T5, T6);
      ur.array2tf(T1,UR_F1Ref);
      ur.array2tf(T2,UR_F2Ref);
      ur.array2tf(T3,UR_F3Ref);
      ur.array2tf(T6,UR_F6Ref);

      tf::Matrix3x3 left_ur_base_in_world(0,-0.7071067811865475,0.7071067811865475,0,-0.7071067811865475,-0.7071067811865475,1,0,0);
      tf::Transform World2LeftUrBase;
      World2LeftUrBase.setBasis(left_ur_base_in_world);
      World2LeftUrBase.setOrigin(tf::Vector3(0,0,0));

      tf::Vector3 ur_link2=UR_F2.getOrigin();
      ur_link2.operator-=(UR_F1.getOrigin());
      tf::Vector3 ur_link3=UR_F3.getOrigin();
      ur_link3.operator-=(UR_F2.getOrigin());
      tf::Vector3 ur_end=UR_F6.getOrigin();
      ur_end.operator-=(UR_F1.getOrigin());
      ur_link2=World2LeftUrBase*(ur_link2);
      ur_link3=World2LeftUrBase*(ur_link3);
      ur_end=World2LeftUrBase*(ur_end);
      UR_F6Ref=World2LeftUrBase.operator*(UR_F6Ref);
      tf::Matrix3x3 rot_ur_end_human_end(0,1,0,0,0,1,1,0,0);
      tf::Transform trans_ur_end_human_end;
      trans_ur_end_human_end.setBasis(rot_ur_end_human_end);
      trans_ur_end_human_end.setOrigin(tf::Vector3(0,0,0));
      UR_F6Ref=UR_F6Ref.operator*(trans_ur_end_human_end);
      tf::Matrix3x3 end_rot=UR_F6Ref.getBasis();        
      end_rot.getRPY(roll,pitch,yaw);
      count_robot++; robot << count_robot<<" ";
      robot << ur_link2.getX()<<" "<<ur_link2.getY()<<" "<<ur_link2.getZ()<<" ";
      robot << ur_link3.getX()<<" "<<ur_link3.getY()<<" "<<ur_link3.getZ()<<" ";
      robot << ur_end.getX()<<" "<<ur_end.getY()<<" "<<ur_end.getZ()<<" ";
      robot << roll <<" "<< pitch<<" "<< yaw <<" ";
      robot << q[0] <<" "<< q[1] <<" "<< q[2] <<" "<< q[3] <<" "<< q[4] <<" "<< q[5] <<" ";
      robot << "\n" << endl;
    }

    if (control_mode==4){
              
      side = 1;
      current_joint_values= group1.getCurrentJointValues();
      double* human_data = new double[12];

      tfPNIntf.tfBroadcaster.sendTransform(tf::StampedTransform(tfPNIntf.transWorldLeftHumanBaseLink, ros::Time::now(), "/world", "/left_human_base_link")); 
      tfPNIntf.tfBroadcaster.sendTransform(tf::StampedTransform(tfPNIntf.transLeftHumanBaseLinkLeftHumanBase, ros::Time::now(), "/left_human_base_link", "/left_human_base"));

      clock_t start = clock();
      tfPNIntf.joint2joint_map(side, current_joint_values, left_ur_joints);

      // record end time
      clock_t ends = clock();
      time << (double)(ends-start)/CLOCKS_PER_SEC << " "<< "\n" << endl;

      group1.setJointValueTarget(joints1);

      // clock_t start_1 = clock();
       if (!group1.plan(plan1))
      {
        ROS_FATAL("Unable to create motion plan.  Aborting.");
        exit(-1);
      } 
      // ends = clock();
      // time << (double)(ends-start_1)/CLOCKS_PER_SEC << " ";

       group1.asyncExecute(plan1); 

      // clock_t start_2 = clock();
      // group1.execute(plan1);
      // ends = clock();
      // time << (double)(ends-start_2)/CLOCKS_PER_SEC << " ";

      // ends = clock();
      // time << (double)(ends-start)/CLOCKS_PER_SEC << "\n" << endl;

      //
      count_human++; human << count_human<< " ";
      for (int j=0;j<12;j++){
        human << human_data[j] << " ";
      }
      human << "\n" << endl;

      double q[6];
      double T1[16];
      double T2[16];
      double T3[16];
      double T4[16];
      double T5[16];
      double T6[16];
      tf::Transform UR_F1;  
      tf::Transform& UR_F1Ref = UR_F1;
      tf::Transform UR_F2;  
      tf::Transform& UR_F2Ref = UR_F2;
      tf::Transform UR_F3;  
      tf::Transform& UR_F3Ref = UR_F3;
      tf::Transform UR_F4;  
      tf::Transform& UR_F4Ref = UR_F4;
      tf::Transform UR_F5;  
      tf::Transform& UR_F5Ref = UR_F5;
      tf::Transform UR_F6;  
      tf::Transform& UR_F6Ref = UR_F6;
      double roll; double pitch; double yaw;
      q[0]=joints1["left_shoulder_pan_joint"];
      q[1]=joints1["left_shoulder_lift_joint"];
      q[2]=joints1["left_elbow_joint"] ;
      q[3]=joints1["left_wrist_1_joint"] ;
      q[4]=joints1["left_wrist_2_joint"] ;
      q[5]=joints1["left_wrist_3_joint"] ;

      ur.forward_all(q,T1,T2,T3, T4, T5, T6);
      ur.array2tf(T1,UR_F1Ref);
      ur.array2tf(T2,UR_F2Ref);
      ur.array2tf(T3,UR_F3Ref);
      ur.array2tf(T6,UR_F6Ref);

      tf::Matrix3x3 left_ur_base_in_world(0,-0.7071067811865475,0.7071067811865475,0,-0.7071067811865475,-0.7071067811865475,1,0,0);
      tf::Transform World2LeftUrBase;
      World2LeftUrBase.setBasis(left_ur_base_in_world);
      World2LeftUrBase.setOrigin(tf::Vector3(0,0,0));

      tf::Vector3 ur_link2=UR_F2.getOrigin();
      ur_link2.operator-=(UR_F1.getOrigin());
      tf::Vector3 ur_link3=UR_F3.getOrigin();
      ur_link3.operator-=(UR_F2.getOrigin());
      tf::Vector3 ur_end=UR_F6.getOrigin();
      ur_end.operator-=(UR_F1.getOrigin());
      ur_link2=World2LeftUrBase*(ur_link2);
      ur_link3=World2LeftUrBase*(ur_link3);
      ur_end=World2LeftUrBase*(ur_end);
      UR_F6Ref=World2LeftUrBase.operator*(UR_F6Ref);
      tf::Matrix3x3 rot_ur_end_human_end(0,1,0,0,0,1,1,0,0);
      tf::Transform trans_ur_end_human_end;
      trans_ur_end_human_end.setBasis(rot_ur_end_human_end);
      trans_ur_end_human_end.setOrigin(tf::Vector3(0,0,0));
      UR_F6Ref=UR_F6Ref.operator*(trans_ur_end_human_end);
      tf::Matrix3x3 end_rot=UR_F6Ref.getBasis();        
      end_rot.getRPY(roll,pitch,yaw);
      count_robot++; robot << count_robot<<" ";
      robot << ur_link2.getX()<<" "<<ur_link2.getY()<<" "<<ur_link2.getZ()<<" ";
      robot << ur_link3.getX()<<" "<<ur_link3.getY()<<" "<<ur_link3.getZ()<<" ";
      robot << ur_end.getX()<<" "<<ur_end.getY()<<" "<<ur_end.getZ()<<" ";
      robot << roll <<" "<< pitch<<" "<< yaw <<" ";
      robot << q[0] <<" "<< q[1] <<" "<< q[2] <<" "<< q[3] <<" "<< q[4] <<" "<< q[5] <<" ";
      robot << "\n" << endl;
    }

    rate.sleep();
  }

  ros::shutdown();
  group1.stop();
  human.close();
  robot.close();
  time.close();
  sleep(1.0); 
  return 0;
}
