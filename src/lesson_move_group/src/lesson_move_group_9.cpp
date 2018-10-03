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

struct tfPercNeuronInterface {
  
  std::vector<std::string> bodyJointsLeftArm; 
  std::vector<std::string> bodyJointsRightArm;
  std::vector<std::string> bodyJointsLeftHand; 
  std::vector<std::string> bodyJointsRightHand;  

  tf::Transform transNeckLeftBaseLink;
  tf::Transform transNeckRightBaseLink;
  tf::Transform transLeftBaseLinkLeftArm;
  tf::Transform transRightBaseLinkRightArm;
  tf::Transform transLeftBaseLinkLeftHand;
  tf::Transform transRightBaseLinkRightHand;
  tf::Transform transLeftGoalPosition;

  tf::TransformListener tfListener;  // tf listener to data from external tf broadcaster
  tf::TransformBroadcaster tfBroadcaster;  // broadcasts goal transform to tf to visualize it in rviz

  int side=1;//1 for left; 2 for right;
  // Get tf Transform / Vector from base_link of ur# to RightHand of PercNeuron
  // The UR# End Effector follows this transform
  // returns 0 for success, -1 for failure
  int gettransLeftBaseLinkLeftArm() {
    
    std::vector<tf::Transform> transformArray;
    int success = 0;  

    // Calculate transformation matrix from ur# base_link to RightHand
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    success += pushIntoTransformArray("/Neck",  "/LeftBaseLink", &transformArray,
                                      &tfListener);

    if (success != 0) {
      printf("\nNo tf Transformation found from left_base_link to Neck.");
      return -1;
    }
    
    for (int i = 0; i < bodyJointsLeftArm.size() - 1; i++) {
      success += pushIntoTransformArray(
          bodyJointsLeftArm.at(i), bodyJointsLeftArm.at(i + 1),
          &transformArray, &tfListener);  // returns 0 for success -1 for failure
      if (success != 0) {
        printf("\nNo tf transformation found from Neck to LeftArm");
        return -1;
      }
    }

    transLeftBaseLinkLeftArm = transformArray.at(0).inverse();
    transLeftBaseLinkLeftArm = transLeftBaseLinkLeftArm.operator*=(transformArray.at(1));
    transLeftBaseLinkLeftArm = transLeftBaseLinkLeftArm.operator*=(transformArray.at(2));
   
    tfBroadcaster.sendTransform(tf::StampedTransform(
        transLeftBaseLinkLeftArm, ros::Time::now(), "/LeftBaseLink", "/LeftBaseLinkToLeftArm"));
    printf("\nsuccess to broadcast tf Transformation from left_base_link to leftarm.");
    transformArray.clear();

    return 0;
  }

  int gettransRightBaseLinkRightArm() {
    
    std::vector<tf::Transform> transformArray;
    int success = 0;  

    // Calculate transformation matrix from ur# base_link to RightHand
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    success += pushIntoTransformArray("/Neck",  "/RightBaseLink", &transformArray,
                                      &tfListener);

    if (success != 0) {
      printf("\nNo tf Transformation found from right_base_link to Neck.");
      return -1;
    }
    
    for (int i = 0; i < bodyJointsRightArm.size() - 1; i++) {
      success += pushIntoTransformArray(
          bodyJointsRightArm.at(i), bodyJointsRightArm.at(i + 1),
          &transformArray, &tfListener);  // returns 0 for success -1 for failure
      if (success != 0) {
        printf("\nNo tf transformation found from Neck to RightArm");
        return -1;
      }
    }
    
    
    transRightBaseLinkRightArm = transformArray.at(0).inverse();
    transRightBaseLinkRightArm = transRightBaseLinkRightArm.operator*=(transformArray.at(1));
    transRightBaseLinkRightArm = transRightBaseLinkRightArm.operator*=(transformArray.at(2));
   
    tfBroadcaster.sendTransform(tf::StampedTransform(
        transRightBaseLinkRightArm, ros::Time::now(), "/RightBaseLink", "/RightBaseLinkToRightArm"));
    printf("\nsuccess to broadcast tf Transformation from right_base_link to rightarm.");
    transformArray.clear();
    return 0;
  }

  int gettransBaseLinkArm(int side) {
    
    std::vector<tf::Transform> transformArray;
    int success = 0;  
    std::vector<std::string> bodyJointsArm; 

    // Calculate transformation matrix from ur# base_link to RightHand
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    if (side==1){
      success += pushIntoTransformArray("/Neck",  "/LeftBaseLink", &transformArray,
                                      &tfListener);

      if (success != 0) {
        printf("\nNo tf Transformation found from left_base_link to Neck.");
        return -1;
      }

      for (int i = 0; i < bodyJointsLeftArm.size() - 1; i++) {
        success += pushIntoTransformArray(
            bodyJointsLeftArm.at(i), bodyJointsLeftArm.at(i + 1),
            &transformArray, &tfListener);  // returns 0 for success -1 for failure
        if (success != 0) {
          printf("\nNo tf transformation found from Neck to LeftArm");
          return -1;
        }
      }

      transLeftBaseLinkLeftArm = transformArray.at(0).inverse();
      transLeftBaseLinkLeftArm = transLeftBaseLinkLeftArm.operator*=(transformArray.at(1));
      transLeftBaseLinkLeftArm = transLeftBaseLinkLeftArm.operator*=(transformArray.at(2));
    
      tfBroadcaster.sendTransform(tf::StampedTransform(
          transLeftBaseLinkLeftArm, ros::Time::now(), "/LeftBaseLink", "/LeftBaseLinkToLeftArm"));
      printf("\nsuccess to broadcast tf Transformation from left_base_link to leftarm.");
      transformArray.clear();
    }
    
    if (side==2){
      success += pushIntoTransformArray("/Neck",  "/RightBaseLink", &transformArray,
                                      &tfListener);

      if (success != 0) {
        printf("\nNo tf Transformation found from right_base_link to Neck.");
        return -1;
      }

      for (int i = 0; i < bodyJointsRightArm.size() - 1; i++) {
        success += pushIntoTransformArray(
            bodyJointsRightArm.at(i), bodyJointsRightArm.at(i + 1),
            &transformArray, &tfListener);  // returns 0 for success -1 for failure
        if (success != 0) {
          printf("\nNo tf transformation found from Neck to RightArm");
          return -1;
        }
      }     

      transRightBaseLinkRightArm = transformArray.at(0).inverse();
      transRightBaseLinkRightArm = transRightBaseLinkRightArm.operator*=(transformArray.at(1));
      transRightBaseLinkRightArm = transRightBaseLinkRightArm.operator*=(transformArray.at(2));
    
      tfBroadcaster.sendTransform(tf::StampedTransform(
          transRightBaseLinkRightArm, ros::Time::now(), "/RightBaseLink", "/RightBaseLinkToRightArm"));
      printf("\nsuccess to broadcast tf Transformation from right_base_link to rightarm.");
      transformArray.clear();
    }
    
    return 0;
  }

  int gettransBaseLinkHand(int side) {
    
    std::vector<tf::Transform> transformArray;
    int success = 0;  
    std::vector<std::string> bodyJointsArm; 

    // Calculate transformation matrix from ur# base_link to RightHand
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    if (side==1){
      success += pushIntoTransformArray("/Neck",  "/LeftBaseLink", &transformArray,
                                      &tfListener);

      if (success != 0) {
        printf("\nNo tf Transformation found from left_base_link to Neck.");
        return -1;
      }

      for (int i = 0; i < bodyJointsLeftHand.size() - 1; i++) {
        success += pushIntoTransformArray(
            bodyJointsLeftHand.at(i), bodyJointsLeftHand.at(i + 1),
            &transformArray, &tfListener);  // returns 0 for success -1 for failure
        if (success != 0) {
          printf("\nNo tf transformation found from Neck to LeftHand");
          return -1;
        }
      }

      transLeftBaseLinkLeftHand = transformArray.at(0).inverse();
      for(int i = 1; i < transformArray.size(); i++){
        transLeftBaseLinkLeftHand = transLeftBaseLinkLeftHand.operator*=(transformArray.at(i));
      }
    
      tfBroadcaster.sendTransform(tf::StampedTransform(
          transLeftBaseLinkLeftHand, ros::Time::now(), "/LeftBaseLink", "/LeftBaseLinkToLeftHand"));
      printf("\nsuccess to broadcast tf Transformation from left_base_link to leftHand.");
      transformArray.clear();

      tf::Matrix3x3 Rot=transLeftBaseLinkLeftHand.getBasis();
      tf::Vector3 Pos= transLeftBaseLinkLeftHand.getOrigin();
      transLeftGoalPosition.setBasis(Rot);
      Pos.operator*=(1.4);
      transLeftGoalPosition.setOrigin(Pos);
      tfBroadcaster.sendTransform(tf::StampedTransform(
              transLeftGoalPosition, ros::Time::now(), "/left_base_link", "/LeftGoalPosition"));
      printf("\nsuccess to broadcast tf Transformation from left_base_link to LeftGoalPosition.");
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
  }

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

  int ik_newmap(int side, std::vector<double> current_joint_values,  tfPercNeuronInterface& tfPNIntf, std::map<std::string, double>& joints){
    tf::Vector3 ArmPos;
    double* Joints1=new double[2];
    double* Joints2=new double[2];
    tf::Quaternion Quat;
    double roll,pitch,yaw;
    double d1=0;
    double d2=0;
    double d=0;
    tf::TransformListener tfListener;
    tf::StampedTransform transform;

    if(side==1){
      int success = -1;
      while (success == -1) {
        tfPNIntf.tfBroadcaster.sendTransform(
            tf::StampedTransform(tfPNIntf.transNeckLeftBaseLink, ros::Time::now(),
                              "/Neck","/LeftBaseLink"));
        printf("\nSuccess publish tf from neck to LeftBaseLink.");
        success = tfPNIntf.gettransBaseLinkArm(side);
      }

      //compute the left ur5's joint values and execute
      ArmPos=tfPNIntf.transLeftBaseLinkLeftArm.getOrigin();
      ArmPos.setX(-1*ArmPos.getX());
      ArmPos.setY(-1*ArmPos.getY());
      printf("\nSuccess get tf from LeftBaseLink to LinkLeftArm.");
      int a= ik_first_two_joints (ArmPos,Joints1,Joints2);
      printf("\nSuccess get ik.");

      //choose a suitable set of joints
    //   d1=0;
    //   d2=0;
    //   for(int j=0;j<2;j++){
    //     d=Joints1[j]-current_joint_values.at(j);
    //     if(d>PI){
    //       Joints1[j]=Joints1[j]-2*PI;
    //       d=d-2*PI;
    //     }
    //     if(d<-1*PI){
    //       Joints1[j]=Joints1[j]+2*PI;
    //       d=d+2*PI;
    //     }
    //     d1=d1+d*d;
    //     d=Joints2[j]-current_joint_values.at(j);
    //     if(d>PI){
    //       Joints1[j]=Joints1[j]-2*PI;
    //       d=d-2*PI;
    //     }
    //     if(d<-1*PI){
    //       Joints1[j]=Joints1[j]+2*PI;
    //       d=d+2*PI;
    //     }
    //     d2=d2+d*d;
    // }
    // if(d1<=d2){
    //   joints["left_shoulder_pan_joint"] = Joints1[0];
    //   joints["left_shoulder_lift_joint"] =  Joints1[1];
    // }else{
    //   joints["left_shoulder_pan_joint"] = Joints2[0];
    //   joints["left_shoulder_lift_joint"] =  Joints2[1];
    // }

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
      printf("\n shoulder_pan_joint and shoulder_lift_joint" );

      try {
      tfListener.waitForTransform("/LeftShoulder", "/LeftArm", ros::Time(0),ros::Duration(3.0));
      tfListener.lookupTransform("/LeftShoulder", "/LeftArm", ros::Time(0),
                                  transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return -1;
      }
      printf("\nSuccess get tf from LeftShoulder to LeftArm.");
      Quat=transform.getRotation();
      tf::Matrix3x3(Quat).getRPY(roll,pitch,yaw);
      joints["left_elbow_joint"] = pitch;

      try {
      tfListener.waitForTransform("/LeftArm", "/LeftForeArm", ros::Time(0),ros::Duration(3.0));
      tfListener.lookupTransform("/LeftArm", "/LeftForeArm", ros::Time(0),
                                  transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return -1;
      }
      printf("\nSuccess get tf from LeftArm to LeftForeArm.");
      Quat=transform.getRotation();
      tf::Matrix3x3(Quat).getRPY(roll,pitch,yaw);
      joints["left_wrist_1_joint"] =  -1*PI/2+pitch;
      joints["left_wrist_2_joint"] =  PI/2-yaw;
      joints["left_wrist_3_joint"] = 0;
    }

    if(side==2){
      int success = -1;
      while (success == -1) {
        tfPNIntf.tfBroadcaster.sendTransform(
            tf::StampedTransform(tfPNIntf.transNeckLeftBaseLink, ros::Time::now(),
                              "/Neck","/RightBaseLink"));
        printf("\nSuccess publish tf from neck to RightBaseLink.");
        success = tfPNIntf.gettransBaseLinkArm(side);
      }

      //compute the Right ur5's joint values and execute
      ArmPos=tfPNIntf.transRightBaseLinkRightArm.getOrigin();
      ArmPos.setX(-1*ArmPos.getX());
      ArmPos.setY(-1*ArmPos.getY());
      printf("\nSuccess get tf from RightBaseLink to RightArm.");
      int a= ik_first_two_joints (ArmPos,Joints1,Joints2);
      printf("\nSuccess get ik.");

      //choose a suitable set of joints
      // d1=0;
      // d2=0;
      // for(int j=0;j<2;j++){
      //   d=Joints1[j]-current_joint_values.at(j);
      //   if(d>PI){
      //     d=d-2*PI;
      //   }
      //   if(d<-1*PI){
      //     d=d+2*PI;
      //   }
      //   d1=d1+d*d;
      //   d=Joints2[j]-current_joint_values.at(j);
      //   if(d>PI){
      //     d=d-2*PI;
      //   }
      //   if(d<-1*PI){
      //     d=d+2*PI;
      //   }
      //   d2=d2+d*d;
      // }
      // if(d1<=d2){
      //   joints["right_shoulder_pan_joint"] = Joints1[0];
      //   joints["right_shoulder_lift_joint"] =  Joints1[1];
      // }else{
      //   joints["right_shoulder_pan_joint"] = Joints1[0];
      //   joints["right_shoulder_lift_joint"] =  Joints1[1];
      // }


      for(int j=0;j<2;j++){
        d=Joints1[j]-current_joint_values.at(j);
        if(d>PI){
          d=d-2*PI;
        }
        if(d<-1*PI){
          d=d+2*PI;
        }
        d1=d1+d*d;
        d=Joints2[j]-current_joint_values.at(j);
        if(d>PI){
          d=d-2*PI;
        }
        if(d<-1*PI){
          d=d+2*PI;
        }
        d2=d2+d*d;
      }
      joints["right_shoulder_pan_joint"] = Joints1[0];
      joints["right_shoulder_lift_joint"] =  Joints1[1];
      printf("\n shoulder_pan_joint and shoulder_lift_joint" );

      try {
      tfListener.waitForTransform("/RightShoulder", "/RightArm", ros::Time(0),ros::Duration(3.0));
      tfListener.lookupTransform("/RightShoulder", "/RightArm", ros::Time(0),
                                  transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return -1;
      }
      printf("\nSuccess get tf from RightShoulder to RightArm.");
      Quat=transform.getRotation();
      tf::Matrix3x3(Quat).getRPY(roll,pitch,yaw);
      joints["right_elbow_joint"] = pitch;

      try {
      tfListener.waitForTransform("/RightArm", "/RightForeArm", ros::Time(0),ros::Duration(3.0));
      tfListener.lookupTransform("/RightArm", "/RightForeArm", ros::Time(0),
                                  transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return -1;
      }
      printf("\nSuccess get tf from RightArm to RightForeArm.");
      Quat=transform.getRotation();
      tf::Matrix3x3(Quat).getRPY(roll,pitch,yaw);
      joints["right_wrist_1_joint"] =  -1*PI/2+pitch;
      joints["right_wrist_2_joint"] =  -1*PI/2-yaw;

      joints["right_wrist_3_joint"] = 0;
    }

    return 0;
  }

  int ik_newmap2(int side, std::vector<double> current_joint_values,  tfPercNeuronInterface& tfPNIntf, std::map<std::string, double>& joints){
    tf::Vector3 ArmPos;
    double* Joints1=new double[2];
    double* Joints2=new double[2];
    tf::Quaternion Quat;
    double roll,pitch,yaw;
    double d1=0;
    double d2=0;
    double d=0;
    tf::TransformListener tfListener;
    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform transform;
    tf::Transform TransformOG;
      tf::Transform TransformOR;
      tf::Transform TransformGR;

    if(side==1){
      int success = -1;
      while (success == -1) {
        tfPNIntf.tfBroadcaster.sendTransform(
            tf::StampedTransform(tfPNIntf.transNeckLeftBaseLink, ros::Time::now(),
                              "/Neck","/LeftBaseLink"));
        printf("\nSuccess publish tf from neck to LeftBaseLink.");
        success = tfPNIntf.gettransBaseLinkArm(side);
      }

      //compute the left ur5's joint values and execute
      ArmPos=tfPNIntf.transLeftBaseLinkLeftArm.getOrigin();
      ArmPos.setX(-1*ArmPos.getX());
      ArmPos.setY(-1*ArmPos.getY());
      printf("\nSuccess get tf from LeftBaseLink to LinkLeftArm.");
      int a= ik_first_two_joints (ArmPos,Joints1,Joints2);
      printf("\nSuccess get ik.");

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
      printf("\n shoulder_pan_joint and shoulder_lift_joint" );

      tf::Transform transShoulderArm;
      try {
      tfListener.waitForTransform("/LeftShoulder", "/LeftArm", ros::Time(0),ros::Duration(3.0));
      tfListener.lookupTransform("/LeftShoulder", "/LeftArm", ros::Time(0),
                                  transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return -1;
      }
      printf("\nSuccess get tf from LeftShoulder to LeftArm.");
      tf::Vector3 origin_shoulder_arm = transform.getOrigin();
      tf::Matrix3x3 rot_shoulder_arm = transform.getBasis();
      transShoulderArm.setBasis(rot_shoulder_arm);
      transShoulderArm.setOrigin(origin_shoulder_arm);

      tf::Transform transArmForeArm;
      try {
      tfListener.waitForTransform("/LeftArm", "/LeftForeArm", ros::Time(0),ros::Duration(3.0));
      tfListener.lookupTransform("/LeftArm", "/LeftForeArm", ros::Time(0),
                                  transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return -1;
      }
      tf::Vector3 origin_arm_forearm = transform.getOrigin();
      origin_shoulder_arm = transShoulderArm*(origin_shoulder_arm);
      joints["left_elbow_joint"] = -1*(origin_shoulder_arm.angle(origin_arm_forearm));

      tf::Transform transShoulderElbow;
      Quat.setRPY(0.0,-1*(origin_shoulder_arm.angle(origin_arm_forearm)),0.0);
      transShoulderElbow.setRotation(Quat);
      transShoulderElbow.setOrigin(tf::Vector3(0, 0, 0));
      tfBroadcaster.sendTransform(tf::StampedTransform(transShoulderElbow, ros::Time::now(),
                            "/LeftShoulder","/LeftShoulderElbow"));

      tf::Transform transElbowArm = transShoulderElbow.inverseTimes(transShoulderArm);
      tfBroadcaster.sendTransform(tf::StampedTransform(transElbowArm, ros::Time::now(),
                            "/LeftShoulderElbow","/LeftElbowArm"));
      Quat=transElbowArm.getRotation();
      tf::Matrix3x3(Quat).getRPY(roll,pitch,yaw);
      double ellow_roll=roll;
      printf("\n the elbow roll is %f %f %f\n", roll, pitch,yaw);

      try {
      tfListener.waitForTransform("/LeftArm", "/LeftForeArm", ros::Time(0),ros::Duration(3.0));
      tfListener.lookupTransform("/LeftArm", "/LeftForeArm", ros::Time(0),
                                  transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return -1;
      }
      Quat=transform.getRotation();
      tf::Matrix3x3(Quat).getRPY(roll,pitch,yaw);
      printf("\n the wrist roll is %f %f %f\n", roll, pitch,yaw);
      roll=roll+ellow_roll;     
      Quat.setRPY(roll,pitch,yaw);
      TransformOG.setRotation(Quat);
      TransformOG.setOrigin(tf::Vector3(0, 0, 0));
      tfBroadcaster.sendTransform(tf::StampedTransform(TransformOG, ros::Time::now(),
                            "/LeftShoulderElbow","/LeftElbowWrist"));


      Quat.setRPY(-PI/2, -PI/2, 0);
      TransformOR.setRotation(Quat);
      TransformOR.setOrigin(tf::Vector3(0, 0, 0));
      tfBroadcaster.sendTransform(tf::StampedTransform(TransformOR, ros::Time::now(),
                            "/LeftShoulderElbow","/LeftElbowNewbase"));

      TransformGR=TransformOR.inverseTimes(TransformOG);
      tfBroadcaster.sendTransform(tf::StampedTransform(TransformGR, ros::Time::now(),
                            "/LeftElbowNewbase","/LeftWrist"));

      TransformGR=TransformGR.operator*(TransformOR);
      Quat=TransformGR.getRotation();
      tf::Matrix3x3(Quat).getRPY(roll,pitch,yaw);

      // joints["left_wrist_1_joint"] =  yaw;
      // joints["left_wrist_2_joint"] =  roll;
      // joints["left_wrist_3_joint"] = -1*pitch;

      joints["left_wrist_1_joint"] =  -PI/2+yaw;
      joints["left_wrist_2_joint"] =  PI/2+pitch;
      joints["left_wrist_3_joint"] = roll;

    }

    if(side==2){
      int success = -1;
      while (success == -1) {
        tfPNIntf.tfBroadcaster.sendTransform(
            tf::StampedTransform(tfPNIntf.transNeckLeftBaseLink, ros::Time::now(),
                              "/Neck","/RightBaseLink"));
        printf("\nSuccess publish tf from neck to RightBaseLink.");
        success = tfPNIntf.gettransBaseLinkArm(side);
      }

      //compute the Right ur5's joint values and execute
      ArmPos=tfPNIntf.transRightBaseLinkRightArm.getOrigin();
      ArmPos.setX(-1*ArmPos.getX());
      ArmPos.setY(-1*ArmPos.getY());
      printf("\nSuccess get tf from RightBaseLink to RightArm.");
      int a= ik_first_two_joints (ArmPos,Joints1,Joints2);
      printf("\nSuccess get ik.");

      //choose a suitable set of joints
      // d1=0;
      // d2=0;
      // for(int j=0;j<2;j++){
      //   d=Joints1[j]-current_joint_values.at(j);
      //   if(d>PI){
      //     d=d-2*PI;
      //   }
      //   if(d<-1*PI){
      //     d=d+2*PI;
      //   }
      //   d1=d1+d*d;
      //   d=Joints2[j]-current_joint_values.at(j);
      //   if(d>PI){
      //     d=d-2*PI;
      //   }
      //   if(d<-1*PI){
      //     d=d+2*PI;
      //   }
      //   d2=d2+d*d;
      // }
      // if(d1<=d2){
      //   joints["right_shoulder_pan_joint"] = Joints1[0];
      //   joints["right_shoulder_lift_joint"] =  Joints1[1];
      // }else{
      //   joints["right_shoulder_pan_joint"] = Joints1[0];
      //   joints["right_shoulder_lift_joint"] =  Joints1[1];
      // }


      for(int j=0;j<2;j++){
        d=Joints1[j]-current_joint_values.at(j);
        if(d>PI){
          d=d-2*PI;
        }
        if(d<-1*PI){
          d=d+2*PI;
        }
        d1=d1+d*d;
        d=Joints2[j]-current_joint_values.at(j);
        if(d>PI){
          d=d-2*PI;
        }
        if(d<-1*PI){
          d=d+2*PI;
        }
        d2=d2+d*d;
      }
      joints["right_shoulder_pan_joint"] = Joints1[0];
      joints["right_shoulder_lift_joint"] =  Joints1[1];
      printf("\n shoulder_pan_joint and shoulder_lift_joint" );

      try {
      tfListener.waitForTransform("/RightShoulder", "/RightArm", ros::Time(0),ros::Duration(3.0));
      tfListener.lookupTransform("/RightShoulder", "/RightArm", ros::Time(0),
                                  transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return -1;
      }
      printf("\nSuccess get tf from RightShoulder to RightArm.");
      Quat=transform.getRotation();
      tf::Matrix3x3(Quat).getRPY(roll,pitch,yaw);
      joints["right_elbow_joint"] = pitch;

      try {
      tfListener.waitForTransform("/RightArm", "/RightForeArm", ros::Time(0),ros::Duration(3.0));
      tfListener.lookupTransform("/RightArm", "/RightForeArm", ros::Time(0),
                                  transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return -1;
      }
      printf("\nSuccess get tf from RightArm to RightForeArm.");
      Quat=transform.getRotation();
      tf::Matrix3x3(Quat).getRPY(roll,pitch,yaw);
      joints["right_wrist_1_joint"] =  -1*PI/2+pitch;
      joints["right_wrist_2_joint"] =  -1*PI/2-yaw;

      joints["right_wrist_3_joint"] = 0;
    }

    return 0;
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

      // while (Joints1[1]>2*PI){
      //   Joints1[1]=Joints1[1]-2*PI;
      // }
      // while (Joints1[1]<=0){
      //   Joints1[1]=Joints1[1]+2*PI;
      // }
      // while (Joints2[1]>2*PI){
      //   Joints2[1]=Joints2[1]-2*PI;
      // }
      // while (Joints2[1]<=0){
      //   Joints2[1]=Joints2[1]+2*PI;
      // }
    }
    return 0;
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
    if(a<0){
      printf("no enough solution can be found");
      return -1;
    }

    for(int i=0;i<a;i++){
      printf("\n before normalization: %1.6f,%1.6f,%1.6f,%1.6f,%1.6f,%1.6f\n",
          q_sols[i*6+0],q_sols[i*6+1],q_sols[i*6+2],q_sols[i*6+3],q_sols[i*6+4],q_sols[i*6+5] );
    }
    printf("\n%d\n",a);

    for (int i=0; i<a;i++){
      for (int j=0; j<6;j++){
        while (q_sols[i*6+j]>PI){
          q_sols[i*6+j]=q_sols[i*6+j]-2*PI;
        }
        if (q_sols[i*6+j]-current_joint_values.at(j)>PI){
            q_sols[i*6+j]=q_sols[i*6+j]-2*PI;
        }else if (current_joint_values.at(j)-q_sols[i*6+j]>PI){
            q_sols[i*6+j]=q_sols[i*6+j]+2*PI;
        }
      }
    }

    for(int i=0;i<a;i++){
      printf("\n after normalization: %1.6f,%1.6f,%1.6f,%1.6f,%1.6f,%1.6f\n",
          q_sols[i*6+0],q_sols[i*6+1],q_sols[i*6+2],q_sols[i*6+3],q_sols[i*6+4],q_sols[i*6+5] );
    }

    printf("\n the current states are: %1.6f,%1.6f,%1.6f,%1.6f,%1.6f,%1.6f\n",
              current_joint_values.at(0),current_joint_values.at(1),current_joint_values.at(2),current_joint_values.at(3),current_joint_values.at(4),current_joint_values.at(5) );

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
    printf("\n%d\n",index);
    printf("\n the choosen joints are: %1.6f,%1.6f,%1.6f,%1.6f,%1.6f,%1.6f\n",
              q_sols[index*6+0],q_sols[index*6+1],q_sols[index*6+2],q_sols[index*6+3],q_sols[index*6+4],q_sols[index*6+5] );

    

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
    //           q_sol[index*6+0],q_sol[index*6+1],q_sol[index*6+2],q_sol[index*6+3],q_sol[index*6+4],q_sol[index*6+5] );

    printf("ok solution can be found");
    return 0;
  }
  
};


      
int main(int argc, char **argv)
{
  printf("\n Please control mode. \n");
  cin>>control_mode;
  if (control_mode!=1 && control_mode!=2){
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
  moveit::planning_interface::MoveGroup group2("right_manipulator");
  move_group_interface::MoveGroup::Plan plan1;
  move_group_interface::MoveGroup::Plan plan2;
  std::map<std::string, double> joints1;
  std::map<std::string, double> joints2;
  std::map<std::string, double>& left_ur_joints = joints1;
  std::map<std::string, double>& right_ur_joints = joints2;

  double framesPerSecPercNeuron = 20;
  ros::Rate rate(framesPerSecPercNeuron);
  

  // initial the left ur robot
  joints1["left_shoulder_pan_joint"] = 0;
  joints1["left_shoulder_lift_joint"] =  PI;
  joints1["left_elbow_joint"] =  -1*PI/4;
  joints1["left_wrist_1_joint"] =  -1*PI/2;
  joints1["left_wrist_2_joint"] =  PI/2;
  joints1["left_wrist_3_joint"] = 0;
  group1.setJointValueTarget(joints1);
  if (!group1.plan(plan1))
  {
    ROS_FATAL("Unable to create motion plan.  Aborting.");
    exit(-1);
  }
  group1.asyncExecute(plan1);

 // initial the right ur robot
  joints2["right_shoulder_pan_joint"] = 0;
  joints2["right_shoulder_lift_joint"] =  0;
  joints2["right_elbow_joint"] =  PI/4;
  joints2["right_wrist_1_joint"] =  -1*PI/2;
  joints2["right_wrist_2_joint"] =  -1*PI/2;
  joints2["right_wrist_3_joint"] = 0;
  group2.setJointValueTarget(joints2);
  if (!group2.plan(plan2))
  {
    ROS_FATAL("Unable to create motion plan.  Aborting.");
    exit(-1);
  }
  group2.asyncExecute(plan2);


  // printf("\n Please control mode. \n", simRobotTopic.c_str());
  // char response;
  // cin >> response;
  // printf("\nresponse: %c", response);
  // if (response == '1' ) {
  //   pubIntf.jointPosPublisher = node.advertise<sensor_msgs::JointState>(
  //     simRobotTopic,1000);
  // }
  // else {
  //     printf("\nTerminating node. Restart in real robot mode.");
  //     ros::shutdown();
  // }

  // Initialize Perception Neuron Interface
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  struct tfPercNeuronInterface tfPNIntf;  
  struct tfPercNeuronInterface& tfPNIntfRef = tfPNIntf;
  struct Ur5Interface ur;
  tfPNIntf.bodyJointsLeftArm = {"Neck","LeftShoulder", "LeftArm"};
  tfPNIntf.bodyJointsRightArm = {"Neck","RightShoulder", "RightArm"};
  tfPNIntf.bodyJointsLeftHand = {"Neck","LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand"}; 
  tfPNIntf.bodyJointsRightHand = {"Neck","RightShoulder", "RightArm", "RightForeArm", "RightHand"}; 

  // Initialize Transform from base_link to LeftArm directly
  tfPNIntf.transLeftBaseLinkLeftArm = tf::Transform(
      tf::Quaternion(0, 0, 0, 1),
      tf::Vector3(0, 0, 0));  

  tfPNIntf.tfBroadcaster.sendTransform(tf::StampedTransform(
        tfPNIntf.transLeftBaseLinkLeftArm, ros::Time::now(), "/LeftBaseLink", "/LeftBaseLinkToLeftArm"));

 // Initialize Transform from base_link to RightArm directly
  tfPNIntf.transRightBaseLinkRightArm = tf::Transform(
      tf::Quaternion(0, 0, 0, 1),
      tf::Vector3(0, 0, 0));  

  tfPNIntf.tfBroadcaster.sendTransform(tf::StampedTransform(
        tfPNIntf.transRightBaseLinkRightArm, ros::Time::now(), "/RightBaseLink", "/RightBaseLinkToLeftArm"));

 //Comput the transform from neck to LeftBaseLink;
  tf::Transform transLeftUrWorldLeftBaseLink; 
  tf::Transform transLeftHandWorldLeftUrWorld; 
  tf::Transform transNeckLeftHandWorld; 
  tf::Transform transRightUrWorldRightBaseLink; 
  tf::Transform transRightHandWorldRightUrWorld; 
  tf::Transform transNeckRightHandWorld;
  tf::TransformListener tfListener;
  tf::StampedTransform transform;
  tf::Quaternion q;

  q.setRPY(-PI/2, PI/2, 0);
  transLeftHandWorldLeftUrWorld.setRotation(q);
  transLeftHandWorldLeftUrWorld.setOrigin(tf::Vector3(0, 0, 0));

  q.setRPY(PI/2, 0, PI/4);
  transLeftUrWorldLeftBaseLink.setRotation(q);
  transLeftUrWorldLeftBaseLink.setOrigin(tf::Vector3(0, 0, 0));

  try {
    tfListener.waitForTransform("/Neck", "/LeftShoulder", ros::Time(0),ros::Duration(3.0));
    tfListener.lookupTransform("/Neck", "/LeftShoulder", ros::Time(0),
                                transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
    return -1;
  }
  
  q.setRPY(0, 0, 0);
  transNeckLeftHandWorld.setRotation(q);
  transNeckLeftHandWorld.setOrigin(transform.getOrigin()); 
  
  tfPNIntf.transNeckLeftBaseLink=transNeckLeftHandWorld;
  tfPNIntf.transNeckLeftBaseLink=tfPNIntf.transNeckLeftBaseLink.operator*=(transLeftHandWorldLeftUrWorld);
  tfPNIntf.transNeckLeftBaseLink=tfPNIntf.transNeckLeftBaseLink.operator*=(transLeftUrWorldLeftBaseLink);
                        
 //Comput the transform from neck to RightBaseLink;
  q.setRPY(-PI/2, PI/2, 0);
  transRightHandWorldRightUrWorld.setRotation(q);
  transRightHandWorldRightUrWorld.setOrigin(tf::Vector3(0, 0, 0));

  q.setRPY(PI/2, 0, 3*PI/4);
  transRightUrWorldRightBaseLink.setRotation(q);
  transRightUrWorldRightBaseLink.setOrigin(tf::Vector3(0, 0, 0));

  try {
    tfListener.waitForTransform("/Neck", "/RightShoulder", ros::Time(0),ros::Duration(3.0));
    tfListener.lookupTransform("/Neck", "/RightShoulder", ros::Time(0),
                                transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
    return -1;
  }
  
  q.setRPY(0, 0, 0);
  transNeckRightHandWorld.setRotation(q);
  transNeckRightHandWorld.setOrigin(transform.getOrigin()); 
  

  tfPNIntf.transNeckRightBaseLink=transNeckLeftHandWorld;
  tfPNIntf.transNeckRightBaseLink=tfPNIntf.transNeckRightBaseLink.operator*=(transRightHandWorldRightUrWorld);
  tfPNIntf.transNeckRightBaseLink=tfPNIntf.transNeckRightBaseLink.operator*=(transRightUrWorldRightBaseLink);

  // Start the loop to get tf position from Perception Neuron and publish it to the
  // robot / controller
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  printf("\nYou can start moving slowly now.");
  std::vector<double> current_joint_values;
  int side=0;

  while (node.ok()) {
    //joint_controlled_mode
    if (control_mode==1){
       //left
        side = 1;
        current_joint_values= group1.getCurrentJointValues();
        printf("\n getCurrentJointValues" );
        printf("\n %1.6f,%1.6f\n",current_joint_values.at(0),current_joint_values.at(1) );
        ur.ik_newmap2(side, current_joint_values,tfPNIntfRef,left_ur_joints);
        group1.setJointValueTarget(joints1);
        if (!group1.plan(plan1))
        {
          ROS_FATAL("Unable to create motion plan.  Aborting.");
          exit(-1);
        }
        group1.asyncExecute(plan1);

        //right
        // side = 2;
        // current_joint_values= group2.getCurrentJointValues();
        // printf("\n getCurrentJointValues" );
        // printf("\n %1.6f,%1.6f\n",current_joint_values.at(0),current_joint_values.at(1) );
        // ur.ik_newmap(side, current_joint_values, tfPNIntfRef, right_ur_joints);
        // group2.setJointValueTarget(joints2);
        // if (!group2.plan(plan2))
        // {
        //   ROS_FATAL("Unable to create motion plan.  Aborting.");
        //   exit(-1);
        // }
        // group2.asyncExecute(plan2);
    }

    // position_controlled_mode
    if (control_mode==2){
      double* T = new double[16];
      double* q_sol =new double[6];
      //left
      side = 1;
      success = -1;
      while (success == -1) {
        tfPNIntf.tfBroadcaster.sendTransform(
            tf::StampedTransform(tfPNIntf.transNeckLeftBaseLink, ros::Time::now(),
                              "/Neck","/LeftBaseLink"));
        printf("\nSuccess publish tf from neck to LeftBaseLink.");
        success = tfPNIntf.gettransBaseLinkHand(side);
      }
      int i = 0;
      // int scale=-1;
      T[0]=tfPNIntf.transLeftGoalPosition.getBasis().getRow(i).getX(),
      T[1]=tfPNIntf.transLeftGoalPosition.getBasis().getRow(i).getY(),
      T[2]=tfPNIntf.transLeftGoalPosition.getBasis().getRow(i).getZ(),
      T[3]=tfPNIntf.transLeftGoalPosition.getOrigin().getX();
      // T[3]=T[3]*scale;
      i = 1;
      T[4]=tfPNIntf.transLeftGoalPosition.getBasis().getRow(i).getX(),
      T[5]=tfPNIntf.transLeftGoalPosition.getBasis().getRow(i).getY(),
      T[6]=tfPNIntf.transLeftGoalPosition.getBasis().getRow(i).getZ(),
      T[7]=tfPNIntf.transLeftGoalPosition.getOrigin().getY();
      // T[7]=T[7]*scale;
      i = 2;
      T[8]=tfPNIntf.transLeftGoalPosition.getBasis().getRow(i).getX(),
      T[9]=tfPNIntf.transLeftGoalPosition.getBasis().getRow(i).getY(),
      T[10]=tfPNIntf.transLeftGoalPosition.getBasis().getRow(i).getZ(),
      T[11]=tfPNIntf.transLeftGoalPosition.getOrigin().getZ();
      // T[11]=T[11]*scale;
      T[12]=0;
      T[13]=0;
      T[14]=0;
      T[15]=1;
      current_joint_values= group1.getCurrentJointValues();
      printf("\n getCurrentJointValues" );
      printf("\n %1.6f,%1.6f\n",current_joint_values.at(0),current_joint_values.at(1) );
      ur.findClosestIK( T,  q_sol, 0.0 , current_joint_values);
      joints1["left_shoulder_pan_joint"] = q_sol[0];
      joints1["left_shoulder_lift_joint"] =  q_sol[1];
      joints1["left_elbow_joint"] = q_sol[2];
      joints1["left_wrist_1_joint"] =  q_sol[3];
      joints1["left_wrist_2_joint"] =  q_sol[4];
      joints1["left_wrist_3_joint"] = q_sol[5];
      group1.setJointValueTarget(joints1);
      if (!group1.plan(plan1))
      {
        ROS_FATAL("Unable to create motion plan.  Aborting.");
        exit(-1);
      }
      group1.asyncExecute(plan1);

      //right
      // side = 2;
      // success = -1;
      // while (success == -1) {
      //   tfPNIntf.tfBroadcaster.sendTransform(
      //       tf::StampedTransform(tfPNIntf.transNeckRightBaseLink, ros::Time::now(),
      //                         "/Neck","/RightBaseLink"));
      //   printf("\nSuccess publish tf from neck to RightBaseLink.");
      //   success = tfPNIntf.gettransBaseLinkHand(side);
      // }
      // i = 0;
      // T[0]=tfPNIntf.transRightBaseLinkRightHand.getBasis().getRow(i).getX(),
      // T[1]=tfPNIntf.transRightBaseLinkRightHand.getBasis().getRow(i).getY(),
      // T[2]=tfPNIntf.transRightBaseLinkRightHand.getBasis().getRow(i).getZ(),
      // T[3]=tfPNIntf.transRightBaseLinkRightHand.getOrigin().getX();
      // i = 1;
      // T[4]=tfPNIntf.transRightBaseLinkRightHand.getBasis().getRow(i).getX(),
      // T[5]=tfPNIntf.transRightBaseLinkRightHand.getBasis().getRow(i).getY(),
      // T[6]=tfPNIntf.transRightBaseLinkRightHand.getBasis().getRow(i).getZ(),
      // T[7]=tfPNIntf.transRightBaseLinkRightHand.getOrigin().getY();
      // i = 2;
      // T[8]=tfPNIntf.transRightBaseLinkRightHand.getBasis().getRow(i).getX(),
      // T[9]=tfPNIntf.transRightBaseLinkRightHand.getBasis().getRow(i).getY(),
      // T[10]=tfPNIntf.transRightBaseLinkRightHand.getBasis().getRow(i).getZ(),
      // T[11]=tfPNIntf.transRightBaseLinkRightHand.getOrigin().getZ();
      // T[12]=0;
      // T[13]=0;
      // T[14]=0;
      // T[15]=1;
      // current_joint_values= group2.getCurrentJointValues();
      // printf("\n getCurrentJointValues" );
      // printf("\n %1.6f,%1.6f\n",current_joint_values.at(0),current_joint_values.at(1) );
      // ur.findClosestIK( T,  q_sol, 0.0 , current_joint_values);
      // joints2["right_shoulder_pan_joint"] = q_sol[0];
      // joints2["right_shoulder_lift_joint"] =  q_sol[1];
      // joints2["right_elbow_joint"] = q_sol[2];
      // joints2["right_wrist_1_joint"] =  q_sol[3];
      // joints2["right_wrist_2_joint"] =  q_sol[4];
      // joints2["right_wrist_3_joint"] = q_sol[5];
      // group2.setJointValueTarget(joints2);
      // if (!group2.plan(plan2))
      // {
      //   ROS_FATAL("Unable to create motion plan.  Aborting.");
      //   exit(-1);
      // }
      // group2.asyncExecute(plan2);

    }

    rate.sleep();
  }

  ros::shutdown();
  group1.stop();
  group2.stop();
  sleep(1.0); 
  return 0;
}
