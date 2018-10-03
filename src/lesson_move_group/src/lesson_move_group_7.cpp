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

using namespace std;

double PI=3.14159;

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

int ik (tf::Vector3 Pos, double *Joints1, double *Joints2){
  tf::Vector3 x(1.0,0.0,0.0);
  tf::Vector3 y(0.0,1.0,0.0);
  tf::Vector3 z(0.0,0.0,1.0);
  if(fabs(Pos.getX())<0.000001 && fabs(Pos.getY())<0.000001)
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
  std::vector<std::string> bodyJointsRightArm; 

  tf::Transform transLeftBaseLinkLeftArm;
  tf::Transform transNeckLeftBaseLink;
  tf::Transform transRightBaseLinkRightArm;
  tf::Transform transNeckRightBaseLink;

  tf::TransformListener tfListener;  // tf listener to data from external tf broadcaster
  tf::TransformBroadcaster tfBroadcaster;  // broadcasts goal transform to tf to visualize it in rviz

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

};
      
int main(int argc, char **argv)
{
  ros::init(argc, argv, "follow_arm_node");
  printf("\nStart ros node now.");
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

  // Initialize Perception Neuron Interface
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  struct tfPercNeuronInterface tfPNIntf;  
  struct tfPercNeuronInterface& tfPNIntfRef = tfPNIntf;
  tfPNIntf.bodyJointsLeftArm = {"Neck","LeftShoulder", "LeftArm"};
  tfPNIntf.bodyJointsRightArm = {"Neck","RightShoulder", "RightArm"};

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
  tf::Vector3 ArmPos;
  std::vector<double> current_joint_values;
  double* Joints1=new double[2];
  double* Joints2=new double[2];
  tf::Quaternion Quat;
  double roll,pitch,yaw;
  double d1=0;
  double d2=0;
  double d=0;
 
  while (node.ok()) {

    // Get goal transform for left UR5's base_link to
    // Perception Neuron's LeftHand
    success = -1;
    while (success == -1) {
      tfPNIntf.tfBroadcaster.sendTransform(
          tf::StampedTransform(tfPNIntf.transNeckLeftBaseLink, ros::Time::now(),
                            "/Neck","/LeftBaseLink"));
      printf("\nSuccess publish tf from neck to LeftBaseLink.");
      success = tfPNIntfRef.gettransLeftBaseLinkLeftArm();
    }

  // Get goal transform for Right UR5's base_link to
    // Perception Neuron's RightHand
    success = -1;
    while (success == -1) {
      tfPNIntf.tfBroadcaster.sendTransform(
          tf::StampedTransform(tfPNIntf.transNeckRightBaseLink, ros::Time::now(),
                            "/Neck","/RightBaseLink"));
      printf("\nSuccess publish tf from neck to RightBaseLink.");
      success = tfPNIntfRef.gettransRightBaseLinkRightArm();
    }

//compute the left ur5's joint values and execute
    ArmPos=tfPNIntfRef.transLeftBaseLinkLeftArm.getOrigin();
    ArmPos.setX(-1*ArmPos.getX());
    ArmPos.setY(-1*ArmPos.getY());
    printf("\nSuccess get tf from LeftBaseLink to LinkLeftArm.");
    int a= ik (ArmPos,Joints1,Joints2);
    printf("\nSuccess get ik.");
    current_joint_values= group1.getCurrentJointValues();
     printf("\n getCurrentJointValues" );
    printf("\n %1.6f,%1.6f\n",
          current_joint_values[0],current_joint_values[1] );

    //choose a suitable set of joints
    d1=0;
    d2=0;
    for(int j=0;j<2;j++){
      d=Joints1[j]-current_joint_values[j];
      if(d>PI){
        d=d-2*PI;
      }
      if(d<-1*PI){
        d=d+2*PI;
      }
      d1=d1+d*d;
      d=Joints2[j]-current_joint_values[j];
      if(d>PI){
        d=d-2*PI;
      }
      if(d<-1*PI){
        d=d+2*PI;
      }
      d2=d2+d*d;
    }
    if(d1<=d2){
      joints1["left_shoulder_pan_joint"] = Joints1[0];
      joints1["left_shoulder_lift_joint"] =  Joints1[1];
    }else{
      joints1["left_shoulder_pan_joint"] = Joints2[0];
      joints1["left_shoulder_lift_joint"] =  Joints2[1];
    }
     printf("\n shoulder_pan_joint and shoulder_lift_joint" );
    printf("\n %1.6f,%1.6f\n",
          joints1["left_shoulder_pan_joint"],joints1["left_shoulder_lift_joint"]  );

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
    joints1["left_elbow_joint"] = pitch;
    printf("\nSuccess get joint 5.");

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
    joints1["left_wrist_1_joint"] =  -1*PI/2+pitch;
    joints1["left_wrist_2_joint"] =  PI/2-yaw;

    joints1["left_wrist_3_joint"] = 0;

    printf("\n %1.6f,%1.6f,%1.6f,%1.6f,%1.6f,%1.6f\n",
          joints1["left_shoulder_pan_joint"],joints1["left_shoulder_lift_joint"], joints1["left_elbow_joint"],
          joints1["left_wrist_1_joint"], joints1["left_wrist_1_joint"],joints1["left_wrist_2_joint"]  );
    // group1.setJointValueTarget(joints1);
    // if (!group1.plan(plan1))
    // {
    //   ROS_FATAL("Unable to create motion plan.  Aborting.");
    //   exit(-1);
    // }
    // group1.asyncExecute(plan1);

//compute the right ur5's joint values and execute
    ArmPos=tfPNIntfRef.transRightBaseLinkRightArm.getOrigin();
    ArmPos.setX(-1*ArmPos.getX());
    ArmPos.setY(-1*ArmPos.getY());
    printf("\nSuccess get tf from RightBaseLink to RightLeftArm.");
    a= ik (ArmPos,Joints1,Joints2);
    printf("\nSuccess get ik.");
    current_joint_values= group2.getCurrentJointValues();
     printf("\n getCurrentJointValues" );
    printf("\n %1.6f,%1.6f\n",
          current_joint_values[0],current_joint_values[1] );

    d1=0;
    d2=0;
    for(int j=0;j<2;j++){
      d=Joints1[j]-current_joint_values[j];
      if(d>PI){
        d=d-2*PI;
      }
      if(d<-1*PI){
        d=d+2*PI;
      }
      d1=d1+d*d;
      d=Joints2[j]-current_joint_values[j];
      if(d>PI){
        d=d-2*PI;
      }
      if(d<-1*PI){
        d=d+2*PI;
      }
      d2=d2+d*d;
    }
    if(d1<=d2){
      joints2["right_shoulder_pan_joint"] = Joints1[0];
      joints2["right_shoulder_lift_joint"] =  Joints1[1];
    }else{
      joints2["right_shoulder_pan_joint"] = Joints2[0];
      joints2["right_shoulder_lift_joint"] =  Joints2[1];
    }
     printf("\n Right shoulder_pan_joint and Right shoulder_lift_joint" );
    printf("\n %1.6f,%1.6f\n",
          joints2["right_shoulder_pan_joint"],joints2["right_shoulder_lift_joint"]  );

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
    joints2["right_elbow_joint"] = pitch;
    printf("\nSuccess get joint 5.");

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
    joints2["right_wrist_1_joint"] =  -1*PI/2+pitch;
    joints2["right_wrist_2_joint"] =  -1*PI/2-yaw;

    joints2["right_wrist_3_joint"] = 0;

    printf("\n %1.6f,%1.6f,%1.6f,%1.6f,%1.6f,%1.6f\n",
          joints2["right_shoulder_pan_joint"],joints2["right_shoulder_lift_joint"], joints2["right_elbow_joint"],
          joints2["right_wrist_1_joint"], joints2["right_wrist_1_joint"],joints2["right_wrist_2_joint"]  );
    group2.setJointValueTarget(joints2);
    if (!group2.plan(plan2))
    {
      ROS_FATAL("Unable to create motion plan.  Aborting.");
      exit(-1);
    }
    group2.asyncExecute(plan2);

    rate.sleep();
  }
  ros::shutdown();

  group1.stop();
  group2.stop();
  sleep(1.0); 
  return 0;
}
