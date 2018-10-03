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
double a[3];

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
  std::vector<std::string> bodyJointsLeftArm;  // Array with tf names of
                                        // Perception Neuron joints. Necessary
                                        // to listen to them with tfListener
  tf::TransformListener
      tfListener;  // tf listener to data from external tf broadcaster
  
  tf::Transform transBaseLinkLeftArm;
  tf::Transform transNeckBaseLink;

  tf::TransformBroadcaster      tfBroadcaster;  // broadcasts goal transform to tf to visualize it in rviz

  // Get tf Transform / Vector from base_link of ur# to RightHand of PercNeuron
  // The UR# End Effector follows this transform
  // returns 0 for success, -1 for failure
  int gettransBaseLinkLeftArm() {
    
    std::vector<tf::Transform> transformArray;
    int success = 0;  

    // Calculate transformation matrix from ur# base_link to RightHand
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    success += pushIntoTransformArray("/Neck",  "/BaseLink", &transformArray,
                                      &tfListener);

    if (success != 0) {
      printf("\nNo tf Transformation found from base_link to Neck.");
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
    
    
    transBaseLinkLeftArm = transformArray.at(0).inverse();
    transBaseLinkLeftArm = transBaseLinkLeftArm.operator*=(transformArray.at(1));
    transBaseLinkLeftArm = transBaseLinkLeftArm.operator*=(transformArray.at(2));
   
    tfBroadcaster.sendTransform(tf::StampedTransform(
        transBaseLinkLeftArm, ros::Time::now(), "/BaseLink", "/BaseLinkToLeftArm"));
    printf("\nsuccess to broadcast tf Transformation from base_link to leftarm.");
    transformArray.clear();
    return 0;
  }
};
	
void printTransformMatrix(tf::Transform& tfTransform) {
  int i = 0;
  printf("\n%1.6f %1.6f %1.6f %1.6f\n",
         tfTransform.getBasis().getColumn(i).getX(),
         tfTransform.getBasis().getColumn(i).getY(),
         tfTransform.getBasis().getColumn(i).getZ(),
         tfTransform.getOrigin().getX());
  i = 1;
  printf("%1.6f %1.6f %1.6f %1.6f\n",
         tfTransform.getBasis().getColumn(i).getX(),
         tfTransform.getBasis().getColumn(i).getY(),
         tfTransform.getBasis().getColumn(i).getZ(),
         tfTransform.getOrigin().getY());
  i = 2;
  printf("%1.6f %1.6f %1.6f %1.6f\n",
         tfTransform.getBasis().getColumn(i).getX(),
         tfTransform.getBasis().getColumn(i).getY(),
         tfTransform.getBasis().getColumn(i).getZ(),
         tfTransform.getOrigin().getZ());
  printf("%1.6f %1.6f %1.6f %1.6f\n", 0.0, 0.0, 0.0, 1.0);
}
      
int main(int argc, char **argv)
{
  ros::init(argc, argv, "follow_arm_node");
  printf("\nStart ros node now.");
  ros::NodeHandle node;
  ros::NodeHandle& nodeRef = node;
  int loopCounter = 0;
  int success;  // int to check for successful execution of functions

//
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveGroup group("manipulator");
  geometry_msgs::Pose pose;
  move_group_interface::MoveGroup::Plan plan;
  std::map<std::string, double> joints;
  joints["shoulder_pan_joint"] = 0;
  joints["shoulder_lift_joint"] =  PI;
  joints["elbow_joint"] =  -1*PI/4;
  joints["wrist_1_joint"] =  -1*PI/2;
  joints["wrist_2_joint"] =  PI/2;
  joints["wrist_3_joint"] = 0;
  group.setJointValueTarget(joints);
  if (!group.plan(plan))
  {
    ROS_FATAL("Unable to create motion plan.  Aborting.");
    exit(-1);
  }
  group.asyncExecute(plan);


  double framesPerSecPercNeuron = 20;

  double mainCycleTime = 1 / framesPerSecPercNeuron;

  // Initialize Perception Neuron Interface
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  struct tfPercNeuronInterface tfPNIntf;  // Interface to perc.neuron tf data
                                          // created by perc neuron tf
                                          // broadcaster
  tfPNIntf.transBaseLinkLeftArm = tf::Transform(
      tf::Quaternion(0, 0, 0, 1),
      tf::Vector3(0, 0, 0));  // Transform from base_link to RightHand directly

  tfPNIntf.tfBroadcaster.sendTransform(tf::StampedTransform(
        tfPNIntf.transBaseLinkLeftArm, ros::Time::now(), "/BaseLink", "/BaseLinkToLeftArm"));

  tf::Transform transLeftUrWorldBaseLink; 
  tf::Transform transLeftHandWorldLeftUrWorld; 
  tf::Transform transNeckLeftHandWorld; 
 
  tf::Quaternion q;
  q.setRPY(-PI/2, PI/2, 0);
  transLeftHandWorldLeftUrWorld.setRotation(q);
  transLeftHandWorldLeftUrWorld.setOrigin(tf::Vector3(0, 0, 0));

  q.setRPY(PI/2, 0, PI/4);
  transLeftUrWorldBaseLink.setRotation(q);
  transLeftUrWorldBaseLink.setOrigin(tf::Vector3(0, 0, 0));


  tf::TransformListener tfListener;
  tf::StampedTransform transform;
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
  

  tfPNIntf.transNeckBaseLink=transNeckLeftHandWorld;
  tfPNIntf.transNeckBaseLink=tfPNIntf.transNeckBaseLink.operator*=(transLeftHandWorldLeftUrWorld);
  tfPNIntf.transNeckBaseLink=tfPNIntf.transNeckBaseLink.operator*=(transLeftUrWorldBaseLink);
                        

  // Create string array with the joints in order of the skeleton hierarchy,
  // provided by the Perception Neuron API, to calculate goal transform
  tfPNIntf.bodyJointsLeftArm = {"Neck","LeftShoulder", "LeftArm"};

  struct tfPercNeuronInterface& tfPNIntfRef = tfPNIntf;

  tf::Vector3 LeftArmPos;
  double* Joints1=new double[2];
  double* Joints2=new double[2];
  tf::Quaternion Quat;
  double roll,pitch,yaw;
  double d1=0;
  double d2=0;
  double d=0;

  // Start the loop to get tf position from Perception Neuron and publish it to the
  // robot / controller
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  printf("\nYou can start moving slowly now.");
  ros::Time begin = ros::Time::now();
  ros::Duration elapsedTime = begin - begin;
  ros::Rate rate(framesPerSecPercNeuron);
  while (node.ok()) {
    begin = ros::Time::now();

    // Get goal transform for UR10's end effector from UR10's base_link to
    // Perception Neuron's RightHand
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    success = -1;
    while (success == -1) {
      tfPNIntf.tfBroadcaster.sendTransform(
          tf::StampedTransform(tfPNIntf.transNeckBaseLink, ros::Time::now(),
                            "/Neck","/BaseLink"));
      printf("\nSuccess publish tf from neck to NeckBaseLink.");
      success = tfPNIntfRef.gettransBaseLinkLeftArm();
    }

    LeftArmPos=tfPNIntfRef.transBaseLinkLeftArm.getOrigin();
    LeftArmPos.setX(-1*LeftArmPos.getX());
    LeftArmPos.setY(-1*LeftArmPos.getY());
    printf("\nSuccess get tf from BaseLink to LinkLeftArm.");
    int a= ik (LeftArmPos,Joints1,Joints2);
    printf("\nSuccess get ik.");
    std::vector<double> current_joint_values= group.getCurrentJointValues();
     printf("\n getCurrentJointValues" );
    printf("\n %1.6f,%1.6f\n",
          current_joint_values[0],current_joint_values[1] );
    // if(Joints1[0]>100){
    //   Joints1[0]=current_joint_values[0];
    // }
    // if(Joints2[0]>100){
    //   Joints2[0]=current_joint_values[0];
    // }

    for(int j=0;j<2;j++){
      d=Joints1[j]-current_joint_values[j];
      if(d>PI){
        d=d-2*PI;
      }
      if(d<-1*PI){
        d=d+PI;
      }
      d1=d1+d*d;
      d=Joints2[j]-current_joint_values[j];
      if(d>PI){
        d=d-2*PI;
      }
      if(d<-1*PI){
        d=d+PI;
      }
      d2=d2+d*d;
    }
    if(d1<=d2){
      joints["shoulder_pan_joint"] = Joints1[0];
      joints["shoulder_lift_joint"] =  Joints1[1];
    }else{
      joints["shoulder_pan_joint"] = Joints2[0];
      joints["shoulder_lift_joint"] =  Joints2[1];
    }
     printf("\n shoulder_pan_joint and shoulder_lift_joint" );
    printf("\n %1.6f,%1.6f\n",
          joints["shoulder_pan_joint"],joints["shoulder_lift_joint"]  );

    //"RightShoulder", "RightArm", "RightForeArm", "RightHand"};
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
    joints["elbow_joint"] = pitch;
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
    joints["wrist_1_joint"] =  -1*PI/2+pitch;
    joints["wrist_2_joint"] =  PI/2-yaw;


    // joints["shoulder_pan_joint"] = -0.4;
    // joints["shoulder_lift_joint"] =  -2;
    // joints["elbow_joint"] =  1.7;
    // joints["wrist_1_joint"] =  0.1;
    // joints["wrist_2_joint"] =  0.75;
    joints["wrist_3_joint"] = 0;

    printf("\n %1.6f,%1.6f,%1.6f,%1.6f,%1.6f,%1.6f\n",
          joints["shoulder_pan_joint"],joints["shoulder_lift_joint"], joints["elbow_joint"],
          joints["wrist_1_joint"], joints["wrist_1_joint"],joints["wrist_2_joint"]  );
    group.setJointValueTarget(joints);
    if (!group.plan(plan))
    {
      ROS_FATAL("Unable to create motion plan.  Aborting.");
      exit(-1);
    }
    group.asyncExecute(plan);

    rate.sleep();
  }
  ros::shutdown();

  group.stop();
  sleep(1.0); 
  return 0;
}
