//
// Created by charmyoung on 19-7-24.
//
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/PoseStamped.h"

int main( int argc, char** argv ){
  ros::init(argc,argv,"test_tf");
  ros::NodeHandle nh;
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("Pose",1, true);
  tf::TransformListener listener;

  tf::StampedTransform transform;

  while(!listener.waitForTransform("map", "odom", ros::Time(0),ros::Duration(1.0))) {
    ros::Duration(1.0).sleep();
  }

  try {
    //Listen to map->odom transform
    listener.lookupTransform("map", "odom", ros::Time(0), transform);
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  ROS_INFO("Reference Frame: %s, Child Frame: %s, ",
           transform.frame_id_.c_str(),
           transform.child_frame_id_.c_str());

  //Calculate 3x1 Translation Vector
  ROS_INFO("Translation Vector: ");
  ROS_INFO("x: %f, y: %f, z: %f ",
           transform.getOrigin().x(),
           transform.getOrigin().y(),
           transform.getOrigin().z()
  );

  //Calculate 3x3 Rotation Matrix
  tf::Matrix3x3 m(transform.getRotation());
  ROS_INFO("Rotation Matrix: ");
  ROS_INFO(" [ %f,  %f,  %f ",
           m.getRow(0).x(), m.getRow(0).y(), m.getRow(0).z());
  ROS_INFO("   %f,  %f,  %f ",
           m.getRow(1).x(), m.getRow(1).y(), m.getRow(1).z());
  ROS_INFO("   %f,  %f,  %f ]",
           m.getRow(2).x(), m.getRow(2).y(), m.getRow(2).z());

  //Calculate Euler Angle
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  ROS_INFO("Euler Angle: ");
  ROS_INFO(" yaw: %f, pitch: %f, roll: %f",
           yaw, pitch, roll);

  geometry_msgs::PoseStamped pub_odom_pose;
  pub_odom_pose.header.frame_id = "odom";
  pub_odom_pose.pose.position.x = 5;
  pub_odom_pose.pose.position.y = 3;
  pub_odom_pose.pose.position.z = 0;
  pub_odom_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI/6);
  tf::Stamped<tf::Pose> odom_pose,map_pose1,map_pose2;
  tf::poseStampedMsgToTF(pub_odom_pose,odom_pose);
  pose_pub.publish(pub_odom_pose);

  // Calculate map pose by multiply transform matrix
  map_pose1.frame_id_="map";
  map_pose1.stamp_ = transform.stamp_;
  map_pose1.setData(transform*odom_pose);
  m = tf::Matrix3x3(map_pose1.getRotation());
  m.getRPY(roll, pitch, yaw);
  ROS_INFO("transform map_pose: ");
  ROS_INFO(" x: %f, y: %f, z: %f , yaw: %f, pitch: %f, roll: %f",
           map_pose1.getOrigin().x(),
           map_pose1.getOrigin().y(),
           map_pose1.getOrigin().z(),
           yaw, pitch ,roll);

  // Calculate map pose by tf transformPose function
  listener.transformPose("map", odom_pose, map_pose2);
  m = tf::Matrix3x3(map_pose2.getRotation());
  m.getRPY(roll, pitch, yaw);
  ROS_INFO("transform map_pose2: ");
  ROS_INFO(" x: %f, y: %f, z: %f , yaw: %f, pitch: %f, roll: %f",
           map_pose2.getOrigin().x(),
           map_pose2.getOrigin().y(),
           map_pose2.getOrigin().z(),
           yaw, pitch ,roll);
}