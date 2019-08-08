//
// Created by kevin on 19-7-27.
//
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <iostream>
#include <memory>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <std_msgs/Bool.h>
#include <cmath>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <memory>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


//ros::Publisher pub_pose_;
//void odometryCallback_(const nav_msgs::Odometry::ConstPtr msg)
//{
//    geometry_msgs::Pose2D pose2d;
//    pose2d.x = msg->pose.pose.position.x;
//    pose2d.y = msg->pose.pose.position.y;
//
//    tf::Quaternion q(
//            msg->pose.pose.orientation.x,
//            msg->pose.pose.orientation.y,
//            msg->pose.pose.orientation.z,
//            msg->pose.pose.orientation.w);
//    tf::Matrix3x3 m(q);
//    double roll, pijitch, yaw;1
//    m.getRPY(roll, pitch, yaw);
//    pose2d.theta = yaw;
//    pub_pose_.publish(pose2d);
//}
const nav_msgs::Odometry::ConstPtr msg;
tf::Transform tag_base_transform;
tf::Quaternion tag_base_q;
tf::TransformListener* listener;
tf::Vector3 base_v_old(0.0,0.0,0.0);
geometry_msgs::Pose odom;
//// Nomenclature: 1. parentFrame_childFrame (If the parentFrame is map, then the parent frame is neglected.)
////               2. _r 3x3 tf rotation matrix, _v 1x3 tf vector, _q tf quaternion.
bool new_detection;

struct CircularMovement{
    const int vx=0;
    float vy;
    float wz;
    float rq;
};


CircularMovement calc(float r){
    CircularMovement A;
    A.rq=r;
    A.vy=-0.1;
    A.wz=fabs(A.vy/A.rq);
    return A;
}

int main(int argc, char **argv)
{
    /// Initialize ROS
    ros::init(argc, argv, "r");
    /// Define Node
    ros::NodeHandle nh_;
    /// Define publication
    ros::Publisher pub_velocity;
    pub_velocity= nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    /// Message type
    geometry_msgs::Twist velocity;
    /// Define tf-listener
    tf::TransformListener listener;
    /// Define tf transformation
    tf::StampedTransform base_map_transform_old;
    tf::StampedTransform odom_base_transform_new;
    tf::StampedTransform tag_cam_transform;
    /// Obtain initial pose of the robot in the odom frame
    /// wait one second to wait  for the publication of map and base_link frames
  ros::Publisher path_pub = nh_.advertise<nav_msgs::Path>("/global_planner/path",5);
ros::Time current_time, last_time;
current_time = ros::Time::now();
last_time = ros::Time::now();
nav_msgs::Path path;
path.header.stamp=current_time;
path.header.frame_id="map";
/*while(!listener.waitForTransform("base_link", "map", ros::Time(0),ros::Duration(1.0))) {
    ros::Duration(1.0).sleep();
}
try {
    listener.lookupTransform("base_link", "map", ros::Time(0), odom_base_transform_old);
    listener.lookupTransform("tag_53", "map", ros::Time(0), odom_base_transform_new);
    listener.lookupTransform("tag_53", "base_link", ros::Time(0), tag_cam_transform);
}
catch (tf::TransformException ex) {
    ROS_ERROR("1:%s", ex.what());
}*/
ros::Rate r(30); /// 30 hz
geometry_msgs::PoseStamped this_pose_stamped;
for(float i=7.905;i>=2.325;i-=0.3){
    this_pose_stamped.pose.position.y=i;
    this_pose_stamped.pose.position.x=5.115;
    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="map";
    path.poses.push_back(this_pose_stamped);
    last_time = current_time;
}
for(float i=5.115;i>=1.395;i-=0.3){
    this_pose_stamped.pose.position.x=i;
    this_pose_stamped.pose.position.y=2.325;
    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="map";
    path.poses.push_back(this_pose_stamped);
    last_time = current_time;
}
for(float i=2.325;i<=5.115;i+=0.3){
    this_pose_stamped.pose.position.y=i;
    this_pose_stamped.pose.position.x=1.395;
    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="map";
    path.poses.push_back(this_pose_stamped);
    last_time = current_time;
}
for(float i=1.395;i<=4.185;i+=0.3){
    this_pose_stamped.pose.position.x=i;
    this_pose_stamped.pose.position.y=5.115;
    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="map";
    path.poses.push_back(this_pose_stamped);
    last_time = current_time;
}
for(float i=4.185;i>=2.325;i-=0.3){
    this_pose_stamped.pose.position.x=i;
    this_pose_stamped.pose.position.y=5.115;
    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="map";
    path.poses.push_back(this_pose_stamped);
    last_time = current_time;
}
for(float i=5.115;i<=6.975;i+=0.3){
    this_pose_stamped.pose.position.x=2.325;
    this_pose_stamped.pose.position.y=i;
    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="map";
    path.poses.push_back(this_pose_stamped);
    last_time = current_time;
}
for(float i=2.325; i<=1.395; i-=0.3){
    this_pose_stamped.pose.position.x=i;
    this_pose_stamped.pose.position.y=6.975;
    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="map";
    path.poses.push_back(this_pose_stamped);
     // check for incoming messages
    last_time = current_time;
}
for(float i=6.975;i<=7.905;i+=0.3){
    this_pose_stamped.pose.position.x=1.395;
    this_pose_stamped.pose.position.y=i;
    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="map";
    path.poses.push_back(this_pose_stamped);
     // check for incoming messages
    last_time = current_time;
}
for(float i=1.395;i>=0.465;i-=0.3){
    this_pose_stamped.pose.position.x=i;
    this_pose_stamped.pose.position.y=7.905;
    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="map";
    path.poses.push_back(this_pose_stamped);
  //   // check for incoming messages
    last_time = current_time;
}
ros::Duration(0.1).sleep();
path_pub.publish(path);
ros::spinOnce();

    


    return 0;

}
