#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
    /// Initialize ROS
    ros::init(argc, argv, "r");
    /// Define Node
    ros::NodeHandle nh_;

    ros::Publisher path_pub = nh_.advertise<nav_msgs::Path>("/global_planner/path",100);
	nav_msgs::Path path;
	sleep(1);
	path.header.stamp=ros::Time::now();
	path.header.frame_id="map";
	path.header.seq = 0;
	int counter = 0;

	geometry_msgs::PoseStamped this_pose_stamped;
	for(float i=7.905;i>=2.325;i-=0.3){
	    this_pose_stamped.pose.position.y=i;
	    this_pose_stamped.pose.position.x=5.115;
	    this_pose_stamped.header.stamp=ros::Time::now();
	    this_pose_stamped.header.frame_id="map";
	    this_pose_stamped.header.seq = counter;
	    ++counter;
	    path.poses.push_back(this_pose_stamped);
	}
	for(float i=5.115;i>=1.395;i-=0.3){
	    this_pose_stamped.pose.position.x=i;
	    this_pose_stamped.pose.position.y=2.325;
	    this_pose_stamped.header.stamp=ros::Time::now();
	    this_pose_stamped.header.frame_id="map";
	    this_pose_stamped.header.seq = counter;
	    ++counter;
	    path.poses.push_back(this_pose_stamped);
	}
	for(float i=2.325;i<=5.115;i+=0.3){
	    this_pose_stamped.pose.position.y=i;
	    this_pose_stamped.pose.position.x=1.395;
	    this_pose_stamped.header.stamp=ros::Time::now();
	    this_pose_stamped.header.frame_id="map";
	    this_pose_stamped.header.seq = counter;
	    ++counter;
	    path.poses.push_back(this_pose_stamped);
	}
	for(float i=1.395;i<=4.185;i+=0.3){
	    this_pose_stamped.pose.position.x=i;
	    this_pose_stamped.pose.position.y=5.115;
	    this_pose_stamped.header.stamp=ros::Time::now();
	    this_pose_stamped.header.frame_id="map";
	    this_pose_stamped.header.seq = counter;
	    ++counter;
	    path.poses.push_back(this_pose_stamped);
	}
	for(float i=4.185;i>=2.325;i-=0.3){
	    this_pose_stamped.pose.position.x=i;
	    this_pose_stamped.pose.position.y=5.115;
	    this_pose_stamped.header.stamp=ros::Time::now();
	    this_pose_stamped.header.frame_id="map";
	    this_pose_stamped.header.seq = counter;
	    ++counter;
	    path.poses.push_back(this_pose_stamped);
	}
	for(float i=5.115;i<=6.975;i+=0.3){
	    this_pose_stamped.pose.position.x=2.325;
	    this_pose_stamped.pose.position.y=i;
	    this_pose_stamped.header.stamp=ros::Time::now();
	    this_pose_stamped.header.frame_id="map";
	    this_pose_stamped.header.seq = counter;
	    ++counter;
	    path.poses.push_back(this_pose_stamped);
	}
	for(float i=2.325; i<=1.395; i-=0.3){
	    this_pose_stamped.pose.position.x=i;
	    this_pose_stamped.pose.position.y=6.975;
	    this_pose_stamped.header.stamp=ros::Time::now();
	    this_pose_stamped.header.frame_id="map";
	    this_pose_stamped.header.seq = counter;
	    ++counter;
	    path.poses.push_back(this_pose_stamped);
	}
	for(float i=6.975;i<=7.905;i+=0.3){
	    this_pose_stamped.pose.position.x=1.395;
	    this_pose_stamped.pose.position.y=i;
	    this_pose_stamped.header.stamp=ros::Time::now();
	    this_pose_stamped.header.frame_id="map";
	    this_pose_stamped.header.seq = counter;
	    ++counter;
	    path.poses.push_back(this_pose_stamped);
	}
	for(float i=1.395;i>=0.465;i-=0.3){
	    this_pose_stamped.pose.position.x=i;
	    this_pose_stamped.pose.position.y=7.905;
	    this_pose_stamped.header.stamp=ros::Time::now();
	    this_pose_stamped.header.frame_id="map";
	    this_pose_stamped.header.seq = counter;
	    ++counter;
	    path.poses.push_back(this_pose_stamped);
	}
	
	path_pub.publish(path);
	sleep(1);
	ros::spinOnce();
    
    return 0;
}
