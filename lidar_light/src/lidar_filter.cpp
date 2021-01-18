#include "ros/ros.h"
//#include "std_msgs/Int32.h"
#include <sensor_msgs/LaserScan.h>
#include <iostream>

ros::Publisher lidar_pub;
ros::Subscriber lidar_sub;

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){

	//ROS_INFO("Received: \n");
	//ROS_INFO("angle min %f", msg->angle_min);
	//ROS_INFO("angle max %f", msg->angle_max);
	//ROS_INFO("angle increment %f", msg->angle_increment);
	//ROS_INFO("range[180] %f", msg->ranges[180]);
	//ROS_INFO("intensities[180] %f", msg->intensities[180]);
	//ROS_INFO("number of samples %d", msg->ranges.size());
	
	//int num = msg->ranges.size();
	
	sensor_msgs::LaserScan scan_filtered;
	scan_filtered = *msg;
		
	//clutter 
	int i =0;
	for(i=0;i<90;i++)
	{
		scan_filtered.ranges[i]=0;
		scan_filtered.intensities[i]=0;	
	}

	for(i=270;i<360;i++)
	{
		scan_filtered.ranges[i]=0;
		scan_filtered.intensities[i]=0;	
	}

	//strcpy(scan_filtered.header.frame_id,"lidar_link");
	scan_filtered.header.frame_id = "lidar_link";
	scan_filtered.header.stamp = ros::Time::now();

	lidar_pub.publish(scan_filtered); 

	//distance = msg->ranges[180];

}

int main(int argc, char **argv){
	ros::init(argc,argv,"lidar_filter");
	ros::NodeHandle node_obj;
	lidar_pub = node_obj.advertise<sensor_msgs::LaserScan>("/scan_filter",4);
	lidar_sub = node_obj.subscribe("/scan",4,scan_callback);
	ros::spin();
	return 0;
}

