#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <cmath>
#include <vector>
#include <iostream>

using namespace std;

ros::Publisher pub_cloud;
ros::Subscriber sub_scan;

void cb_scan(const sensor_msgs::LaserScan::ConstPtr& msg_lrf)
{
	
	float min = msg_lrf->angle_min;
	cout << msg_lrf->ranges[msg_lrf->ranges.size()/2] << endl << endl;
	float angle, range;
	sensor_msgs::PointCloud cloud;
	cloud.header.frame_id = "/world";
	cloud.points.resize(msg_lrf->ranges.size());
	for (int i = 0; i < msg_lrf->ranges.size(); i++)
	{
		angle = min + (i*msg_lrf->angle_increment);
		range = msg_lrf->ranges[i];
		cloud.points[i].x = range*sin(angle);
		cloud.points[i].y = range*cos(angle);
		cloud.points[i].z = 1;
	}

	pub_cloud.publish(cloud);
}

int main (int argc, char** argv) 
{
	ros::init(argc, argv, "display");
	ros::NodeHandle nh;

	pub_cloud = nh.advertise<sensor_msgs::PointCloud>("/cloud" , 2);
	sub_scan = nh.subscribe("/scan", 2, cb_scan);
	
	ros::Rate loop_rate(1);
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
