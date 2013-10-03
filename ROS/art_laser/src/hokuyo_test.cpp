#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <vector>
#include <iostream>

using namespace std;


class Hokuyo_Test 
{

	public:

	ros::NodeHandle nh;
	ros::Subscriber sub_scan;

	Hokuyo_Test(ros::NodeHandle& _nh): nh(_nh) {
			sub_scan = nh.subscribe("/scan", 1, &Hokuyo_Test::lrf_callback, this);
		}

	~Hokuyo_Test() {}

	void lrf_callback(const sensor_msgs::LaserScan::ConstPtr& msg_lrf) 
	{
		
		int tot_scans = msg_lrf->ranges.size();
	    float min_range = msg_lrf->range_min;
	    float max_range = msg_lrf->range_max;
	    double min_angle = msg_lrf->angle_min;
	    double max_angle = msg_lrf->angle_max;
	    double angle_increment = msg_lrf->angle_increment;
	
		vector<float> current_scan;

		for (int i=0; i < tot_scans; i++)
		{
			current_scan.push_back(msg_lrf->ranges[i]);
		}


		int p = tot_scans/2;
		cout << current_scan[p] << endl;
	}

};



int main (int argc, char** argv) 
{
	ros::init(argc, argv, "hokuyo_test");
	ros::NodeHandle nh;
	
	Hokuyo_Test ht(nh);


	ros::Rate loop_rate(10);
	while(ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
