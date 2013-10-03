#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include <vector>
#include <iostream>

using namespace std;


class Avoid 
{

	public:

	float min_range;
	float avoid_angle;
	float roll;
	float pitch;
	float avoid_threshold;
	float K;

	ros::NodeHandle nh;
	ros::Subscriber sub_scan;
	ros::Publisher pub_move;

	Avoid(ros::NodeHandle& _nh): nh(_nh) {
			pub_move = nh.advertise<geometry_msgs::PointStamped>("/avoid", 1);
			sub_scan = nh.subscribe("/scan", 1, &Avoid::lrf_callback, this);
			avoid_threshold = 2.0;
			roll = 0.0;
			pitch = 0.0;
			K = 8.0;
		}

	~Avoid() {}

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

		float current_angle = min_angle;
	
		min_range = 999.0;
		avoid_angle = current_angle;
		for (int i=0; i < tot_scans; i++)
		{
			current_angle = min_angle + (angle_increment*i);
			if ((current_scan[i] < min_range) && (current_scan[i] > 0.1)) 
			{
				min_range = current_scan[i];
				avoid_angle = current_angle;
			}
		}		
		cout << "min_range = " << min_range << "   avoid_angle = " << avoid_angle << endl;

		if (min_range < avoid_threshold)
		{
                    float pitch_gain;
                    float roll_gain;
                    float magnitude = (K/min_range)<(K/.2)?(K/min_range):(K/.2);

                    pitch_gain = cos(avoid_angle);
                    roll_gain = sin(avoid_angle);

                    pitch = pitch_gain * magnitude;
                    roll = roll_gain * magnitude;

		} else
		{
			roll = 0.0;
			pitch = 0.0;
		}


		cout << "roll = " << roll << "   pitch = " << pitch << endl << endl;
		geometry_msgs::Point32 move_msg;
		move_msg.x = roll;
		move_msg.y = pitch;
		move_msg.z = 0.0;
		
		pub_move.publish(move_msg);
	}

};



int main (int argc, char** argv) 
{
	ros::init(argc, argv, "obstacle_avoid");
	ros::NodeHandle nh;
	
	Avoid av(nh);


	ros::Rate loop_rate(10);
	while(ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
