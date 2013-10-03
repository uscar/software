#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <math.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Header.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#define PI 3.141592

using namespace std;


class Hold_pos
{
public:
    ros::NodeHandle nh;
    message_filters::Subscriber<geometry_msgs::PointStamped> sub_avoid;
    message_filters::Subscriber<geometry_msgs::PointStamped> sub_icp;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, geometry_msgs::PointStamped> Policy_sync_subs;
    message_filters::Synchronizer<Policy_sync_subs> sync_subs;
    ros::Publisher pub_move;
	geometry_msgs::Point32 move_msg;

	float x, y, theta, x_des, y_des, theta_des, d_x, d_y, d_theta;
	float roll_avoid;          // roll command from obstacle avoidance
	float roll_odom;		   // roll command from icp
	float pitch_avoid;         // pitch command from obstacle avoidance
	float pitch_odom;          // pitch command from odometry
	float roll;		           // resultant roll to send to ardupilot
	float pitch;		       // resultant pitch to send to ardupilot

	bool new_plan;			   // flag used to control whether (x/y/theta)_des can be set

	Hold_pos(ros::NodeHandle& _nh): nh(_nh), sub_avoid(nh, "/move", 10), sub_icp(nh, "/icp", 10), 
				       sync_subs(Policy_sync_subs(10), sub_avoid, sub_icp) 
	{
	    sync_subs.registerCallback(boost::bind(&Hold_pos::sync_subs_callback, this, _1, _2));
		pub_move = nh.advertise<geometry_msgs::Point32>("/arduino/move", 1);
		roll = 0.0;
		roll_avoid = 0.0;
		roll_odom = 0.0;
		pitch = 0.0;
		pitch_avoid = 0.0;
		pitch_odom = 0.0;
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		x_des = 0.0;
		y_des = 0.0;
		theta_des = 0.0;
		d_x = 0.0;
		d_y = 0.0;
		d_theta = 0.0;
		new_plan = true;
	}



	float response(float dx)
	{
		/*
			Response of the form A*tanh(B*dx), where A scales the min/max angle and B scales the slope.
			Positive dx (want to move backwards or to the left) results in positive angle response ->  
						positive pitch angle moves quad backwards but positive roll angle moves quad to the right
		*/
		float angle = 5*tanh(5*dx/2);
		return angle;
	}


	void sync_subs_callback(const geometry_msgs::PointStamped::ConstPtr& msg_avoid, const geometry_msgs::PointStamped::ConstPtr& msg_icp) 
    {
		
		if (new_plan)
		{
			// Set new waypoints
			x_des = msg_icp->point.x;
			y_des = msg_icp->point.y;
			theta_des = msg_icp->point.z;
			x = x_des;
			y = y_des;
			theta = theta_des;

			// set attitude response
			roll_avoid = msg_avoid->point.x;
			pitch_avoid = msg_avoid->point.y;
			
			roll = roll_avoid;
			pitch = pitch_avoid;
			
			// publish attitude response
			move_msg.x = roll;
			move_msg.y = pitch;
			move_msg.z = 0.0;
			pub_move.publish(move_msg);

			new_plan = false;

		} else
		{
			x = msg_icp->point.x;
			y = msg_icp->point.y;
			theta = msg_icp->point.z;

			d_x = x - x_des;
			d_y = y - y_des;
			d_theta = theta - theta_des;

			// set attitude response assuming positive x forward and positive y to the right
			roll_odom = response(d_y);
			pitch_odom = response(d_x);			

			roll_avoid = msg_avoid->point.x;
			pitch_avoid = msg_avoid->point.y;

			roll = roll_avoid + roll_odom;
			pitch = pitch_avoid + pitch_odom;

			// publish attitude response
			move_msg.x = roll;
			move_msg.y = pitch;
			move_msg.z = d_theta;
			pub_move.publish(move_msg);

		}
		
	}
};



int main (int argc, char** argv) {
    ros::init(argc, argv, "hold_pos");
    ros::NodeHandle nh;
	
    Hold_pos plan(nh);


    ros::Rate loop_rate(10);
    while(ros::ok()) {
	ros::spinOnce();
	loop_rate.sleep();
    }
    return 0;
}

