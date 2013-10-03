#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Eigen>
#include <vector>



#include "plane.h"
#include <cmath>
#include <iostream>

using namespace std;

cPlaneEKF::cPlaneEKF() 
{
        setDim(4, 1, 2, 2, 2);
        Period = 0.2;
        Gravity = 9.8;
        Bfriction = 0.35;
        Portance = 3.92;
        Mass = 1000;
}

void cPlaneEKF::makeBaseA()
{
        A(1,1) = 1.0;
        // A(1,2) = Period - Period*Period*Bfriction/Mass*x(2);
        A(1,3) = 0.0;
        A(1,4) = 0.0;

        A(2,1) = 0.0;
        // A(2,2) = 1 - 2*Period*Bfriction/Mass*x(2);
        A(2,3) = 0.0;
        A(2,4) = 0.0;

        A(3,1) = 0.0;
        // A(3,2) = Period*Period*Portance/Mass*x(2);
        A(3,3) = 1.0;
        A(3,4) = Period;

        A(4,1) = 0.0;
        // A(4,2) = 2*Period*Portance/Mass*x(2);
        A(4,3) = 0.0;
        A(4,4) = 1.0;
}

void cPlaneEKF::makeA()
{
        // A(1,1) = 1.0;
        A(1,2) = Period - Period*Period*Bfriction/Mass*x(2);
        // A(1,3) = 0.0;
        // A(1,4) = 0.0;

        // A(2,1) = 0.0;
        A(2,2) = 1 - 2*Period*Bfriction/Mass*x(2);
        // A(2,3) = 0.0;
        // A(2,4) = 0.0;

        // A(3,1) = 0.0;
        A(3,2) = Period*Period*Portance/Mass*x(2);
        // A(3,3) = 1.0;
        // A(3,4) = Period;

        // A(4,1) = 0.0;
        A(4,2) = 2*Period*Portance/Mass*x(2);
        // A(4,3) = 0.0;
        // A(4,4) = 1.0;
}

void cPlaneEKF::makeBaseW()
{
        W(1,1) = 0.0;
        W(1,2) = 0.0;
        W(2,1) = 1.0;
        W(2,2) = 0.0;
        W(3,1) = 0.0;
        W(3,2) = 0.0;
        W(4,1) = 0.0;
        W(4,2) = 1.0;
}

void cPlaneEKF::makeBaseQ()
{
        Q(1,1) = 0.01*0.01;
        Q(1,2) = 0.01*0.01/10.0;
        Q(2,1) = 0.01*0.01/10.0;
        Q(2,2) = 0.01*0.01;
}

void cPlaneEKF::makeBaseH()
{
        // H(1,1) = -x(3)/(x(1)*x(1)+x(3)*x(3));
        H(1,2) = 0.0;
        // H(1,3) = x(1)/(x(1)*x(1)+x(3)*x(3));
        H(1,4) = 0.0;

        // H(2,1) = x(1)/sqrt(x(1)*x(1)+x(3)*x(3));
        H(2,2) = 0.0;
        // H(2,3) = x(3)/sqrt(x(1)*x(1)+x(3)*x(3));
        H(2,4) = 0.0;
}

void cPlaneEKF::makeH()
{
        H(1,1) = -x(3)/(x(1)*x(1)+x(3)*x(3));
        // H(1,2) = 0.0;
        H(1,3) = x(1)/(x(1)*x(1)+x(3)*x(3));
        // H(1,4) = 0.0;

        H(2,1) = x(1)/sqrt(x(1)*x(1)+x(3)*x(3));
        // H(2,2) = 0.0;
        H(2,3) = x(3)/sqrt(x(1)*x(1)+x(3)*x(3));
        // H(2,4) = 0.0;
}

void cPlaneEKF::makeBaseV()
{
        V(1,1) = 1.0;
        V(2,2) = 1.0;
}

void cPlaneEKF::makeBaseR()
{
        R(1,1) = 0.01*0.01;
        R(2,2) = 50*50;
}

void cPlaneEKF::makeProcess()
{
        Vector x_(x.size());
        x_(1) = x(1) + x(2)*Period + (Period*Period)/2*(u(1)/Mass - Bfriction/Mass*x(2)*x(2));
        x_(2) = x(2) + (u(1)/Mass - Bfriction/Mass*x(2)*x(2))*Period;
        x_(3) = x(3) + x(4)*Period + (Period*Period)/2*(Portance/Mass*x(2)*x(2)-Gravity);
        x_(4) = x(4) + (Portance/Mass*x(2)*x(2)-Gravity)*Period;
        x.swap(x_);
}

void cPlaneEKF::makeMeasure()
{
        z(1)=atan2(x(3), x(1));
        z(2)=sqrt(x(1)*x(1)+x(3)*x(3));
}



using namespace Eigen;

class EKF
{

public:
	ros::NodeHandle nh;
	//Subscribe to scan matching data (x,y,heading) and laser altitude estimation (z)
	message_filters::Subscriber<geometry_msgs::PointStamped> sub_xyheading, sub_z;
	//Subscribe to IMU data from arduino (roll, pitch, x_accel, y_accel, z_accel)
	message_filters::Subscriber<geometry_msgs::PoseStamped> sub_arduino;
	typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, geometry_msgs::PoseStamped> Policy_sync_subs;
    message_filters::Synchronizer<Policy_sync_subs> sync_subs;
	//Figure out how and to publish
	//ros::Publisher pub_ekf;


	EKF(ros::NodeHandle& _nh): nh(_nh), sub_xyheading(nh, "/odometry", 10), sub_z(nh, "/altitude", 10), sub_arduino(nh, "/arduino/IMU", 10),
										sync_subs(Policy_sync_subs(10), sub_xyheading, sub_z, sub_arduino) 
		{
			//pub_ekf = nh.advertise<sensor_msgs::PointCloud>("/world", 10);
			sync_subs.registerCallback(boost::bind(&EKF::sync_subs_callback, this, _1, _2, _3));
		}

};
