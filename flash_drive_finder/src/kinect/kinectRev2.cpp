#include <ros/ros.h>
#include <cmath>
#include <string>
#include <iostream>
#include <cv.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class kinectNode {

	public:

	private:
		ros::NodeHandle nh;
		ros::Subscriber subImage;
		uint subImageCount;
		
		Mat imgColor;
		Mat imgGray;
		Mat imgGrayPrevious;
		Mat imgCorner;

	public:
	
	kinectNode(ros::NodeHandle& _nh) {
		this->nh = _nh;
		
		subImageCount = 0;
		imgColor = Mat(cv::Size(640, 480), CV_8UC3);
		imgGrayPrevious = Mat(cv::Size(640, 480), CV_8UC1);
		imgGray = Mat(cv::Size(640, 480), CV_8UC1);
		
		//subscribe to the image
		//subImage = nh.subscribe("/camera/rgb/image_rect", 10, &kinectNode::subImageCallback, this);
		subImage = nh.subscribe("/camera/depth/image_rect", 10, &kinectNode::subImageCallback, this);

		//Create a cv window to view image in for now
		cv::namedWindow("Image");
	}
	
	~kinectNode() {
		cv::destroyWindow("Image");
		imgColor.release();
		imgGray.release();
	}
	
	private:
	
	void subImageCallback(const sensor_msgs::ImageConstPtr& msg) {
		
		subImageCount++;
		cout << "Got image number: " << subImageCount << endl;
		
		//convert image from ROS image message to OpenCV Mat
		convert_image(msg, imgColor);
		
		vector<Point2f> *corners = new vector<Point2f>();
		cvtColor(imgColor, imgGray, CV_RGB2GRAY);
		
		cv::imshow("Image", imgColor);
		cv::waitKey(1);
	}

	void convert_image(const sensor_msgs::ImageConstPtr& _msg, Mat& _img)
	{
		cv_bridge::CvImageConstPtr cv_image_ptr;
		//cv_image_ptr = cv_bridge::toCvShare(_msg, "bgr8");
		cv_image_ptr = cv_bridge::toCvShare(_msg, "32FC1");
		_img = cv_image_ptr->image;
	}
	
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "flash_drive_finder");
	ros::NodeHandle nh;
	
	cout << "Initializing kinectNode..." << endl;
	kinectNode kinectNode(nh);
	cout << "Initialization Complete" << endl;
	
	ros::spin();
	return 0;
}

