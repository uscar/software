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

class FlashDriveFinderNode {

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
	
	FlashDriveFinderNode(ros::NodeHandle& _nh) {
		this->nh = _nh;
		
		subImageCount = 0;
		imgColor = Mat(cv::Size(640, 480), CV_8UC3);
		imgGrayPrevious = Mat(cv::Size(640, 480), CV_8UC1);
		imgGray = Mat(cv::Size(640, 480), CV_8UC1);
		
		//subscribe to the image
		subImage = nh.subscribe("/camera/image_raw", 10, &FlashDriveFinderNode::subImageCallback, this);

		//Create a cv window to view image in for now
		cv::namedWindow("Image");
	}
	
	~FlashDriveFinderNode() {
		cv::destroyWindow("Image");
		imgColor.release();
		imgGray.release();
	}
	
	private:
	
	void subImageCallback(const sensor_msgs::ImageConstPtr& msg) {
		//if(subImageCount % 4 != 0) {
		//	subImageCount++;
		//	return;
		//}
		
		subImageCount++;
		cout << "Got image number: " << subImageCount << endl;
		
		
		//convert image from ROS image message to OpenCV Mat
		convert_image(msg, imgColor);
		
		//Do things with the image here
		
		//int numPyramidLevels = 0;
		//numPyramidLevels = buildOpticalFlowPyramid(prevImgGray, imgColor, Size winSize, int maxLevel, bool withDerivatives=true, int pyrBorder=BORDER_REFLECT_101, int derivBorder=BORDER_CONSTANT, bool tryReuseInputImage=true);
		
		
		vector<Point2f> *corners = new vector<Point2f>();
		cvtColor(imgColor, imgGray, CV_RGB2GRAY);
		
		if(subImageCount <= 1) {
			imgGrayPrevious = imgGray;
			return;
		}
		
		goodFeaturesToTrack(imgGrayPrevious, *corners, 100, .1, 10, Mat(), 3, true, .04);
		
		
		vector<Point2f> *nextPoints = new vector<Point2f>((*corners).size(), Point2f(0, 0));
		vector<uchar> status;
		vector<float> errors;
		

		calcOpticalFlowPyrLK(imgGrayPrevious, imgGray, *corners, *nextPoints, status, errors, Size(21,21), OPTFLOW_USE_INITIAL_FLOW, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.3), 0, .0001);
		
		for(int i = 0; i < corners->size(); i++) {
			printf("corners: x = %f y = %f  nextPts: x = %f y = %f\n", (*corners)[i].x, (*corners)[i].y, (*nextPoints)[i].x, (*nextPoints)[i].y);
		}
		
		cout << "Drawing circles..." << endl;
		for(unsigned int i = 0; i < corners->size(); i++) {
			cout << "point " << i << " x = " << (*corners)[i].x << "  y = " << (*corners)[i].y << endl;
			circle(imgColor, (*corners)[i], 8, cvScalar(255, 0, 0), 2, 8, 0);
		}
		
		cout << "Drawing lines..." << endl;
		cout << "corners size: " << corners->size() << "  nextpoints size: " << nextPoints->size() << endl;
		for(unsigned int i = 0; i < corners->size(); i++) {
			cout << i << endl;
			line(imgColor, (*corners)[i], (*nextPoints)[i], cvScalar(0, 0, 255), 1, 8, 0);
		}
		cout << "Done drawing" << endl;
		
		
		//Display that image back onto the window
		cv::imshow("Image", imgColor);
		
		//Wait...
		cv::waitKey(1);
		imgGrayPrevious = imgGray.clone();
	}

	void convert_image(const sensor_msgs::ImageConstPtr& _msg, Mat& _img)
	{
		cv_bridge::CvImageConstPtr cv_image_ptr;
		cv_image_ptr = cv_bridge::toCvShare(_msg, "bgr8");
		_img = cv_image_ptr->image;
	}
	
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "flash_drive_finder");
	ros::NodeHandle nh;
	
	cout << "Initializing FlashDriveFinderNode..." << endl;
	FlashDriveFinderNode flashDriveFinderNode(nh);
	cout << "Initialization Complete" << endl;
	
	ros::spin();
	return 0;
}

