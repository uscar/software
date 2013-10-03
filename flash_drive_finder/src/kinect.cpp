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
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <fovis/fovis.hpp>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>




#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <string>


namespace enc = sensor_msgs::image_encodings;


using namespace std;
using namespace cv;

class kinectNode {

public:

private:
  ros::NodeHandle nh;
  ros::Subscriber subImage;
  uint subImageCount;

  Mat imgColor;
  Mat imgDepth;
  Mat imgGray;
  Mat imgGrayPrevious;
  Mat imgCorner;
  fovis::VisualOdometry* odom;
  fovis::CameraIntrinsicsParameters rgb_params_;

  fovis::DepthImage * depth_image_;
  fovis::Rectification * rect;
  int width;
  int height;
  tf::TransformBroadcaster tf_broadcaster_;

  ros::Publisher odom_pub_;
  fovis::DepthImage * depth_image_;
  fovis::Rectification * rect;
  int width;
  int height;



public:

  message_filters::Subscriber<sensor_msgs::Image> sub_rgb;
  message_filters::Subscriber<sensor_msgs::Image> sub_depth;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> Policy_sync_subs;
  message_filters::Synchronizer<Policy_sync_subs> sync_subs;


  kinectNode(ros::NodeHandle& _nh):
    nh(_nh), sub_rgb(nh, "/camera/rgb/image_rect", 10), sub_depth(nh, "/camera/depth/image_rect", 10), 
    sync_subs(Policy_sync_subs(10), sub_rgb, sub_depth) 
  {


    this->nh = _nh;

    odom_pub_ = _nh.advertise<nav_msgs::Odometry>("fovis_odometry", 1000);

    subImageCount = 0;
    imgColor = Mat(cv::Size(640, 480), CV_8UC3);
    imgDepth = Mat(cv::Size(640, 480), CV_32FC1);
    imgGrayPrevious = Mat(cv::Size(640, 480), CV_8UC1);
    imgGray = Mat(cv::Size(640, 480), CV_8UC1);

    // Create some intrinsic parameters for the kinect.
    // TODO: Pull these from the param server or something
    memset(&rgb_params_, 0, sizeof(fovis::CameraIntrinsicsParameters));
    width = 640;
    height = 480;
    rgb_params_.width = width;
    rgb_params_.height = height;
    rgb_params_.fx = 528.49404721; 
    rgb_params_.fy = rgb_params_.fx;
    rgb_params_.cx = width / 2.0;
    rgb_params_.cy = height / 2.0;

    // get the RGB camera parameters of our device
    rect = new fovis::Rectification(rgb_params_);
    fovis::VisualOdometryOptions options = fovis::VisualOdometry::getDefaultOptions();
    // If we wanted to play around with the different VO parameters, we could set
    // them here in the "options" variable.

    // setup the visual odometry
    odom = new fovis::VisualOdometry(rect, options);
    depth_image_ = new fovis::DepthImage(rgb_params_, width, height);

    //subscribe to the image
    //subImage = nh.subscribe("/camera/rgb/image_rect", 10, &kinectNode::subImageCallback, this);
    //subImage = nh.subscribe("/camera/depth/image_rect", 10, &kinectNode::subImageCallback, this);

    //Create a cv window to view image in for now
    sync_subs.registerCallback(boost::bind(&kinectNode::subImageCallback, this, _1, _2));		

    //cv::namedWindow("Image");
  }

  ~kinectNode() {
    //cv::destroyWindow("Image");
    imgColor.release();
    imgDepth.release();
    imgGray.release();
  }

  this->nh = _nh;

  subImageCount = 0;
  imgColor = Mat(cv::Size(640, 480), CV_8UC3);
  imgDepth = Mat(cv::Size(640, 480), CV_32FC1);
  imgGrayPrevious = Mat(cv::Size(640, 480), CV_8UC1);
  imgGray = Mat(cv::Size(640, 480), CV_8UC1);

  // Create some intrinsic parameters for the kinect.
  // TODO: Pull these from the param server or something
  memset(&rgb_params_, 0, sizeof(fovis::CameraIntrinsicsParameters));
  width = 640;
  height = 480;
  rgb_params_.width = width;
  rgb_params_.height = height;
  rgb_params_.fx = 528.49404721; 
  rgb_params_.fy = rgb_params_.fx;
  rgb_params_.cx = width / 2.0;
  rgb_params_.cy = height / 2.0;

  // get the RGB camera parameters of our device
  rect = new fovis::Rectification(rgb_params_);
  fovis::VisualOdometryOptions options = fovis::VisualOdometry::getDefaultOptions();
  // If we wanted to play around with the different VO parameters, we could set
  // them here in the "options" variable.

  // setup the visual odometry
  odom = new fovis::VisualOdometry(rect, options);
  depth_image_ = new fovis::DepthImage(rgb_params_, width, height);

  //subscribe to the image
  //subImage = nh.subscribe("/camera/rgb/image_rect", 10, &kinectNode::subImageCallback, this);
  //subImage = nh.subscribe("/camera/depth/image_rect", 10, &kinectNode::subImageCallback, this);

  //Create a cv window to view image in for now
  sync_subs.registerCallback(boost::bind(&kinectNode::subImageCallback, this, _1, _2));		

  cv::namedWindow("Image");
}

  ~kinectNode() {
    cv::destroyWindow("Image");
    imgColor.release();
    imgDepth.release();
    imgGray.release();
  }

std::string  isometryToString(const Eigen::Isometry3d& m)
{
  char result[80];
  memset(result, 0, sizeof(result));
  Eigen::Vector3d xyz = m.translation();
  Eigen::Vector3d rpy = m.rotation().eulerAngles(0, 1, 2);
  snprintf(result, 79, "%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f", 
	   xyz(0), xyz(1), xyz(2), 
	   rpy(0) * 180/M_PI, rpy(1) * 180/M_PI, rpy(2) * 180/M_PI);
  return std::string(result);
}



private:

void subImageCallback(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_depth) {

  std::cout << "GOT A NEW DEPTH IMAGE: " << msg_depth->encoding << " !!!!!!!!!!!!!!!!!!!!!!!" << std::endl;


  std::cout << "Image Callback: " << msg_rgb->header.stamp << " | " << msg_depth->header.stamp << std::endl;

  subImageCount++;
  cout << "Got image number: " << subImageCount << endl;
			
                        
  // convert image from ROS image message to OpenCV Mat
  // convert_rgb_image(msg, imgColor);
			
  convert_depth_image(msg_depth, imgDepth);
  convert_rgb_image(msg_rgb, imgColor);


  cv::imshow("rgb image", imgColor);
  cv::waitKey(1);

  cv::imshow("depth image", imgDepth);
  cv::waitKey(1);

  cvtColor(imgColor, imgGray, CV_RGB2GRAY);
			


  //cv::imshow("rgb image", imgColor);
  //cv::waitKey(1);

  //cv::imshow("depth image", imgDepth);
  //cv::waitKey(1);

  cvtColor(imgColor, imgGray, CV_RGB2GRAY);

  // float* depth_data = new float[width*height];
  // depth_data = reinterpret_cast<float*>(imgDepth.data);
  // for( int i=0; i<width*height; i++)
  // {
  //     depth_data[i] = depth_data[i]!=0 ? depth_data[i]*1e-3 : NAN;
  // }

  depth_image_->setDepthImage(reinterpret_cast<float*>(imgDepth.data));
  //depth_image_->setDepthImage((depth_data));
  odom->processFrame(imgGray.data, depth_image_);

  Eigen::Isometry3d cam_to_local = odom->getPose();
  Eigen::Isometry3d motion_estimate = odom->getMotionEstimate();

  Eigen::Quaterniond quat(cam_to_local.rotation());
  Eigen::Vector3d trans(cam_to_local.translation());

  tf::StampedTransform transform;

  btQuaternion btquat;
  btquat.setX(quat.x());
  btquat.setY(quat.y());
  btquat.setZ(quat.z());
  btquat.setW(quat.w());

  btVector3 bttranslation;
  bttranslation.setX(trans.x());
  bttranslation.setY(trans.y());
  bttranslation.setZ(trans.z());

  btTransform btTrans(btquat, bttranslation);

  tf::StampedTransform stampedtrans(btTrans, ros::Time::now(), "/world", "/fovis_frame");
  tf_broadcaster_.sendTransform(stampedtrans);


  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = "kinect";
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.pose.pose.position.x = cam_to_local.translation().x();
  odom_msg.pose.pose.position.y = cam_to_local.translation().y();
  odom_msg.pose.pose.position.z = cam_to_local.translation().z();

  odom_msg.pose.pose.orientation.w = quat.w();
  odom_msg.pose.pose.orientation.x = quat.x();
  odom_msg.pose.pose.orientation.y = quat.y();
  odom_msg.pose.pose.orientation.z = quat.z();

  odom_pub_.publish(odom_msg);

  std::cout << isometryToString(cam_to_local) << " " << isometryToString(motion_estimate) << "\n";
  //delete[] depth_data;

}

void convert_rgb_image(const sensor_msgs::ImageConstPtr& _msg, Mat& _img)
{
  cv_bridge::CvImageConstPtr cv_image_ptr;
  cv_image_ptr = cv_bridge::toCvShare(_msg, "bgr8");
  //cv_image_ptr = cv_bridge::toCvCopy(_msg, enc::TYPE_32FC1);
  _img = cv_image_ptr->image;

}

void convert_depth_image(const sensor_msgs::ImageConstPtr& _msg, Mat& _img)
{
  cv_bridge::CvImageConstPtr cv_image_ptr;
  //cv_image_ptr = cv_bridge::toCvShare(_msg, "bgr8");
  cv_image_ptr = cv_bridge::toCvCopy(_msg, enc::TYPE_32FC1);
  _img = cv_image_ptr->image;
}


// for( int i=0; i<width*height; i++)
// {
//     float x = reinterpret_cast<float*>(imgDepth.data)[i];
//     reinterpret_cast<float*>(imgDepth.data)[i] = x!=0 ? x : NAN;
// }

depth_image_->setDepthImage(reinterpret_cast<float*>(imgDepth.data));
odom->processFrame(imgGray.data, depth_image_);

Eigen::Isometry3d cam_to_local = odom->getPose();
Eigen::Isometry3d motion_estimate = odom->getMotionEstimate();

std::cout << isometryToString(cam_to_local) << " " << isometryToString(motion_estimate) << "\n";
}

void convert_rgb_image(const sensor_msgs::ImageConstPtr& _msg, Mat& _img)
{
  cv_bridge::CvImageConstPtr cv_image_ptr;
  cv_image_ptr = cv_bridge::toCvShare(_msg, "bgr8");
  //cv_image_ptr = cv_bridge::toCvCopy(_msg, enc::TYPE_32FC1);
  _img = cv_image_ptr->image;

}

void convert_depth_image(const sensor_msgs::ImageConstPtr& _msg, Mat& _img)
{
  cv_bridge::CvImageConstPtr cv_image_ptr;
  //cv_image_ptr = cv_bridge::toCvShare(_msg, "bgr8");
  cv_image_ptr = cv_bridge::toCvCopy(_msg, enc::TYPE_32FC1);
  _img = cv_image_ptr->image;
}

	
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flash_drive_finder");
  ros::NodeHandle nh;

  ROS_INFO_STREAM("Initializing kinectNode...");
  kinectNode kinectNode(nh);
  cout << "Initialization Complete" << endl;

  ros::spin();
  return 0;
}

