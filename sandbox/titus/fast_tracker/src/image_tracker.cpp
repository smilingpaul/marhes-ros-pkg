/*
 * imageTracker.cpp
 *
 *  Created on: Nov 10, 2010
 *      Author: Titus Appel
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/Image.h>

using namespace cv;
using namespace std;

image_transport::Publisher pub;
Mat prevImage, prevDescriptors;
vector<KeyPoint> prevPoints;
bool firstTime = true;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	sensor_msgs::CvBridge bridge;
	IplImage* cvInImage = NULL;
	IplImage  cvOutImage;
	Mat matImage, descriptors;
	FastFeatureDetector fast(20, true);
	vector<KeyPoint> points;
	//CalonderDescriptorMatch matcher;
	vector<int> matches;
	SurfDescriptorExtractor extractor;
	BruteForceMatcher<L2<float> > matcher;

	try
	{
		cvInImage = bridge.imgMsgToCv(msg, "mono8");
	}
	catch (sensor_msgs::CvBridgeException& e)
	{
	    ROS_ERROR("Could not convert from '%s' to 'IplImage'.", msg->encoding.c_str());
	}

	matImage = cvInImage;

	ros::Time first = ros::Time::now();

	// Detect Keypoints in the image
	fast.detect(matImage, points);
	int numPoints = points.size();
	ROS_INFO("Keypoints: %i", numPoints);


	// Compute SURF descriptors
	extractor.compute(matImage, points, descriptors);
	ros::Duration diff = ros::Time::now() - first;
	ROS_INFO("%i", diff.nsec);
	// If first time just store the image and the keypoints, else match the
	// keypoints between the two images
//	if (!firstTime)
//	{
////		ROS_INFO("Training the descriptors");
////		// Train the previous image and keypoints
////		matcher.add(prevImage, prevPoints);
////		matcher.add(matImage, points);
//
////		ROS_INFO("Matching the keypoints");
////		// Match these keypoints with the previous keypoints
////		matcher.match(matImage, points, indices);
//
//		// matching descriptors
//		matcher.add(prevDescriptors);
//		matcher.match(descriptors, matches);  // matches is size of numPoints
//	}
//	else
//		firstTime = false;

	//int numMatches = matches.size();

	// Store this image and its keypoints
	prevImage = matImage;
	prevPoints = points;
	prevDescriptors = descriptors;
	//ROS_INFO("Stored the images");


	// Start drawing results
	cvtColor(cvInImage, matImage, CV_GRAY2RGB);

//	for(int i = 0; i < numMatches; i++)
//	{
//		ROS_INFO("Match %i: %i", i, matches[i]);
//		// Draw lines between matches
//		cv::line(matImage,points[i].pt,prevPoints[matches[i]].pt,CV_RGB(0,255,0),1,8,0);
//	}



	for(int i = 0; i < numPoints; i++)
	{

		cv::circle(matImage, points[i].pt, 2, CV_RGB(255,0,0), -1, 8, 0);

	}

	// Convert to image to publish
	cvOutImage = matImage;

	try
	{
		pub.publish(bridge.cvToImgMsg(&cvOutImage, "bgr8"));
	}
	catch (sensor_msgs::CvBridgeException& e)
	{
	    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", cvOutImage.colorModel);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imageTracker");
	ros::NodeHandle n;
//	ros::Rate loop_rate(30);
	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub;

	try
	{
		sub = it.subscribe("camera/image_raw", 1, imageCallback);
	}
	catch(image_transport::TransportLoadException& e)
	{
		ROS_ERROR("%s", e.what());
	}

	try
	{
		pub = it.advertise("my_camera", 1);
	}
	catch(image_transport::TransportLoadException& e)
	{
		ROS_ERROR("%s", e.what());
	}

	ros::spin();

//	while(n.ok())
//	{
//
//
//		ros::spinOnce();
//		loop_rate.sleep();
//	}
	return 0;
}

