#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "cv_bridge/CvBridge.h"
#include <math.h>

using namespace std;

class TFToOdom
{
public:
  TFToOdom(void)
  {
	ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    n_private.param("odom_frame", odom_frame_, string("/odom"));
    n_private.param("odom_topic", odom_topic_, string("/vo"));
    n_private.param("odom_filter_topic", odom_filter_topic_, string("/vo_filter"));
    n_private.param("freq", freq_, 50.0);
//    n_private.param("bool_tf", bool_tf_, true);

    if (!n_private.hasParam("frame"))
      ROS_FATAL("Error: Parameter \"frame\" was not provided");
    else
      n_private.getParam("frame", frame_);

    ROS_INFO("%s", frame_.c_str());
    dt_ = 1 / freq_;
    last_theta_ = 0.0;

    z_ = cvCreateMat(6,1,CV_32FC1);
    kalman_ = cvCreateKalman(6, 6, 0); // 6 states [x y th dx dy dth]
                                       // 3 measurements [x y th]
                                       // 0 control inputs

    const float transitionMat[] = {1, 0, 0, dt_, 0, 0,
                                   0, 1, 0, 0, dt_, 0,
                                   0, 0, 1, 0, 0, dt_,
                                   0, 0, 0, 1, 0, 0,
                                   0, 0, 0, 0, 1, 0,
                                   0, 0, 0, 0, 0, 1};
    memcpy(kalman_->transition_matrix->data.fl, transitionMat, sizeof(transitionMat));
    PrintMat(kalman_->transition_matrix);

    const float measurementMat[] = {1, 0, 0, 0, 0, 0,
    		                        0, 1, 0, 0, 0, 0,
    		                        0, 0, 1, 0, 0, 0,
    		                        0, 0, 0, 0, 0, 0,
    		                        0, 0, 0, 0, 0, 0,
    		                        0, 0, 0, 0, 0, 0};
    memcpy(kalman_->measurement_matrix->data.fl, measurementMat, sizeof(measurementMat));
    PrintMat(kalman_->measurement_matrix);

    const float processNoiseMat[] = {0.1, 0, 0, 0, 0, 0,
    		                         0, 0.1, 0, 0, 0, 0,
    		                         0, 0, 0.1, 0, 0, 0,
    		                         0, 0, 0, 5, 0, 0,
    		                         0, 0, 0, 0, 5, 0,
    		                         0, 0, 0, 0, 0, 5};
    memcpy(kalman_->process_noise_cov->data.fl, processNoiseMat, sizeof(processNoiseMat));
    PrintMat(kalman_->process_noise_cov);

    const float measNoiseMat[] = {0.01, 0, 0, 0, 0, 0,
    		                      0, 0.01, 0, 0, 0, 0,
    		                      0, 0, 0.01, 0, 0, 0,
    		                      0, 0, 0, 1000, 0, 0,
    		                      0, 0, 0, 0, 1000, 0,
    		                      0, 0, 0, 0, 0, 1000};
    memcpy(kalman_->measurement_noise_cov->data.fl, measNoiseMat, sizeof(measNoiseMat));
    PrintMat(kalman_->measurement_noise_cov);

    const float postErrorCovMat[] = {1, 0, 0, 0, 0, 0,
    		                         0, 1, 0, 0, 0, 0,
    		                         0, 0, 1, 0, 0, 0,
    		                         0, 0, 0, 1, 0, 0,
    		                         0, 0, 0, 0, 1, 0,
    		                         0, 0, 0, 0, 0, 1};
    memcpy(kalman_->error_cov_post->data.fl, postErrorCovMat, sizeof(postErrorCovMat));
    PrintMat(kalman_->error_cov_post);

    const float init_state[] = {0, 0, 0, 0, 0, 0};
    memcpy(kalman_->state_post->data.fl, init_state, sizeof(init_state));
    PrintMat(kalman_->state_post);

    odom_pub_ = n.advertise<nav_msgs::Odometry>(odom_topic_.c_str(), 50);
    odom_filter_pub_ = n.advertise<nav_msgs::Odometry>(odom_filter_topic_.c_str(), 50); 
    timer_ = n.createTimer(ros::Duration(1/freq_), &TFToOdom::UpdateOdom, this);
  }

  void PrintMat(CvMat* mat)
  {
	string s;
	for(int i = 0; i < mat->rows; i++)
	{
		for(int j = 0; j < mat->cols; j++)
		{
			s.append(boost::lexical_cast<std::string>(cvmGet(mat, i, j)) + ", ");
		}
		s.append("\n");
	}
	ROS_INFO("Mat: \n%s", s.c_str());
  }

  void UpdateOdom(const ros::TimerEvent& event)
  {
    tf::StampedTransform transform;
    double current_theta, last_theta, d_theta;  //double vx, vtheta, dt;

    try
    {
      listener_.lookupTransform(odom_frame_, frame_, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }

    dt_ = transform.stamp_.toSec() - last_transform_.stamp_.toSec();

    if (dt_ > 0)
    {
        cvmSet(z_, 0, 0, (float)transform.getOrigin().getX());
        cvmSet(z_, 1, 0, (float)transform.getOrigin().getY());

        // Figure out theta
        current_theta = transform.getRotation();
        last_theta = cvmGet(kalman_->state_post, 2, 0);
        if (current_theta > 0 && )
        	d_theta = tf::getYaw(transform.getRotation()) - ;
        theta_ += d_theta;
        cvmSet(z_, 2, 0, (float)tf::getYaw(transform.getRotation()));
        cvmSet(z_, 3, 0, 0);
        cvmSet(z_, 4, 0, 0);
        cvmSet(z_, 5, 0, 0);
        ROS_INFO("yaw: %f", cvmGet(z_, 2, 0));
		const float transitionMat[] = {1, 0, 0, dt_, 0, 0,
									   0, 1, 0, 0, dt_, 0,
									   0, 0, 1, 0, 0, dt_,
									   0, 0, 0, 1, 0, 0,
									   0, 0, 0, 0, 1, 0,
									   0, 0, 0, 0, 0, 1};
		memcpy(kalman_->transition_matrix->data.fl, transitionMat, sizeof(transitionMat));
//		PrintMat(kalman_->transition_matrix);

		cvKalmanPredict(kalman_, 0);
		cvKalmanCorrect(kalman_, z_);

		nav_msgs::Odometry odom_msg;
		//next, we'll publish the odometry message over ROS
		odom_msg.header.stamp = transform.stamp_;
		odom_msg.header.frame_id = transform.frame_id_;
		odom_msg.child_frame_id = transform.child_frame_id_;

		//set the position
		odom_msg.pose.pose.position.x = (double)cvmGet(kalman_->state_post, 0, 0);
		odom_msg.pose.pose.position.y = (double)cvmGet(kalman_->state_post, 1, 0);
		odom_msg.pose.pose.position.z = 0.0;
		odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(
				(double)cvmGet(kalman_->state_post, 2, 0));
	//    odom_msg.pose.pose.orientation.y = transform.getRotation().y();
	//    odom_msg.pose.pose.orientation.z = transform.getRotation().z();
	//    odom_msg.pose.pose.orientation.w = transform.getRotation().w();

		odom_msg.pose.covariance[0] = 4.5;
		odom_msg.pose.covariance[7] = 4.5;
		odom_msg.pose.covariance[14] = 99999;
		odom_msg.pose.covariance[21] = 99999;
		odom_msg.pose.covariance[28] = 99999;
		odom_msg.pose.covariance[35] = 0.75;

		//set the velocity
		odom_msg.twist.twist.linear.x = sqrt(
				pow((double)cvmGet(kalman_->state_post, 3, 0), 2) +
				pow((double)cvmGet(kalman_->state_post, 4, 0), 2));
		odom_msg.twist.twist.linear.y = 0.0;
		odom_msg.twist.twist.linear.z = 0.0;
		odom_msg.twist.twist.angular.z = (double)cvmGet(kalman_->state_post, 5, 0);

		//publish the message
		odom_pub_.publish(odom_msg);
		last_transform_ = transform;
    }
  }
private:
  ros::NodeHandle n_;
  ros::Publisher odom_pub_, odom_filter_pub_;
  ros::Timer timer_;
  tf::TransformListener listener_;
  tf::StampedTransform last_transform_;
  string frame_, odom_frame_, odom_topic_, odom_filter_topic_;
  float dt_;
  double freq_, last_theta_;
//  bool bool_tf_;
  CvKalman* kalman_;
  CvMat* z_;

};

int main(int argc, char **argv)
{
  // Initial ROS setup
	ros::init(argc, argv, "vicon_tf_to_odom");

  TFToOdom* translater = new TFToOdom();

  ros::spin();

  return 0;
}
