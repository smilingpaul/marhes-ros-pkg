#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "vicon_sdk/Client.h"

using namespace ViconDataStreamSDK::CPP;

class ViconToPose
{
public:
  ViconToPose(ros::NodeHandle nh);
private:
  ros::NodeHandle n_;
  ros::Publisher pose_pub_;
  double freq_;
  std::string subject_, server_, subjectName_, rootSegment_;
  ros::Timer calc_tmr_;
  Client * vicon_client_;
  
  void ViconTmr(const ros::TimerEvent& event);
};

ViconToPose::ViconToPose(ros::NodeHandle nh)
{
  n_ = nh;
  ros::NodeHandle n_private("~");
  n_private.param("freq", freq_, 100.0);
  n_private.param("server", server_, std::string("Vicon"));
  
  if (!n_private.hasParam("subject"))
  {
    ROS_FATAL("Please specify target frame to track!");
    exit(1);
  }
  
  // Get the subject to track
  n_private.getParam("subject", subject_);
  ROS_INFO("Tracking subject: %s", subject_.c_str());
  
  vicon_client_ = new Client();
  
  // Try to connect to the server
  while(ros::ok() && !vicon_client_.IsConnected().Connected)
  {
    ROS_INFO("In connect loop");
    vicon_client_.Connect(server_);
    ROS_INFO("Waiting to connect.");
    ros::Duration(1).sleep();
  }
  ROS_INFO("Connected to Vicon Server at %s", server_.c_str());
  
  vicon_client_.EnableSegmentData();
  vicon_client_.DisableMarkerData();
  vicon_client_.DisableUnlabeledMarkerData();
  vicon_client_.DisableDeviceData();
   
  vicon_client_.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPull );
  //vicon_client_.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch );
  //vicon_client_.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ServerPush );
  
  // Set the global up axis
  vicon_client_.SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up);
  
  unsigned int subjectCount = vicon_client_.GetSubjectCount().SubjectCount;
  for( unsigned int subjectIndex = 0 ; subjectIndex < subjectCount ; ++subjectIndex )
  {
    // Get the subject name
    std::string subjectName = vicon_client_.GetSubjectName(subjectIndex).SubjectName;
    // Get the root segment
    std::string rootSegment = vicon_client_.GetSubjectRootSegmentName(subjectName).SegmentName;
    
    if (subjectName == subject_)
    {
      subjectName_ = subjectName;
      rootSegment_ = rootSegment;
      break;
    }
  }
  
  pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("pose", 1);
  calc_tmr_ = n_.createTimer(ros::Duration(1/freq_), &ViconToPose::ViconTmr, this);
}

void ViconToPose::ViconTmr(const ros::TimerEvent& event)
{
  if(vicon_client_.GetFrame().Result != Result::Success)
  {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/odom";
  
    Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation = 
        vicon_client_.GetSegmentGlobalTranslation(subjectName_, rootSegment_);
    msg.pose.position.x = _Output_GetSegmentGlobalTranslation.Translation[0];
    msg.pose.position.y = _Output_GetSegmentGlobalTranslation.Translation[1];
    msg.pose.position.z = _Output_GetSegmentGlobalTranslation.Translation[2];
    
    Output_GetSegmentGlobalRotationQuaternion _Output_GetSegGlobalQuaternion = 
        vicon_client_.GetSegmentGlobalRotationQuaternion(subjectName_, rootSegment_);
    msg.pose.orientation.x = _Output_GetSegGlobalQuaternion.Rotation[0];
    msg.pose.orientation.y = _Output_GetSegGlobalQuaternion.Rotation[1];
    msg.pose.orientation.z = _Output_GetSegGlobalQuaternion.Rotation[2];
    msg.pose.orientation.w = _Output_GetSegGlobalQuaternion.Rotation[3];
    
    pose_pub_.publish(msg);
  }
}

int main(int argc, char **argv)
{
  // Initial ROS setup
	ros::init(argc, argv, "vicon_to_pose");
	ros::NodeHandle n;

  ViconToPose* translater = new ViconToPose(n);

  ros::spin();

  return 0;
}
